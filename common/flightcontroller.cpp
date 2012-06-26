#include "flightcontroller.h"
#include "motioncommand.h"

FlightController::FlightController() : QObject(), mFlightState(ManualControl)
{
    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mTimeOfLastControllerUpdate = QTime::currentTime();
    mFirstControllerRun = true;

    mBackupTimerComputeMotion = new QTimer(this);
    connect(mBackupTimerComputeMotion, SIGNAL(timeout()), SLOT(slotComputeBackupMotion()));

    // Tests have shown:
    // - with metal hood (2425 gram) it hovers at 127/128.
    mThrustHover = 127.0;

    /*// For testing in simulator
    mWayPoints.append(WayPoint(QVector3D(130,90,110)));
    mWayPoints.append(WayPoint(QVector3D(140,80,130)));
    mWayPoints.append(WayPoint(QVector3D(120,90,120)));
    mWayPoints.append(WayPoint(QVector3D(150,80,150)));
    setFlightState(ApproachingNextWayPoint);*/
}

FlightController::~FlightController()
{

}

void FlightController::slotComputeBackupMotion()
{
    qDebug() << t() << "FlightController::slotComputeBackupMotion(): timer has fired, so there was no GNSS pose in" << mBackupTimerComputeMotion->interval() << "ms! Starting backup motion computation.";
    slotComputeMotionCommands();
}

void FlightController::slotComputeMotionCommands()
{
    // Elapsed time since last call in seconds. May not become zero (divide by zero)
    // and shouldn't grow too high, as that will screw up the controllers.
    const float timeDiff = qBound(
                0.01f,
                (float)(mTimeOfLastControllerUpdate.msecsTo(QTime::currentTime()) / 1000.0f),
                0.2f);

    quint8 out_thrust = 0;
    qint8 out_yaw, out_pitch, out_roll = 0;

    // TODO: Do we still need this flightstate? how do we detect that manual control is activated?
    if(mFlightState == ManualControl)
    {
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: ManualControl, no motion emitted.";
    }
    else if(mFlightState == Freezing)
    {
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: Freezing. PANIC! Emitting safeControlValues()";

        // Emit safe-hover-values
        emitSafeControlValues();
    }
    else if(mFlightState == ApproachingNextWayPoint)
    {
        if(mWayPoints.size() < 1)
        {
            qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: ApproachingNextWayPoint: Cannot approach, no waypoints present!";
            emitSafeControlValues();
            return;
        }

        if(getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp > 82)
        {
            qDebug() << t() << "FlightController::slotComputeMotionCommands(): ApproachingNextWayPoint, vehicle pose update is" << getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp << "ms ago, skipping motion computation, emitting safe control values";
            emitSafeControlValues();
            return;
        }

        const WayPoint nextWayPoint = mWayPoints.first();

        const QVector2D vectorVehicleToWayPoint = (nextWayPoint.getPositionOnPlane() - mLastKnownVehiclePose.getPlanarPosition());
        const float directionNorthToWayPointRadians = atan2(-vectorVehicleToWayPoint.x(), -vectorVehicleToWayPoint.y());
        const float angleToTurnToWayPoint = Pose::getShortestTurnRadians(directionNorthToWayPointRadians - mLastKnownVehiclePose.getYawRadians());

//        qDebug() << "mLastKnownVehiclePose.getPlanarPosition()" << mLastKnownVehiclePose.getPlanarPosition();
//        qDebug() << "nextWayPoint.getPositionOnPlane():" << nextWayPoint.getPositionOnPlane();
        qDebug() << "FlightController::slotComputeMotionCommands(): LastKnownPose:" << mLastKnownVehiclePose << "NextWayPoint:" << nextWayPoint;
        qDebug() << "directionNorthToWayPoint:" << RAD2DEG(directionNorthToWayPointRadians) << "angleToTurnToWayPoint: turn" << (angleToTurnToWayPoint < 0 ? "right" : "left") << RAD2DEG(angleToTurnToWayPoint);

        // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!

        // If the planar distance to the next waypoint is very small (this happens when only the height is off),
        // we don't want to yaw and pitch. So, we introduce a factor [0;1], which becomes 0 with small distance
        const float factorPlanarDistance = qBound(0.0f, (float)(mLastKnownVehiclePose.getPlanarPosition() - Pose::getPlanarPosition(nextWayPoint)).length() * 2.0f, 1.0f);

        // If the height very small (this happens during liftoff and landing),
        // we don't want to yaw and pitch. So, we introduce a factor [0;1], which becomes 0 with small distance
        // We don't check for mLastKnownHeightOverGroundTimestamp, as an old value means that the scanner hasn't
        // seen anything for a long time, which SHOULD mean we're high up.
        const float factorHeight = qBound(0.0f, mLastKnownHeightOverGround, 1.0f);

        qDebug() << "timediff:" << timeDiff << "factorPlanarDistance" << factorPlanarDistance << "factorHeight" << factorHeight;

        // If angleToTurnToWayPoint is:
        // - positive, we need to rotate CCW, which needs a negative yaw value.
        // - negative, we need to rotate  CW, which needs a positive yaw value.
        const float errorYaw = RAD2DEG(angleToTurnToWayPoint);
        mErrorIntegralYaw += errorYaw*timeDiff;
        const float derivativeYaw = mFirstControllerRun ? 0.0f : (errorYaw - mPrevErrorYaw + 0.00001f)/timeDiff;
        const float outputYaw = factorHeight * factorPlanarDistance * ((1.0f * errorYaw) + (0.0f * mErrorIntegralYaw) + (0.3f/*0.5f*/ * derivativeYaw));

        // adjust pitch/roll to reach target, maximum pitch is -20 degrees (forward)
        float desiredRoll = 0.0f;
        float desiredPitch = -pow(20.0f - qBound(0.0, fabs(errorYaw), 20.0), 2.0f) / 20.0f;

        // try to get ourselves straight up
        const float errorPitch = desiredPitch - mLastKnownVehiclePose.getPitchDegrees();
        mErrorIntegralPitch += errorPitch*timeDiff;
        const float derivativePitch = mFirstControllerRun ? 0.0f : (errorPitch - mPrevErrorPitch + 0.00001f)/timeDiff;
        // WARNING: If we multiply by factorHeight, we won't stabilize the kopter at low height, it'll have to do that by itself. Is that good?
        const float outputPitch = factorHeight * factorPlanarDistance * ((6.0f * errorPitch) + (0.3f * mErrorIntegralPitch) + (0.0f * derivativePitch));

        const float errorRoll = desiredRoll - mLastKnownVehiclePose.getRollDegrees();
        mErrorIntegralRoll += errorRoll*timeDiff;
        const float derivativeRoll = mFirstControllerRun ? 0.0f : (errorRoll - mPrevErrorRoll + 0.00001f)/timeDiff;
        // WARNING: If we multiply by factorHeight, we won't stabilize the kopter at low height, it'll have to do that by itself. Is that good?
        const float outputRoll = factorHeight * factorPlanarDistance * ((6.0f * errorRoll) + (0.3f * mErrorIntegralRoll) + (0.0f * derivativeRoll));

        const float errorHeight = nextWayPoint.y() - mLastKnownVehiclePose.getPosition().y();
        mErrorIntegralHeight += errorHeight*timeDiff;

        // We do need to use the I-controller, but we should clear the integrated error once we have crossed the height of a waypoint.
        // Otherwise, the integral will grow large while ascending and then keep the kopter above the waypoint for a loooong time.
        if((mPrevErrorHeight > 0.0f && errorHeight < 0.0f) || (mPrevErrorHeight < 0.0f && errorHeight > 0.0f)) mErrorIntegralHeight /= 3.0f;

        const float derivativeHeight = mFirstControllerRun ? 0.0f : (errorHeight - mPrevErrorHeight + 0.00001f)/timeDiff;
        const float outputThrust = mThrustHover + (25.0f * errorHeight) + (0.01f * mErrorIntegralHeight) + (1.0f * derivativeHeight);

        qDebug() << mWayPoints.size() << "waypoints, next wpt height" << nextWayPoint.y() << "curr height" << mLastKnownVehiclePose.getPosition().y();

        qDebug() << "values PRYH:" << QString::number(mLastKnownVehiclePose.getPitchDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getRollDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getYawDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getPosition().y(), 'f', 2);
        qDebug() << "should PRYH:" << QString::number(desiredPitch, 'f', 2) << "\t" << QString::number(desiredRoll, 'f', 2) << "\t" << QString::number(RAD2DEG(directionNorthToWayPointRadians), 'f', 2) << "\t" << QString::number(nextWayPoint.y(), 'f', 2);
        qDebug() << "error  PRYH:" << QString::number(errorPitch, 'f', 2) << "\t" << QString::number(errorRoll, 'f', 2) << "\t" << QString::number(errorYaw, 'f', 2) << "\t" << QString::number(errorHeight, 'f', 2);
        qDebug() << "derivt PRYH:" << QString::number(derivativePitch, 'f', 2) << "\t" << QString::number(derivativeRoll, 'f', 2) << "\t" << QString::number(derivativeYaw, 'f', 2) << "\t" << QString::number(derivativeHeight, 'f', 2);
        qDebug() << "prevEr PRYH:" << QString::number(mPrevErrorPitch, 'f', 2) << "\t" << QString::number(mPrevErrorRoll, 'f', 2) << "\t" << QString::number(mPrevErrorYaw, 'f', 2) << "\t" << QString::number(mPrevErrorHeight, 'f', 2);
        qDebug() << "inteEr PRYH:" << QString::number(mErrorIntegralPitch, 'f', 2) << "\t" << QString::number(mErrorIntegralRoll, 'f', 2) << "\t" << QString::number(mErrorIntegralYaw, 'f', 2) << "\t" << QString::number(mErrorIntegralHeight, 'f', 2);
        qDebug() << "output PRYH:" << QString::number(outputPitch, 'f', 2) << "\t" << QString::number(outputRoll, 'f', 2) << "\t" << QString::number(outputYaw, 'f', 2) << "\t" << QString::number(outputThrust, 'f', 2);

        mPrevErrorPitch = errorPitch;
        mPrevErrorRoll = errorRoll;
        mPrevErrorYaw = errorYaw;
        mPrevErrorHeight = errorHeight;

        MotionCommand mc;

        mc.thrust = (quint8)qBound(100.0f, outputThrust, 140.0f);
        // A yaw of 15 rotates by about 15 degrees per second.
        mc.yaw = (qint8)qBound(-25.0, outputYaw > 0.0f ? ceil(outputYaw) : floor(outputYaw), 25.0);
        // 20 seems a good pitch/roll-value in production, but lets limit to 10 for testing
        mc.pitch = (qint8)qBound(-10.0f, outputPitch, 10.0f);
        mc.roll = (qint8)qBound(-10.0f, outputRoll, 10.0f);

        // For safety, we don't need it right now.
        mc.roll = 0;

        qDebug() << t() << "FlightController::slotComputeMotionCommands(): thrust:" << mc.thrust << "yaw:" << mc.yaw << "pitch:" << mc.pitch << "roll:" << mc.roll;

        emit motion(mc);
        //emit debugValues(mLastKnownVehiclePose, mc);

        // See whether we've reached the waypoint
        if(mLastKnownVehiclePose.getPosition().distanceToLine(nextWayPoint, QVector3D()) < 1.00f) // close to wp
        {
            nextWayPointReached();
        }

        mFirstControllerRun = false;
    }
    else if(mFlightState == Idle)
    {
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): flightstate idle, emitting idle thrust.";
        emit motion(MotionCommand(90));
    }
    else
    {
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FLIGHTSTATE NOT DEFINED:" << mFlightState;
    }

    mTimeOfLastControllerUpdate = QTime::currentTime();
}

void FlightController::nextWayPointReached()
{
    // The next waypoint has been reached.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    qDebug() << "FlightController::nextWayPointReached(): reached waypoint" << mWayPointsPassed.last();

    if(getFlightState() != ApproachingNextWayPoint)
        qDebug() << "FlightController::nextWayPointReached(): reached waypoint" << mWayPointsPassed.last() << "but am not in ApproachingNextWayPoint state. Expect trouble!";

    // First emit this signal for the base to sync the list, THEN call ensureSafeFlightAfterWaypointsChanged(),
    // which might append more waypoints.
    emit wayPointReached(mWayPointsPassed.last());

    ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(mWayPoints);
}

void FlightController::slotWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "FlightController::slotSetNextWayPoint(): state is" << getFlightStateString(mFlightState) << "inserting waypoint" << wayPoint << "into index" << index;
    mWayPoints.insert(index, wayPoint);
    emit currentWayPoints(mWayPoints);

    // Just in case we were idle before...
    if(mFlightState == Idle)
    {
        qDebug() << t() << "FlightController::slotWayPointInsert(): we were idle, switching to ApproachingNextWayPoint";
        setFlightState(ApproachingNextWayPoint);
    }
}

void FlightController::slotWayPointDelete(const quint16& index)
{
    qDebug() << QString("FlightController::slotWayPointDelete(const quint16& index = %1)").arg(index);
    if(mWayPoints.size() <= index)
    {
        qWarning("FlightController::slotWayPointDelete(): couldn't delete waypoint at index %d, list only has %d entries.", index, mWayPoints.size());
    }
    else
    {
        mWayPoints.removeAt(index);

        // The list might now be empty, so me might have to land.
        ensureSafeFlightAfterWaypointsChanged();
    }
    qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, emitting new wpt list of size" << mWayPoints.size();
    emit currentWayPoints(mWayPoints);
}

void FlightController::slotSetWayPoints(const QList<WayPoint>& wayPoints)
{
    mWayPoints = wayPoints;

    // Just in case we were idle before...
    if(mFlightState == Idle && mWayPoints.size())
    {
        qDebug() << t() << "FlightController::slotSetWayPoints(): we were idle, switching to ApproachingNextWayPoint";
        setFlightState(ApproachingNextWayPoint);
    }

    // The list might now be empty, so we might have to land.
    ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(mWayPoints);
}

Pose FlightController::getLastKnownPose(void) const
{
    return mLastKnownVehiclePose;
}

QList<WayPoint> FlightController::getWayPoints()
{
    return mWayPoints;
}

FlightState FlightController::getFlightState(void) const { return mFlightState; }

void FlightController::slotNewVehiclePose(const Pose& pose)
{
    // Note that we don't expect ModeIntegrated
    if(pose.precision & Pose::AttitudeAvailable && pose.precision & Pose::HeadingFixed && pose.precision && Pose::RtkFixed)
    {
        qDebug() << t() << "FlightController::slotNewVehiclePose(): received a usable:" << pose;
        mLastKnownVehiclePose = pose;
    }
    else
    {
        qDebug() << t() << "FlightController::slotNewVehiclePose(): received a useless" << pose.toStringVerbose();
    }

    mBackupTimerComputeMotion->start(50); // re-set the safety timer
    slotComputeMotionCommands();
}

void FlightController::slotHoldPosition()
{
    setFlightState(Freezing);
}

void FlightController::setFlightState(FlightState newFlightState)
{
    if(mFlightState != newFlightState)
    {
        qDebug() << t() << "FlightController::setFlightState():" << getFlightStateString(mFlightState) << "=>" << getFlightStateString(newFlightState);

        if(newFlightState == ApproachingNextWayPoint)
        {
            qDebug() << t() << "FlightController::setFlightState(): ApproachingNextWayPoint - initializing controllers, enabling backup motion timer.";
            // We're going to use the controllers, so make sure to initialize them
            mPrevErrorPitch = 0.1f;
            mPrevErrorRoll = 0.1f;
            mPrevErrorYaw = 0.1f;
            mPrevErrorHeight = 0.1f;

            mErrorIntegralPitch = 0.1f;
            mErrorIntegralRoll = 0.1f;
            mErrorIntegralYaw = 0.1f;
            mErrorIntegralHeight = 0.1;

            // So the that the derivative will be predictable
            mTimeOfLastControllerUpdate = QTime::currentTime();

            mFirstControllerRun = true;
            mBackupTimerComputeMotion->start();
        }

        if(newFlightState == Freezing)
        {
            qDebug() << t() << "FlightController::setFlightState(): freezing - deleting waypoints, emitting empty list, enabling backup motion timer.";
            mWayPoints.clear();
            emit currentWayPoints(mWayPoints);
            mBackupTimerComputeMotion->start();
        }

        if(newFlightState == ManualControl)
        {
            qDebug() << t() << "FlightController::setFlightState(): manual control - disabling backup motion timer.";
            mBackupTimerComputeMotion->stop();
        }

        if(newFlightState == Idle)
        {
            qDebug() << t() << "FlightController::setFlightState(): idle - disabling backup motion timer.";
            mBackupTimerComputeMotion->stop();
        }

        mFlightState = newFlightState;
        emit flightStateChanged(newFlightState);
    }
}

void FlightController::slotSetHeightOverGround(const float& beamLength)
{
    // Height is given from vehicle center to ground, but we care about the bottom of the landing gear.
    qDebug() << t() << "FlightController::slotSetHeightOverGround()" << beamLength - 0.16f << "meters";
    mLastKnownHeightOverGroundTimestamp = QTime::currentTime();
    mLastKnownHeightOverGround = beamLength - 0.16f;
}

bool FlightController::isHeightOverGroundValueRecent() const
{
    qDebug() << t() << "FlightController::isHeightOverGroundValueRecent(): age of last heightOverGround measurement is" << mLastKnownHeightOverGroundTimestamp.msecsTo(QTime::currentTime()) << "msecs";
    return mLastKnownHeightOverGroundTimestamp.msecsTo(QTime::currentTime()) < 750;
}

// To be called after waypoints have changed to check for dangerous states:
// If wpt list is empty and state is ApproachingNextWayPoint, will either
//  - idle if low
//  - get a landing waypoint if heightOverGround is known and up-to-date
//  - descend slowly if heightOverGround is unknown
void FlightController::ensureSafeFlightAfterWaypointsChanged()
{
    mFirstControllerRun = true; // to tame the derivatives

    if(mWayPoints.size() == 0 && mFlightState == ApproachingNextWayPoint)
    {
        // The list is now empty and we are flying. Insert a landing-waypoint if we're not close to the ground;
        if(isHeightOverGroundValueRecent() && mLastKnownHeightOverGround < 0.3f)
        {
            // We're low anyway, just got to idle mode.
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, we're low with valid heightOverGround of" << mLastKnownHeightOverGround << ", idling";
            setFlightState(Idle);
        }
        else if(isHeightOverGroundValueRecent())
        {
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround is known to be" << mLastKnownHeightOverGround << "inserting landing wpt 0.2m above";
            mWayPoints.append(WayPoint(mLastKnownVehiclePose.getPosition() - QVector3D(0.0, mLastKnownHeightOverGround - 0.2, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
        else
        {
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround unknown, WARNING, next wpt is 0.2m below us.";
            mWayPoints.append(WayPoint(mLastKnownVehiclePose.getPosition() - QVector3D(0.0, 0.2, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
    }
}


void FlightController::emitSafeControlValues()
{
    qDebug() << "FlightController::emitSafeControlValues()";
    // A value of mThrustHover * 1.05f should make the kopter rise slowly. Thats great, because that means we
    // can control thrust ourselves by defining the upper thrust bound with the remote control.
    emit motion(MotionCommand(mThrustHover*1.05f));
}

void FlightController::slotExternalControlStatusChanged(bool computerControlActive)
{
    qDebug() << t() << "FlightController::slotExternalControlStatusChanged(): computer control changed to:" << computerControlActive;

    if(mFlightState == Freezing)
    {
        qDebug() << t() << "FlightController::slotExternalControlStatusChanged(): computer control active:" << computerControlActive << "but we're FREEZING, won't change flightState";
        return;
    }

    // We might be in any FlightState and the user switched SW1 to enable or disable externalControl (=computer control)
    if(computerControlActive)
    {
        // The user tells us to control the kopter.
        if(mFlightState == Idle)
        {
            qDebug() << t() << "FlightController::slotExternalControlStatusChanged(): externalControl activated, but previous flightState is" << getFlightStateString(mFlightState) << "- Staying in idle mode.";
        }
        else
        {
            if(mFlightState != ManualControl)
            {
                qDebug() << t() << "FlightController::slotExternalControlStatusChanged(): externalControl activated, but previous flightState is" << getFlightStateString(mFlightState) << "instead of ManualControl - WARNING. Still switching to ApproachingNextWayPoint";
            }

            setFlightState(ApproachingNextWayPoint);
            ensureSafeFlightAfterWaypointsChanged();
        }
    }
    else
    {
        // This one is easy. When the user disallows computer control, switch to ManualControl
        qDebug() << t() << "FlightController::slotExternalControlStatusChanged(): externalControl deactivated, switching to ManualControl";
        setFlightState(ManualControl);
    }
}

void FlightController::slotEmitFlightState()
{
    emit flightStateChanged(mFlightState);
}
