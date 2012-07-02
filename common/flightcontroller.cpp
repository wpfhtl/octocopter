#include "flightcontroller.h"
#include "motioncommand.h"

FlightController::FlightController() : QObject(), mFlightState(UserControl)
{
    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mTimeOfLastControllerUpdate = QTime::currentTime();
    mFirstControllerRun = true;

    mBackupTimerComputeMotion = new QTimer(this);
    connect(mBackupTimerComputeMotion, SIGNAL(timeout()), SLOT(slotComputeBackupMotion()));

    mImuOffsets.pitch = 0.0f;
    mImuOffsets.roll = 0.0f;

    // Tests have shown:
    // - with metal hood (2425 gram) it hovers at 127/128.
    mThrustHover = 127.0;

    /*// For testing in simulator
    mWayPoints.append(WayPoint(QVector3D(130,90,110)));
    mWayPoints.append(WayPoint(QVector3D(140,80,130)));
    mWayPoints.append(WayPoint(QVector3D(120,90,120)));
    mWayPoints.append(WayPoint(QVector3D(150,80,150)));
    setFlightState(ApproachWayPoint);*/
}

FlightController::~FlightController()
{
    mBackupTimerComputeMotion->stop();
    mBackupTimerComputeMotion->deleteLater();
}

void FlightController::slotComputeBackupMotion()
{
    qDebug() << t() << "FlightController::slotComputeBackupMotion(): no pose for" << mBackupTimerComputeMotion->interval() << "ms, flightstate" << getFlightStateString(mFlightState) << ": starting backup motion computation.";
    Q_ASSERT(mFlightState != UserControl && "FlightController::slotComputeBackupMotion(): i was called in FlightState UserControl");
    slotComputeMotionCommands();
}

void FlightController::slotComputeMotionCommands()
{
    Q_ASSERT(mFlightState != UserControl && "FlightController::slotComputeMotionCommands(): i was called in FlightState UserControl");

    switch(mFlightState)
    {


    case Hover:
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: Hover, emitting hover-thrust";
        emit motion(MotionCommand(mThrustHover));
        break;

    case Idle:
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: Idle, emitting idle-thrust.";
        emit motion(MotionCommand(90));
        break;

    case ApproachWayPoint:
    {
        if(mWayPoints.size() < 1)
        {
            qDebug() << t() << "FlightController::slotComputeMotionCommands(): FlightState: ApproachWayPoint: Cannot approach, no waypoints present!";
            emit motion(MotionCommand(mThrustHover));
            return;
        }

        if(getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp > 82)
        {
            qDebug() << t() << "FlightController::slotComputeMotionCommands(): ApproachWayPoint, vehicle pose update is" << getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp << "ms ago, skipping motion computation, emitting hover thrust";
            emit motion(MotionCommand(mThrustHover));
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

        // Elapsed time since last call in seconds. May not become zero (divide by zero)
        // and shouldn't grow too high, as that will screw up the controllers.
        const float timeDiff = qBound(
                    0.01f,
                    mTimeOfLastControllerUpdate.msecsTo(QTime::currentTime()) / 1000.0f,
                    0.2f);

        qDebug() << "timediff:" << timeDiff << "factorPlanarDistance" << factorPlanarDistance << "factorHeight" << factorHeight;

        // If angleToTurnToWayPoint is:
        // - positive, we need to rotate CCW, which needs a negative yaw value.
        // - negative, we need to rotate  CW, which needs a positive yaw value.
        const float errorYaw = RAD2DEG(angleToTurnToWayPoint);
        mErrorIntegralYaw += errorYaw * timeDiff;
        const float derivativeYaw = mFirstControllerRun ? 0.0f : (errorYaw - mPrevErrorYaw + 0.00001f)/timeDiff;
        const float outputYaw = factorHeight * factorPlanarDistance * ((1.0f * errorYaw) + (0.0f * mErrorIntegralYaw) + (0.3f/*0.5f*/ * derivativeYaw));

        // adjust pitch/roll to reach target, maximum pitch is -20 degrees (forward)
        float desiredRoll = 0.0f;
        float desiredPitch = -pow(20.0f - qBound(0.0, fabs(errorYaw), 20.0), 2.0f) / 20.0f;

        // try to get ourselves straight up
        const float currentPitch = mLastKnownVehiclePose.getPitchDegrees() - mImuOffsets.pitch;
        const float errorPitch = desiredPitch - currentPitch;
        mErrorIntegralPitch += errorPitch * timeDiff;
        const float derivativePitch = mFirstControllerRun ? 0.0f : (errorPitch - mPrevErrorPitch + 0.00001f)/timeDiff;
        // WARNING: If we multiply by factorHeight, we won't stabilize the kopter at low height, it'll have to do that by itself. Is that good?
        const float outputPitch = factorHeight * factorPlanarDistance * ((6.0f * errorPitch) + (0.3f * mErrorIntegralPitch) + (0.0f * derivativePitch));

        const float currentRoll = mLastKnownVehiclePose.getRollDegrees() - mImuOffsets.roll;
        const float errorRoll = desiredRoll - currentRoll;
        mErrorIntegralRoll += errorRoll * timeDiff;
        const float derivativeRoll = mFirstControllerRun ? 0.0f : (errorRoll - mPrevErrorRoll + 0.00001f)/timeDiff;
        // WARNING: If we multiply by factorHeight, we won't stabilize the kopter at low height, it'll have to do that by itself. Is that good?
        const float outputRoll = factorHeight * factorPlanarDistance * ((6.0f * errorRoll) + (0.3f * mErrorIntegralRoll) + (0.0f * derivativeRoll));

        /* Todo: move stuff above to
        class PidController
        {
            float valueDesired;
            float valueRaw;
            float valueCalibrated;
            float error;
            float errorIntegral;
            float derivative;
            float output;

            void update(float valueRaw)
        };*/

        const float errorHeight = nextWayPoint.y() - mLastKnownVehiclePose.getPosition().y();
        mErrorIntegralHeight += errorHeight * timeDiff;

        // We do need to use the I-controller, but we should clear the integrated error once we have crossed the height of a waypoint.
        // Otherwise, the integral will grow large while ascending and then keep the kopter above the waypoint for a loooong time.
        if((mPrevErrorHeight > 0.0f && errorHeight < 0.0f) || (mPrevErrorHeight < 0.0f && errorHeight > 0.0f))
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): crossed waypoint height, cutting mErrorIntegralHeight to a third.";
            mErrorIntegralHeight /= 3.0f;
        }

        const float derivativeHeight = mFirstControllerRun ? 0.0f : (errorHeight - mPrevErrorHeight + 0.00001f)/timeDiff;
        const float outputThrust = mThrustHover + (25.0f * errorHeight) + (0.01f * mErrorIntegralHeight) + (1.0f * derivativeHeight);

        qDebug() << mWayPoints.size() << "waypoints, next wpt height" << nextWayPoint.y() << "curr height" << mLastKnownVehiclePose.getPosition().y();

        qDebug() << "values PRYH:" << QString::number(currentPitch, 'f', 2) << "\t" << QString::number(currentRoll, 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getYawDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getPosition().y(), 'f', 2);
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
    break;

    default:
        qDebug() << t() << "FlightController::slotComputeMotionCommands(): FLIGHTSTATE NOT DEFINED:" << mFlightState;
        Q_ASSERT(false);
    }

    mTimeOfLastControllerUpdate = QTime::currentTime();
}

void FlightController::slotCalibrateImu()
{
    mImuOffsets.pitch = mLastKnownVehiclePose.getPitchDegrees();
    mImuOffsets.roll = mLastKnownVehiclePose.getRollDegrees();
    qDebug() << t() << "FlightController::slotCalibrateImu(): calibrated IMU offsets to pitch" << mImuOffsets.pitch << "and roll" << mImuOffsets.roll << "from pose TOW" << mLastKnownVehiclePose.timestamp;
}

void FlightController::nextWayPointReached()
{
    Q_ASSERT(mFlightState == ApproachWayPoint && "FlightController::nextWayPointReached(): reached waypoint, but am not in ApproachWayPoint state. Expect trouble!");

    // The next waypoint has been reached.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    qDebug() << "FlightController::nextWayPointReached(): reached waypoint" << mWayPointsPassed.last();

    // First emit this signal for the base to sync the list, THEN call
    // ensureSafeFlightAfterWaypointsChanged(), which might append more waypoints.
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
        qDebug() << t() << "FlightController::slotWayPointInsert(): we were idle, switching to ApproachWayPoint";
        setFlightState(ApproachWayPoint);
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
        if(mFlightState == ApproachWayPoint)
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
        qDebug() << t() << "FlightController::slotSetWayPoints(): we were idle and got new waypoints, switching to ApproachWayPoint";
        setFlightState(ApproachWayPoint);
    }

    // The list might now be empty, so we might have to land.
    if(mFlightState == ApproachWayPoint)
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
    qDebug() << pose.timestamp << "FlightController::slotNewVehiclePose(): flightstate:" << getFlightStateString(mFlightState);

    // Whatever precision and flightstate, save the pose.
    mLastKnownVehiclePose = pose;

    switch(mFlightState)
    {
    case UserControl:
        // Keep ourselves disabled in UserControl.
        Q_ASSERT(!mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): UserControl has active backup timer!");
        break;

    case ApproachWayPoint:
        Q_ASSERT(mBackupTimerComputeMotion->interval() == 50 && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): ApproachWayPoint has inactive backup timer or wrong interval!");

        // Note that we don't expect ModeIntegrated, because only 1 of 5 packets is integrated - and thats ok.
        if(pose.precision & Pose::AttitudeAvailable && pose.precision & Pose::HeadingFixed && pose.precision && Pose::RtkFixed)
        {
            qDebug() << "FlightController::slotNewVehiclePose(): received a usable:" << pose << ": computing motion.";

            // This method was called with a new pose, so lets use it to compute motion
            slotComputeMotionCommands();

            // Re-set the safety timer, making it fire 50ms in the future - not when its scheduled, which might be sooner.
            // So that when no new pose comes in for too long, we'll compute a backup motion.
            mBackupTimerComputeMotion->start(50);
        }
        else
        {
            qDebug() << "FlightController::slotNewVehiclePose(): received a useless" << pose.toStringVerbose() << ": doing nothing.";
        }
        break;

    case Hover:
        Q_ASSERT(mBackupTimerComputeMotion->interval() == 1000 && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Hover has inactive backup timer or wrong interval!");
        break;

    case Idle:
        Q_ASSERT(mBackupTimerComputeMotion->interval() == 1000 && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Idle has inactive backup timer or wrong interval!");
        break;

    default:
        Q_ASSERT(false && "illegal flightstate in FlightController::slotNewVehiclePose()!");
    }

}

void FlightController::slotHoldPosition()
{
    setFlightState(Hover);
}

void FlightController::setFlightState(FlightState newFlightState)
{
    qDebug() << t() << "FlightController::setFlightState():" << getFlightStateString(mFlightState) << "=>" << getFlightStateString(newFlightState);
    if(mFlightState == newFlightState)
    {
        qDebug() << t() << "FlightController::setFlightState(): switching to same flightstate doesn't make much sense, returning.";
        return;
    }

    switch(newFlightState)
    {
    case ApproachWayPoint:
        qDebug() << t() << "FlightController::setFlightState(): ApproachWayPoint - initializing controllers, setting backup motion timer to high-freq";

        // We're going to use the controllers, so make sure to initialize them. We don't
        // need to set mTimeOfLastControllerUpdate, as that is qBound()ed anyway.
        mPrevErrorPitch = 0.1f;
        mPrevErrorRoll = 0.1f;
        mPrevErrorYaw = 0.1f;
        mPrevErrorHeight = 0.1f;

        mErrorIntegralPitch = 0.1f;
        mErrorIntegralRoll = 0.1f;
        mErrorIntegralYaw = 0.1f;
        mErrorIntegralHeight = 0.1;

        mFirstControllerRun = true;
        mBackupTimerComputeMotion->start(50);
        break;

    case Hover:
        qDebug() << t() << "FlightController::setFlightState(): setting backup motion timer to low-freq";
        mBackupTimerComputeMotion->start(1000);
        break;

    case UserControl:
        qDebug() << t() << "FlightController::setFlightState(): disabling backup motion timer.";
        mBackupTimerComputeMotion->stop();
        break;

    case Idle:
        qDebug() << t() << "FlightController::setFlightState(): setting backup motion timer to low-freq";
        mBackupTimerComputeMotion->start(1000);
        break;
    default:
        Q_ASSERT(false && "FlightController::setFlightState(): undefined flightstate");
    }

    mFlightState = newFlightState;
    emit flightStateChanged(newFlightState);
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
// If wpt list is empty and state is ApproachWayPoint, will either
//  - idle if low
//  - get a landing waypoint if heightOverGround is known and up-to-date
//  - descend slowly if heightOverGround is unknown
void FlightController::ensureSafeFlightAfterWaypointsChanged()
{
    mFirstControllerRun = true; // to tame the derivatives

    Q_ASSERT(mFlightState == ApproachWayPoint && "FlightController::ensureSafeFlightAfterWaypointsChanged(): flightstate is NOT ApproachWayPoint!");

    if(mWayPoints.size() == 0)
    {
        // The list is now empty and we are still flying.
        if(isHeightOverGroundValueRecent() && mLastKnownHeightOverGround < 0.3f)
        {
            // We're low anyway, just got to idle mode.
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, we're low with valid heightOverGround of" << mLastKnownHeightOverGround << ", idling";
            setFlightState(Idle);
        }
        else if(isHeightOverGroundValueRecent())
        {
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround is known to be" << mLastKnownHeightOverGround << "inserting landing wpt 0.2m above ground";
            mWayPoints.append(WayPoint(mLastKnownVehiclePose.getPosition() - QVector3D(0.0, mLastKnownHeightOverGround - 0.2, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
        else
        {
            // TODO: We might instead switch to Hover instead of trying to land here?!

            // Descend with 10cm per second, so thats 0.1m/PoseFrequency, assuming that we'll
            // hit the new waypoint on every controller computation. This is likely, as hitting
            // a waypoints means being closer than 50cm to it.

            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround unknown, WARNING, next wpt is 0.005m below us.";
            mWayPoints.append(WayPoint(mLastKnownVehiclePose.getPosition() - QVector3D(0.0, 0.005, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
    }
}

void FlightController::slotComputerControlStatusChanged(bool computerControl)
{
    qDebug() << t() << "FlightController::slotComputerControlStatusChanged(): flightstate" << getFlightStateString(mFlightState) << "computer control changed to:" << computerControl;

    switch(mFlightState)
    {
    case UserControl:
        if(computerControl)
        {
            // We are in UserControl, but switching to ComputerControl.
            setFlightState(ApproachWayPoint);

            // Make sure there actually ARE waypoints. If not, create some for landing.
            ensureSafeFlightAfterWaypointsChanged();
        }
        else
        {
            // This is an illegal state: We are already in UserControl, and now we're
            // told that the user has disabled computerControl. Thats impossible.
            Q_ASSERT(false && "FlightController::slotComputerControlStatusChanged(): we're in UserControl and now computercontrol was disabled?!");
        }
        break;
    case ApproachWayPoint:
        if(computerControl)
        {
            // We are approaching waypoints, and now the user changed to ComputerControl?
            // Thats impossible, because we are already in ComputerControl (ApproachWayPoint)
            Q_ASSERT(false && "FlightController::slotComputerControlStatusChanged(): we're in ApproachWayPoint and now computercontrol was enabled?!");
        }
        else
        {
            // We are approaching waypoints, but the user wants to take over control. Ok.
            setFlightState(UserControl);

            // We don't need the timer, as the remote control will overrule our output anyway.
            mBackupTimerComputeMotion->stop();
        }
        break;
    case Hover:
        if(computerControl)
        {
            // We are in Hover, and now the user changed to ComputerControl?
            // Thats impossible, because we are already in ComputerControl (Hover)
            Q_ASSERT(false && "FlightController::slotComputerControlStatusChanged(): we're in Hover and now computercontrol was enabled?!");
        }
        else
        {
            // We are hovering, but the user wants to take over control. Ok.
            setFlightState(UserControl);

            // We don't need the timer, as the remote control will overrule our output anyway.
            mBackupTimerComputeMotion->stop();
        }
        break;
    case Idle:
        if(computerControl)
        {
            // We are Idle, and now the user changed to ComputerControl?
            // Thats impossible, because we are already in ComputerControl (Idle)
            Q_ASSERT(false && "FlightController::slotComputerControlStatusChanged(): we're in Idle and now computercontrol was enabled?!");
        }
        else
        {
            // We are Idle, but the user wants to take over control. Ok.
            setFlightState(UserControl);

            // We don't need the timer, as the remote control will overrule our output anyway.
            mBackupTimerComputeMotion->stop();
        }
        break;
    default:
        Q_ASSERT(false && "FlightController::slotComputerControlStatusChanged(): illegal flightstate!");
    }

    qDebug() << t() << "FlightController::slotComputerControlStatusChanged(): done, new flightstate" << getFlightStateString(mFlightState);
}

void FlightController::slotEmitFlightState()
{
    emit flightStateChanged(mFlightState);
}
