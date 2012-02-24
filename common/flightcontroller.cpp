#include "flightcontroller.h"

FlightController::FlightController() : QObject(), mFlightState(Idle)
{
    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mTimeOfLastControllerUpdate = QTime::currentTime();
    mFirstControllerRun = true;

    // For testing in simulator
    mWayPoints.append(WayPoint(QVector3D(130,90,110)));
    mWayPoints.append(WayPoint(QVector3D(140,80,130)));
    mWayPoints.append(WayPoint(QVector3D(120,90,120)));
    mWayPoints.append(WayPoint(QVector3D(150,80,150)));
    setFlightState(ApproachingNextWayPoint);

    emit currentWayPoints(mWayPoints);
}

FlightController::~FlightController()
{

}

void FlightController::slotComputeMotionCommands()
{
    const double timeDiff = std::min(0.2, (double)(mTimeOfLastControllerUpdate.msecsTo(QTime::currentTime()) / 1000.0)); // elapsed time since last call in seconds

    quint8 out_thrust = 0;
    qint8 out_yaw, out_pitch, out_roll = 0;

    // TODO: Do we still need this flightstate? how do we detect that manual control is activated?
    if(getFlightState() == ManualControl)
    {
        qDebug() << "FlightController::slotComputeMotionCommands(): FlightState: ManualControl";

        // We do not emit motion commands, as they should be ignored anyway.
        emitSafeControlValues();
    }
    else if(getFlightState() == ApproachingNextWayPoint)
    {
        Q_ASSERT(mWayPoints.size() > 0);

        if(getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp > 500)
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): vehicle pose update is more than 500ms ago, skipping motion computation, emitting safe control values";
            emitSafeControlValues();
            return;
        }

        const WayPoint nextWayPoint = mWayPoints.first();

        const QVector2D vectorVehicleToWayPoint = (nextWayPoint.getPositionOnPlane() - mLastKnownVehiclePose.getPlanarPosition());
        const float directionToWayPointRadians = atan2(-vectorVehicleToWayPoint.x(), -vectorVehicleToWayPoint.y());
        const float angleToTurnToWayPoint = Pose::getShortestTurnRadians(directionToWayPointRadians - mLastKnownVehiclePose.getYawRadians());

//        qDebug() << "mLastKnownVehiclePose.getPlanarPosition()" << mLastKnownVehiclePose.getPlanarPosition();
//        qDebug() << "nextWayPoint.getPositionOnPlane():" << nextWayPoint.getPositionOnPlane();
//        qDebug() << "directionVectorToNextWayPoint:" << directionVectorToNextWayPoint;
        qDebug() << "timediff:" << timeDiff;
        qDebug() << "directionToWayPoint:" << RAD2DEG(directionToWayPointRadians);
        qDebug() << "angleToTurnToWayPoint: turn" << (angleToTurnToWayPoint < 0 ? "right" : "left") << RAD2DEG(angleToTurnToWayPoint);

        // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!

        // If the planar distance to the next waypoint is very small (this happens when only the height is off),
        // we don't wnat to yaw and pitch. So, we introduce a factor [0;1], which becomes 0 with small distance
        const float planarDistanceFactor = qBound(0.0f, (float)(mLastKnownVehiclePose.getPlanarPosition() - Pose::getPlanarPosition(nextWayPoint)).length() * 2.0f, 1.0f);

        // If angleToTurnToWayPoint is:
        // - positive, we need to rotate CCW, which needs a negative yaw value.
        // - negative, we need to rotate  CW, which needs a positive yaw value.
        const float errorYaw = RAD2DEG(angleToTurnToWayPoint);
        mErrorIntegralYaw += errorYaw*timeDiff;
        const float derivativeYaw = mFirstControllerRun ? 0.0f : (errorYaw - mPrevErrorYaw + 0.00001f)/timeDiff;
        const float outputYaw = planarDistanceFactor * (1.0f * errorYaw) + (0.0f * mErrorIntegralYaw) + (0.5f * derivativeYaw);

        // adjust pitch/roll to reach target, maximum pitch is -20 degrees (forward)
        float desiredRoll = 0.0f;
        float desiredPitch = -pow(20.0f - qBound(0.0, fabs(errorYaw), 20.0), 2.0f) / 20.0f;

        // try to get ourselves straight up
        const float errorPitch = desiredPitch - mLastKnownVehiclePose.getPitchDegrees();
        mErrorIntegralPitch += errorPitch*timeDiff;
        const float derivativePitch = mFirstControllerRun ? 0.0f : (errorPitch - mPrevErrorPitch + 0.00001f)/timeDiff;
        const float outputPitch = planarDistanceFactor * (6.0f * errorPitch) + (0.3f * mErrorIntegralPitch) + (0.0f * derivativePitch);

        const float errorRoll = desiredRoll - mLastKnownVehiclePose.getRollDegrees();
        mErrorIntegralRoll += errorRoll*timeDiff;
        const float derivativeRoll = mFirstControllerRun ? 0.0f : (errorRoll - mPrevErrorRoll + 0.00001f)/timeDiff;
        const float outputRoll = planarDistanceFactor * (6.0f * errorRoll) + (0.3f * mErrorIntegralRoll) + (0.0f * derivativeRoll);

        const float outputHover = 120.0;
        const float errorHeight = nextWayPoint.y() - mLastKnownVehiclePose.position.y();
        mErrorIntegralHeight += errorHeight*timeDiff;
        const float derivativeHeight = mFirstControllerRun ? 0.0f : (errorHeight - mPrevErrorHeight + 0.00001f)/timeDiff;
        const float outputThrust = outputHover + (15.0f * errorHeight) + (0.0f * mErrorIntegralHeight) + (1.0f * derivativeHeight);

        qDebug() << "no wpts" << mWayPoints.size() << "next wpt height" << nextWayPoint.y() << "curr height" << mLastKnownVehiclePose.position.y() << "thrust" << outputThrust;

        qDebug() << "values PRYH:" << QString::number(mLastKnownVehiclePose.getPitchDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getRollDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getYawDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.position.y(), 'f', 2);
        qDebug() << "should PRYH:" << QString::number(desiredPitch, 'f', 2) << "\t" << QString::number(desiredRoll, 'f', 2) << "\t" << QString::number(RAD2DEG(directionToWayPointRadians), 'f', 2) << "\t" << QString::number(nextWayPoint.y(), 'f', 2);
        qDebug() << "error  PRYH:" << QString::number(errorPitch, 'f', 2) << "\t" << QString::number(errorRoll, 'f', 2) << "\t" << QString::number(errorYaw, 'f', 2) << "\t" << QString::number(errorHeight, 'f', 2);
        qDebug() << "derivt PRYH:" << QString::number(derivativePitch, 'f', 2) << "\t" << QString::number(derivativeRoll, 'f', 2) << "\t" << QString::number(derivativeYaw, 'f', 2) << "\t" << QString::number(derivativeHeight, 'f', 2);
        qDebug() << "prevEr PRYH:" << QString::number(mPrevErrorPitch, 'f', 2) << "\t" << QString::number(mPrevErrorRoll, 'f', 2) << "\t" << QString::number(mPrevErrorYaw, 'f', 2) << "\t" << QString::number(mPrevErrorHeight, 'f', 2);
        qDebug() << "inteEr PRYH:" << QString::number(mErrorIntegralPitch, 'f', 2) << "\t" << QString::number(mErrorIntegralRoll, 'f', 2) << "\t" << QString::number(mErrorIntegralYaw, 'f', 2) << "\t" << QString::number(mErrorIntegralHeight, 'f', 2);
        qDebug() << "output PRYH:" << QString::number(outputPitch, 'f', 2) << "\t" << QString::number(outputRoll, 'f', 2) << "\t" << QString::number(outputYaw, 'f', 2) << "\t" << QString::number(outputThrust, 'f', 2);

        mPrevErrorPitch = errorPitch;
        mPrevErrorRoll = errorRoll;
        mPrevErrorYaw = errorYaw;
        mPrevErrorHeight = errorHeight;

        out_thrust = (quint8)qBound(90.0f, outputThrust, 200.0f);
        out_yaw = (qint8)qBound(-127.0, outputYaw > 0.0f ? ceil(outputYaw) : floor(outputYaw), 127.0);
        out_pitch = (qint8)qBound(-20.0f, outputPitch, 20.0f);
        out_roll = (qint8)qBound(-20.0f, outputRoll, 20.0f);

        // For safety, we don't need it right now.
        out_roll = 0;

        qDebug() << "FlightController::slotComputeMotionCommands(): motion is" << out_thrust << out_yaw << out_pitch << out_roll;

        emit motion(out_thrust, out_yaw, out_pitch, out_roll, 0);
        emit debugValues(mLastKnownVehiclePose, out_thrust, out_yaw, out_pitch, out_roll, 0);

        // See whether we've reached the waypoint
        if(mLastKnownVehiclePose.position.distanceToLine(nextWayPoint, QVector3D()) < 0.25) // close to wp
        {
            wayPointReached();
        }

        mFirstControllerRun = false;
    }
    else if(mFlightState == Idle)
    {
        emit motion(0, 0, 0, 0, 0);
    }
    else
    {
        Q_ASSERT(false && "FlightState not defined!");
    }

    mTimeOfLastControllerUpdate = QTime::currentTime();
}



void FlightController::wayPointReached()
{
    // The current waypoint has been reached.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    qDebug() << "FlightController::wayPointReached(): reached waypoint" << mWayPointsPassed.last();

    if(getFlightState() != ApproachingNextWayPoint)
        qDebug() << "FlightController::wayPointReached(): reached waypoint" << mWayPointsPassed.last() << "but am not in ApproachingNextWayPoint state. Expect trouble!";

    if(mWayPoints.size())
    {
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, "waypoint reached, more waypoints present, approaching");
        qDebug() << "FlightController::wayPointReached(): approaching next waypoint" << mWayPoints.first();
        mFirstControllerRun = true; // to tame the derivatives
    }
    else if(mLastKnownBottomBeamLength < 0.3)
    {
        qDebug() << "FlightController::wayPointReached(): reached waypoint, no more, we're low, idling";
        setFlightState(Idle);
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, "waypoint reached, no more wayPoints, HeightAboveGround low, idling");
    }
    else
    {
        mWayPoints.append(getLandingWayPoint());
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, QString("ManualControl disabled, no further wayPoints, HeightAboveGround high (" + QString::number(mLastKnownBottomBeamLength) + "m), now landing"));
        qDebug() << "FlightController::wayPointReached(): reached waypoint, no more, we're NOT low, adding landing wpt.";
        mFirstControllerRun = true; // to tame the derivatives
    }

    emit wayPointReached(mWayPointsPassed.last());
    emit currentWayPoints(mWayPoints);
}

void FlightController::clearWayPoints()
{
    mWayPoints.clear();
    emit currentWayPoints(mWayPoints);
}

void FlightController::slotWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "FlightController::slotWayPointInsert(): trying to insert waypoint at index" << index;
    qDebug() << "FlightController::slotSetNextWayPoint(): state is" << getFlightStateString() << "inserting waypoint into index" << index;
    mWayPoints.insert(index, wayPoint);
    emit currentWayPoints(mWayPoints);

    // Just in case we were idle (or landing, which is the same) before...
    if(mFlightState == Idle) setFlightState(ApproachingNextWayPoint);
    //else Q_ASSERT(false && "hashfailure in slotWayPointInsert");
    //    qDebug() <<  "FlightController::slotWayPointInsert():" << hashValue << index << wayPoint << "hash failure";

//    emit currentWayPoints(mWayPoints);
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
        if(mWayPoints.size() == 0 && getFlightState() == ApproachingNextWayPoint)
        {
            // The list is now empty and we are flying. Insert a landing-waypoint if we're not close to the ground;
            if(mLastKnownBottomBeamLength > 0.2)
            {
                // Insert landing-waypoint, keep approaching
                qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, list is empty, inserting landing wpt";
                mWayPoints.append(getLandingWayPoint());
            }
            else
            {
                // We're low anyway, just got to idle mode.
                qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, list is empty, we're low with beamlength of" << mLastKnownBottomBeamLength << ", idling";
                setFlightState(Idle);
            }
        }
    }
    qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, emitting new wpt list of size" << mWayPoints.size();
    emit currentWayPoints(mWayPoints);
}

void FlightController::slotSetWayPoints(const QList<WayPoint>& wayPoints)
{
    mWayPoints.clear();
    mWayPoints = wayPoints;

    // The list might now be empty, so we might have to land.
    if(mWayPoints.size() == 0 && getFlightState() == ApproachingNextWayPoint)
    {
        // The list is now empty and we are flying. Insert a landing-waypoint if we're not close to the ground;
        if(mLastKnownBottomBeamLength > 0.2)
        {
            // Insert landing-waypoint, keep approaching
            mWayPoints.append(getLandingWayPoint());
        }
        else
        {
            // We're low anyway, just got to idle mode.
            setFlightState(Idle);
        }
    }
    else if(mWayPoints.size() > 0 && mFlightState == Idle)
    {
        setFlightState(ApproachingNextWayPoint);
    }

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

WayPoint FlightController::getLandingWayPoint() const
{
    // naive implementation, WARNING: make sure kopter is straight in the air!
    qDebug() << "FlightController::getLandingWayPoint(): kopter is at" << mLastKnownVehiclePose.position << " lastKnownBottomBeamLength is from" << mLastKnownBottomBeamLengthTimestamp << "and is" << mLastKnownBottomBeamLength << "so returning a waypoint at curPos - BeamLength.";
    return WayPoint(mLastKnownVehiclePose.position - QVector3D(0.0, mLastKnownBottomBeamLength, 0.0));
}

FlightController::FlightState FlightController::getFlightState(void) const { return mFlightState; }

QString FlightController::getFlightStateString(void) const
{
    switch(mFlightState)
    {
    case ManualControl: return "ManualControl"; break;
    case ApproachingNextWayPoint: return "ApproachingNextWayPoint"; break;
    case Freezing: return "Freezing"; break;
    case Idle: return "Idle"; break;
    }

    Q_ASSERT(false && "FLIGHTSTATE UNDEFINED!");
}

void FlightController::slotNewVehiclePose(const Pose& pose)
{
//    qDebug() << "FlightController::slotSetVehiclePose(): vehicle now at" << pose;
    mLastKnownVehiclePose = pose;
}

void FlightController::slotFreeze()
{
    setFlightState(Freezing);
    mWayPoints.clear();
    emit currentWayPoints(mWayPoints);
}

void FlightController::setFlightState(const FlightState& flightState)
{
    if(mFlightState != flightState)
    {
        if(flightState == ApproachingNextWayPoint)
        {
            // We're going to use the controllers, so make sure to initialize them
            mPrevErrorPitch = 0.1f;
            mPrevErrorRoll = 0.1f;
            mPrevErrorYaw = 0.1f;
            mPrevErrorHeight = 0.1f;

            // clear the error integrals, so we don't get spikes after releasing the joystick
            mErrorIntegralPitch = 0.1f;
            mErrorIntegralRoll = 0.1f;
            mErrorIntegralYaw = 0.1f;
            mErrorIntegralHeight = 0.1;

            // So the that the derivative will be predictable
            mTimeOfLastControllerUpdate = QTime::currentTime();

            mFirstControllerRun = true;
        }

        mFlightState = flightState;
        emit flightStateChanged(flightState);
    }
}

void FlightController::slotSetHeightOverGround(const float& beamLength)
{
    qDebug() << "FlightController::slotSetHeightOverGround()" << beamLength;
    mLastKnownBottomBeamLengthTimestamp = QTime::currentTime();
    mLastKnownBottomBeamLength = beamLength;
}


void FlightController::emitSafeControlValues()
{
    qDebug() << "FlightController::emitSafeControlValues()";
    emit motion(110, 0, 0, 0, 0);
}
