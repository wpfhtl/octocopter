#include "flightcontroller.h"

FlightController::FlightController() : QObject()
{
    setFlightState(ApproachingNextWayPoint);

    for(int i=0;i<4;i++) {
//        setFlightState(ApproachingNextWayPoint);
        mWayPoints.append(QVector3D(140, 100, 80));
        mWayPoints.append(QVector3D(240, 100, 80));
        mWayPoints.append(QVector3D(140, 100, 160));
        mWayPoints.append(QVector3D(240, 100, 160));
    }

    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mDesiredYaw = 0.0;

    mTimeOfLastControllerUpdate = QTime::currentTime();
    mTimeOfLastLaserScan = QTime::currentTime().addSecs(-2);

    emit currentWayPoints(mWayPoints);
}

FlightController::~FlightController()
{

}

void FlightController::slotComputeMotionCommands()
{
    const double timeDiff = std::min(0.2, (double)(mTimeOfLastControllerUpdate.msecsTo(QTime::currentTime()) / 1000.0)); // elapsed time since last call in seconds

    // deprecated
//    int f,b,l,r;

    quint8 out_thrust = 0;
    qint8 out_yaw, out_pitch, out_roll = 0;

    // TODO: Do we still need this flightstate? how do we detect that manual control is activated?
    if(getFlightState() == ManualControl)
    {
//        qDebug() << "FlightController::slotComputeMotionCommands(): joystick";


        mPrevErrorPitch = 0 - mLastKnownVehiclePose.getPitchDegrees(); //mMotionState->getOrientation().getPitch(false).valueDegrees();
        mPrevErrorRoll = 0 - mLastKnownVehiclePose.getRollDegrees(); //mMotionState->getOrientation().getRoll(false).valueDegrees();

        // clear the error integrals, so we don't get spikes after releasing the joystick
        mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

        // It should not matter, but just emit some halfway-safe value;
        emit motion(100, 0, 0, 0, 0);
    }
    else if(getFlightState() == ApproachingNextWayPoint)
    {
        Q_ASSERT(mWayPoints.size() > 0);

        if(getCurrentGpsTowTime() - mLastKnownVehiclePose.timestamp > 500)
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): vehicle pose update is more than 500ms ago, skipping motion computation.";
            return;
        }

        WayPoint nextWayPoint = mWayPoints.first();

        // If the laserscanner is active, rotate slowly
        if(mTimeOfLastLaserScan.msecsTo(QTime::currentTime()) > 500)
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): timeDiff" << timeDiff << "approaching" << mLastKnownVehiclePose.position << "->" << nextWayPoint;
            mDesiredYaw = 0.0;
        }
        else
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): timeDiff" << timeDiff << "scanning" << mLastKnownVehiclePose.position << "->" << nextWayPoint;
            mDesiredYaw = fmod(mDesiredYaw + (timeDiff * 10.0 /*deg per second*/), 360.0);
        }

        const QVector2D directionVectorToNextWayPoint = (nextWayPoint.getPositionOnPlane() - mLastKnownVehiclePose.getPlanarPosition());//.normalized();
        const float angleBetweenVehicleNorthAndNextWayPoint = atan2(-directionVectorToNextWayPoint.x(), -directionVectorToNextWayPoint.y());
        const float angleToTurnToWayPoint = Pose::normalizeAngleRadians(angleBetweenVehicleNorthAndNextWayPoint - mLastKnownVehiclePose.getYawRadians());

        qDebug() << "mLastKnownVehiclePose.getPlanarPosition()" << mLastKnownVehiclePose.getPlanarPosition();
        qDebug() << "nextWayPoint.getPositionOnPlane():" << nextWayPoint.getPositionOnPlane();
        qDebug() << "directionVectorToNextWayPoint:" << directionVectorToNextWayPoint;
        qDebug() << "angleBetweenVehicleNorthAndNextWayPoint:" << RAD2DEG(angleBetweenVehicleNorthAndNextWayPoint);

        if(angleToTurnToWayPoint > 0)
            qDebug() << "To point at waypoint, turn left:" << RAD2DEG(angleToTurnToWayPoint);
        else
            qDebug() << "To point at waypoint, turn right:" << RAD2DEG(angleToTurnToWayPoint);

        // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!
        static double Kp = 5.1;
        static double Ki = 1.2;
        static double Kd = 0.6;

//        const float currentPitch = mMotionState->getOrientation().getPitch(true).valueDegrees();
//        const float currentRoll = mMotionState->getOrientation().getRoll(true).valueDegrees();
//        const float currentYaw = 180.0 + mMotionState->getNode()->_getDerivedOrientation().get/*Pitch*/(true).valueDegrees();
//        const float currentPitch = mLastKnownVehiclePose.getPitchDegrees(); //mMotionState->getNode()->_getDerivedOrientation().getPitch(true).valueDegrees();
//        const float currentRoll = mLastKnownVehiclePose.getRollDegrees(); //mMotionState->getNode()->_getDerivedOrientation().getRoll(true).valueDegrees();
//        const float currentYaw = 180.0 + mLastKnownVehiclePose.getYawDegrees(); //mMotionState->getNode()->_getDerivedOrientation().getYaw(true).valueDegrees();
//        const float currentHeight = mLastKnownVehiclePose.position.y();//mLastKnownBottomBeamLength; // WARNING: use position.y? if not, check mLastKnownBottomBeamLengthTimestamp//mLastKnownVehiclePose.position.y(); //mMotionState->getPosition().y;//mVehicle->getHeightAboveGround();

        double errorYaw = mDesiredYaw - Pose::normalizeAngleDegrees(mLastKnownVehiclePose.getYawDegrees());
        mErrorIntegralYaw += errorYaw*timeDiff;
        double derivativeYaw = (errorYaw - mPrevErrorYaw + 0.00001)/timeDiff;
        double outputYaw = (Kp*errorYaw) + (Ki*mErrorIntegralYaw) + (Kd*derivativeYaw);
//        outputYaw *= -1.0;

        // adjust pitch/roll to reach target
        double desiredRoll, desiredPitch = 0.0;
        if(fabs(errorYaw) < 5.0 || true)
        {
            qDebug() << "ROLL  SHOULD BE" << sin(angleToTurnToWayPoint);
            qDebug() << "PITCH SHOULD BE" << cos(angleToTurnToWayPoint);

//            if(mLastKnownVehiclePose.position.x() > nextWayPoint.x())
//            {
                desiredRoll = sin(angleToTurnToWayPoint) * /*fmin(abs(mLastKnownVehiclePose.position.x() - nextWayPoint.x())*3,*/ 15.0/*)*/;
//            }
//            else
//            {
//                desiredRoll = sin(angleToTurnToWayPoint) * -fmin((nextWayPoint.x() - mLastKnownVehiclePose.position.x())*3, 15.0);
//            }
//            if(mLastKnownVehiclePose.position.z() > nextWayPoint.z())
//            {
                desiredPitch = cos(angleToTurnToWayPoint) * /*fmin(abs(mLastKnownVehiclePose.position.z() - nextWayPoint.z())*3,*/ 15.0/*)*/;
//            }
//            else
//            {
//                desiredPitch = cos(angleToTurnToWayPoint) * -fmin((nextWayPoint.z() - mLastKnownVehiclePose.position.z())*3, 15.0);
//            }

            qDebug() << "ROLL  DESIRED IS" << desiredRoll;
            qDebug() << "PITCH DESIRED IS" << desiredPitch;
        }

        // try to get ourselves straight up
        double errorPitch = desiredPitch + mLastKnownVehiclePose.getPitchDegrees();
        mErrorIntegralPitch += errorPitch*timeDiff;
        double derivativePitch = (errorPitch - mPrevErrorPitch + 0.00001)/timeDiff;
        double outputPitch = (Kp*errorPitch) + (Ki*mErrorIntegralPitch) + (0.0*Kd*derivativePitch);

        double errorRoll = desiredRoll - mLastKnownVehiclePose.getRollDegrees();
        mErrorIntegralRoll += errorRoll*timeDiff;
        double derivativeRoll = (errorRoll - mPrevErrorRoll + 0.00001)/timeDiff;
        double outputRoll = (Kp*errorRoll) + (Ki*mErrorIntegralRoll) + (0.0*Kd*derivativeRoll);

        double errorHeight = nextWayPoint.y() - mLastKnownVehiclePose.position.y();
        mErrorIntegralHeight += errorHeight*timeDiff;
        double derivativeHeight = (errorHeight - mPrevErrorHeight + 0.00001)/timeDiff;
        double outputThrust = (Kp*errorHeight) + (0.5*Ki*mErrorIntegralHeight) + (Kd*derivativeHeight);

        outputPitch /= 10.0;
        outputRoll /= 10.0;

        qDebug() << "values PRYH:" << QString::number(mLastKnownVehiclePose.getPitchDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getRollDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.getYawDegrees(), 'f', 2) << "\t" << QString::number(mLastKnownVehiclePose.position.y(), 'f', 2);
        qDebug() << "should PRYH:" << QString::number(desiredPitch, 'f', 2) << "\t" << QString::number(desiredRoll, 'f', 2) << "\t" << QString::number(mDesiredYaw, 'f', 2) << "\t" << QString::number(nextWayPoint.y(), 'f', 2);
        qDebug() << "error  PRYH:" << QString::number(errorPitch, 'f', 2) << "\t" << QString::number(errorRoll, 'f', 2) << "\t" << QString::number(errorYaw, 'f', 2) << "\t" << QString::number(errorHeight, 'f', 2);
        qDebug() << "derivt PRYH:" << QString::number(derivativePitch, 'f', 2) << "\t" << QString::number(derivativeRoll, 'f', 2) << "\t" << QString::number(derivativeYaw, 'f', 2) << "\t" << QString::number(derivativeHeight, 'f', 2);
        qDebug() << "prevEr PRYH:" << QString::number(mPrevErrorPitch, 'f', 2) << "\t" << QString::number(mPrevErrorRoll, 'f', 2) << "\t" << QString::number(mPrevErrorYaw, 'f', 2) << "\t" << QString::number(mPrevErrorHeight, 'f', 2);
        qDebug() << "inteEr PRYH:" << QString::number(mErrorIntegralPitch, 'f', 2) << "\t" << QString::number(mErrorIntegralRoll, 'f', 2) << "\t" << QString::number(mErrorIntegralYaw, 'f', 2) << "\t" << QString::number(mErrorIntegralHeight, 'f', 2);
        qDebug() << "output PRYH:" << QString::number(outputPitch, 'f', 2) << "\t" << QString::number(outputRoll, 'f', 2) << "\t" << QString::number(outputYaw, 'f', 2) << "\t" << QString::number(outputThrust, 'f', 2);

        mPrevErrorPitch = errorPitch;
        mPrevErrorRoll = errorRoll;
        mPrevErrorYaw = errorYaw;
        mPrevErrorHeight = errorHeight;

        out_thrust = outputThrust > 255.0 ? 255 : outputThrust < 0.0 ? 0 : (quint8)outputThrust;
        out_yaw = outputYaw > 127.0 ? 127 : outputYaw < -127.0 ? -127 : (qint8)outputYaw;
        out_pitch = outputPitch > 127.0 ? 127 : outputPitch < -127.0 ? -127 : (qint8)outputPitch;
        out_roll = outputRoll > 127.0 ? 127 : outputRoll < -127.0 ? -127 : (qint8)outputRoll;

        qDebug() << "FlightController::slotComputeMotionCommands(): motion is" << out_thrust << out_yaw << out_pitch << out_roll << 20;

//        emit motion(out_thrust, out_pitch, out_roll, out_yaw, 0);
        emit motion(out_thrust, -desiredPitch, desiredRoll, out_yaw, 0);
        emit debugValues(mLastKnownVehiclePose, out_thrust, out_pitch, out_roll, out_yaw, 0);

//        emit motion(100, 3, 0, 0, 20);

        // See whether we're close at the waypoint and moving slowly
        if(
                mLastKnownVehiclePose.position.distanceToLine(nextWayPoint, QVector3D()) < 2.0 // half-close to wp
                &&
                // slow, if no further wps present, so we can go land and switch to idle
                (
                    mWayPoints.size()
                    ||
                    mLastKnownVehiclePose.position.distanceToLine(mLastKnownVehiclePose.position, QVector3D()) < 0.01 // slow
                    &&
                    mLastKnownVehiclePose.position.distanceToLine(nextWayPoint, QVector3D()) < 0.10 // )
                ))
        {
            wayPointReached();
        }
    }
    else if(mFlightState == Idle)
    {
//        f = 0; b = 0; l = 0; r = 0;
        emit motion(0, 0, 0, 0, 0);
    }
    else
    {
        Q_ASSERT(false && "FlightState not defined!");
    }

    mTimeOfLastControllerUpdate = QTime::currentTime();

//    qDebug() << "engine spds:" << "f" << f << "\tb" << b << "\tl" << l << "\tr" << r << endl;
}



void FlightController::wayPointReached()
{
    // The current waypoint has been reached.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    Q_ASSERT(getFlightState() == ApproachingNextWayPoint);

    if(mWayPoints.size())
    {
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, "waypoint reached, more waypoints present, approaching");
    }
    else if(mLastKnownBottomBeamLength < 0.2)
    {
        setFlightState(Idle);
        // TODO: slow down first. Not necessary, we ARE slow when reaching a waypoint. Hopefully.
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, "waypoint reached, no more wayPoints, HeightAboveGround low, idling");
    }
    else
    {
        mWayPoints.append(getLandingWayPoint());
        emit message(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Information, QString("ManualControl disabled, no further wayPoints, HeightAboveGround high (" + QString::number(mLastKnownBottomBeamLength) + "m), now landing"));
    }

    emit wayPointReached(mWayPointsPassed.last());
    emit currentWayPoints(mWayPoints);
}

void FlightController::clearWayPoints()
{
    mWayPoints.clear();
    emit currentWayPoints(mWayPoints);
}

void FlightController::slotWayPointInsert(const QString &hashValue, const int index, const QVector3D &wayPoint)
{
    if(hash(mWayPoints) == hashValue)
    {
//        qDebug() << "FlightController::slotSetNextWayPoint(): inserting" << wayPoint << "into index" << index;
        mWayPoints.insert(index, wayPoint);
        emit currentWayPoints(mWayPoints);

        // Just in case we were idle (or landing, which is the same) before...
        if(mFlightState == Idle) setFlightState(ApproachingNextWayPoint);
    }
    //else Q_ASSERT(false && "hashfailure in slotWayPointInsert");
//    qDebug() <<  "FlightController::slotWayPointInsert():" << hashValue << index << wayPoint << "hash failure";

    emit currentWayPoints(mWayPoints);
}

void FlightController::slotWayPointDelete(const QString &hashValue, const int index)
{
    if(hash(mWayPoints) == hashValue)
    {
        mWayPoints.removeAt(index);

        // The list might now be empty, so me might have to land.
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
    }
    else Q_ASSERT("hash failure");
//    else
//    {
        emit currentWayPoints(mWayPoints);
//    }
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
    return WayPoint(mLastKnownVehiclePose.position - QVector3D(0.0, mLastKnownBottomBeamLength, 0.0));
}

FlightController::FlightState FlightController::getFlightState(void) const { return mFlightState; }

QString FlightController::getFlightStateString(void) const
{
    switch(mFlightState)
    {
    case ManualControl: return "ManualControl"; break;
    case ApproachingNextWayPoint: return "ApproachingNextWayPoint"; break;
//    case Landing: return "Landing"; break;
    case Freezing: return "Freezing"; break;
    case Idle: return "Idle"; break;
    }

    Q_ASSERT(false && "FLIGHTSTATE UNDEFINED!");
}

void FlightController::slotSetVehiclePose(const Pose& pose)
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
        mFlightState = flightState;
        emit flightStateChanged(flightState);
    }
}

void FlightController::slotScanningInProgress(const quint32& timestamp)
{
//    qDebug() << "FlightController::slotScanningInProgress()!";
    mTimeOfLastLaserScan = QTime::currentTime();
}

void FlightController::slotSetBottomBeamLength(const float& beamLength)
{
    mLastKnownBottomBeamLengthTimestamp = QTime::currentTime();
    mLastKnownBottomBeamLength = beamLength;
}
