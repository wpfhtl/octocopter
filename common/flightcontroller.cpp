#include "flightcontroller.h"
#include "motioncommand.h"

FlightController::FlightController(const QString& logFilePrefix) : QObject()
{
    mLogFile = 0;
    if(!logFilePrefix.isNull())
    {
        mLogFile = new QFile(logFilePrefix + QString("flightcontroller.flt"));
        if(!mLogFile->open(QIODevice::WriteOnly | QIODevice::Text))
            qFatal("FlightController::FlightController(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));
    }

    mFlightState = FlightState(FlightState::Value::UserControl);

    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mTimeOfLastControllerUpdate = QTime::currentTime();
    mFirstControllerRun = true;

    mLastKnownVehiclePose = Pose();
    mLastMotionCommand = MotionCommand((quint8)0, (qint8)0, (qint8)0, (qint8)0);

    mBackupTimerComputeMotion = new QTimer(this);
    connect(mBackupTimerComputeMotion, SIGNAL(timeout()), SLOT(slotComputeBackupMotion()));

    mImuOffsets.pitch = 0.0f;
    mImuOffsets.roll = 0.0f;
}

FlightController::~FlightController()
{
    mBackupTimerComputeMotion->stop();
    mBackupTimerComputeMotion->deleteLater();

    if(mLogFile)
    {
        qDebug() << "FlightController::~FlightController(): closing logfile...";
        mLogFile->close();
        mLogFile->deleteLater();
        qDebug() << "FlightController::~FlightController(): done.";
    }
}

void FlightController::slotComputeBackupMotion()
{
    qDebug() << "FlightController::slotComputeBackupMotion(): no pose for" << mBackupTimerComputeMotion->interval() << "ms, flightstate" << mFlightState.toString() << ": starting backup motion computation.";
    Q_ASSERT(mFlightState != FlightState::Value::UserControl && "FlightController::slotComputeBackupMotion(): i was called in FlightState UserControl");
    slotComputeMotionCommands();
}

// WARNING: We computer safe motion commands when the pose is not precise or old. If it turns out such poses are unavoidable at times,
// it might be better not to generate motion at all, because a single safe-motion between otherwise valid motion-commands can cause
// the helicopter to start bouncing in the air. We need to test this!
void FlightController::slotComputeMotionCommands()
{
    Q_ASSERT(mFlightState != FlightState::Value::UserControl && "FlightController::slotComputeMotionCommands(): i was called in FlightState UserControl");
    Q_ASSERT(mFlightState != FlightState::Value::Undefined && "FlightController::slotComputeMotionCommands(): i was called in FlightState Undefined");

    // This can be set for all the cases below.
    // mLastFlightControllerValues.mFlightState is already set in slotNewVehiclePose()
    mLastFlightControllerValues.lastKnownHeightOverGround = mLastKnownHeightOverGround;

    switch(mFlightState.state)
    {

    case FlightState::Value::Idle:
    {
        qDebug() << "FlightController::slotComputeMotionCommands(): FlightState: Idle, emitting idle-thrust.";
        mLastFlightControllerValues.targetPosition = mLastKnownVehiclePose.getPosition();
        mLastFlightControllerValues.motionCommand = MotionCommand((quint8)90, 0.0f, 0.0f, 0.0f);
        break;
    }

    case FlightState::Value::ApproachWayPoint:
    {
        Q_ASSERT(mWayPoints.size() && "I'm in ApproachWayPoint, but there's no more waypoints present!");

        mLastFlightControllerValues.targetPosition = mWayPoints.first();

        // Can be overridden with better non-default values below
        mLastFlightControllerValues.motionCommand = MotionCommand(MotionCommand::thrustHover, 0.0f, 0.0f, 0.0f);

        if(
                mLastKnownVehiclePose.getAge() < 80
                // If pose is unprecise, emit safe stuff. Note that we don't expect ModeIntegrated, because only 1 of 2_or_5 packets is integrated - and thats ok.
                && mLastKnownVehiclePose.precision & Pose::AttitudeAvailable
                && mLastKnownVehiclePose.precision & Pose::HeadingFixed
                && mLastKnownVehiclePose.precision & Pose::RtkFixed
                )
        {
            const WayPoint nextWayPoint = mWayPoints.first();

            const QVector2D vectorVehicleToWayPoint = (nextWayPoint.getPositionOnPlane() - mLastKnownVehiclePose.getPlanarPosition());
            const float directionNorthToWayPointRadians = atan2(-vectorVehicleToWayPoint.x(), -vectorVehicleToWayPoint.y());
            const float angleToTurnToWayPoint = Pose::getShortestTurnRadians(directionNorthToWayPointRadians - mLastKnownVehiclePose.getYawRadians());

            // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!

            // If the planar distance to the next waypoint is very small (this happens when only the height is off),
            // we don't want to yaw and pitch. So, we introduce a factor [0;1], which becomes 0 with small distance
            const float planarDistanceToWayPoint = (mLastKnownVehiclePose.getPlanarPosition() - Pose::getPlanarPosition(nextWayPoint)).length();
            const float factorPlanarDistance = qBound(0.0f, planarDistanceToWayPoint * 2.0f, 1.0f);

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

            qDebug() << "FlightController::slotComputeMotionCommands(): ApproachWayPoint, lastKnownPose:" << mLastKnownVehiclePose.toStringVerbose() << "nextWayPoint:" << nextWayPoint;
            qDebug() << "FlightController::slotComputeMotionCommands(): timediff:" << timeDiff << "angleToTurn" << RAD2DEG(angleToTurnToWayPoint) << "planar distance:" << planarDistanceToWayPoint << "factorPlanarDistance" << factorPlanarDistance << "factorHeight" << factorHeight;

            // If angleToTurnToWayPoint is:
            // - positive, we need to rotate CCW, which needs a negative yaw value.
            // - negative, we need to rotate  CW, which needs a positive yaw value.
            const float errorYaw = RAD2DEG(angleToTurnToWayPoint);
            mErrorIntegralYaw += errorYaw * timeDiff;
            const float derivativeYaw = mFirstControllerRun ? 0.0f : (errorYaw - mPrevErrorYaw + 0.00001f)/timeDiff;
            const float outputYaw = factorHeight * factorPlanarDistance * ((1.0f * errorYaw) + (0.0f * mErrorIntegralYaw) + (0.3f/*0.5f*/ * derivativeYaw));

            // adjust pitch/roll to reach target, maximum pitch is -20 degrees (forward)
            if(mApproachPhase == ApproachPhase::OrientTowardsTarget && errorYaw < 3.0f)
            {
                qDebug() << "FlightController::slotComputeMotionCommands(): pointing at target, switching from orientation to approach phase";
                mApproachPhase == ApproachPhase::ApproachTarget;
            }

            float desiredRoll = 0.0f;
            float desiredPitch = 0.0f;

            // In ApproachTarget, activate roll to correct lateral offset
            if(mApproachPhase == ApproachPhase::ApproachTarget)
            {
                // Lateral offset between vehicle and line. A positive value means the
                // vehicle is too far on the right, so it should roll positively.
                float lateralOffsetFromLine = sin(angleToTurnToWayPoint) * planarDistanceToWayPoint;
                desiredRoll = qBound(-10.0f, lateralOffsetFromLine * 10.0f, 10.0f);
                desiredRoll *= factorPlanarDistance;
                qDebug() << "FlightController::slotComputeMotionCommands(): lateral vehicle offset: vehicle is" << lateralOffsetFromLine << "m too far" << (lateralOffsetFromLine > 0.0f ? "right" : "left") << "- desiredRoll is" << desiredRoll;
            }
            else if(mApproachPhase == ApproachPhase::OrientTowardsTarget)
            {
                // TODO: stay on position while turning!
            }
            desiredPitch = -pow(20.0f - qBound(0.0, fabs(errorYaw), 20.0), 2.0f) / 20.0f;
            desiredPitch *= factorPlanarDistance;


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
            const float outputThrust = MotionCommand::thrustHover + (25.0f * errorHeight) + (0.001f * mErrorIntegralHeight) + (1.0f * derivativeHeight);

            qDebug() << mWayPoints.size() << "waypoints, next wpt height" << nextWayPoint.y() << "curr height" << mLastKnownVehiclePose.getPosition().y();

            qDebug() << qSetRealNumberPrecision(4) << "values PRYH:" << currentPitch << "\t" << currentRoll << "\t" << mLastKnownVehiclePose.getYawDegrees() << "\t" << mLastKnownVehiclePose.getPosition().y();
            qDebug() << qSetRealNumberPrecision(4) << "should PRYH:" << desiredPitch << "\t" << desiredRoll << "\t" << RAD2DEG(directionNorthToWayPointRadians) << "\t" << nextWayPoint.y();
            qDebug() << qSetRealNumberPrecision(4) << "error  PRYH:" << errorPitch << "\t" << errorRoll << "\t" << errorYaw << "\t" << errorHeight;
            qDebug() << qSetRealNumberPrecision(4) << "derivt PRYH:" << derivativePitch << "\t" << derivativeRoll << "\t" << derivativeYaw << "\t" << derivativeHeight;
            qDebug() << qSetRealNumberPrecision(4) << "prevEr PRYH:" << mPrevErrorPitch << "\t" << mPrevErrorRoll << "\t" << mPrevErrorYaw << "\t" << mPrevErrorHeight;
            qDebug() << qSetRealNumberPrecision(4) << "inteEr PRYH:" << mErrorIntegralPitch << "\t" << mErrorIntegralRoll << "\t" << mErrorIntegralYaw << "\t" << mErrorIntegralHeight;
            qDebug() << qSetRealNumberPrecision(4) << "output PRYH:" << outputPitch << "\t" << outputRoll << "\t" << outputYaw << "\t" << outputThrust;

            mPrevErrorPitch = errorPitch;
            mPrevErrorRoll = errorRoll;
            mPrevErrorYaw = errorYaw;
            mPrevErrorHeight = errorHeight;

            mFirstControllerRun = false;

            mLastFlightControllerValues.motionCommand = MotionCommand(outputThrust, outputYaw, outputPitch, outputRoll);

            // See whether we've reached the waypoint
            if(mLastKnownVehiclePose.getPosition().distanceToLine(nextWayPoint, QVector3D()) < 0.80f) // close to wp
            {
                nextWayPointReached();
            }
        }
        else
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): ApproachWayPoint, pose precision" << mLastKnownVehiclePose.precision << "- update is from" << mLastKnownVehiclePose.timestamp << " - age in ms:" << mLastKnownVehiclePose.getAge() << "- not overwriting safe hover values";
        }

        break;
    }

    case FlightState::Value::Hover:
    {
        mLastFlightControllerValues.targetPosition = mHoverPosition;

        // Can be overridden with better non-default values below
        mLastFlightControllerValues.motionCommand = MotionCommand(MotionCommand::thrustHover, 0.0f, 0.0f, 0.0f);

        if(
                mLastKnownVehiclePose.getAge() < 80
                // If pose is unprecise, emit safe stuff. Note that we don't expect ModeIntegrated, because only 1 of 2_or_5 packets is integrated - and thats ok.
                && mLastKnownVehiclePose.precision & Pose::AttitudeAvailable
                && mLastKnownVehiclePose.precision & Pose::HeadingFixed
                // && mLastKnownVehiclePose.precision & Pose::RtkFixed // Hover even in non-precise modes
                )
        {
            const QVector2D vectorVehicleToOrigin = -mLastKnownVehiclePose.getPlanarPosition();

            // We want to point exactly away from the origin, because thats probably where the user is
            // standing - steering is easier if the vehicle's forward arm points away from the user.
            const float angleToTurnAwayFromOrigin = RAD2DEG(Pose::getShortestTurnRadians(-Pose::getShortestTurnRadians(DEG2RAD(180.0f) - atan2(-vectorVehicleToOrigin.x(), -vectorVehicleToOrigin.y())) - mLastKnownVehiclePose.getYawRadians()));

            qDebug() << "FlightController::slotComputeMotionCommands(): FlightState Hover, mHoverPosition:" << mHoverPosition << "angleToTurnAwayFromOrigin: turn" << (angleToTurnAwayFromOrigin < 0.0f ? "right" : "left") << angleToTurnAwayFromOrigin;

            // If the planar distance to mHoverPosition is very small (this happens when only the height is off),
            // we don't want to yaw and pitch. So, we introduce a factor [0;1], which becomes 0 with small distance
            const float factorPlanarDistance = qBound(0.0f, (float)(mLastKnownVehiclePose.getPlanarPosition() - mHoverPosition).length(), 1.0f);

            // Elapsed time since last call in seconds. May not become zero (divide by zero)
            // and shouldn't grow too high, as that will screw up the controllers.
            const float timeDiff = qBound(
                        0.01f,
                        mTimeOfLastControllerUpdate.msecsTo(QTime::currentTime()) / 1000.0f,
                        0.2f);

            // Now that we've yawed to look away from the origin, pitch/roll to move towards hover-position
            const QVector3D vectorVehicleToHoverPosition = mLastKnownVehiclePose.getPlanarPosition() - QVector2D(mHoverPosition.x(), mHoverPosition.z());
            const float angleToTurnToHoverPosition = Pose::getShortestTurnRadians(
                        atan2(-vectorVehicleToHoverPosition.x(), -vectorVehicleToHoverPosition.y())
                        - mLastKnownVehiclePose.getYawRadians()
                        );

            float desiredPitch = cos(-angleToTurnToHoverPosition) * 10.0f;
            float desiredRoll = sin(-angleToTurnToHoverPosition) * 10.0f;

            desiredPitch = qBound(
                        -10.0,
                        pow(factorPlanarDistance, 4.0) * desiredPitch,
                        10.0);

            desiredRoll  = qBound(
                        -10.0,
                        pow(factorPlanarDistance, 4.0) * desiredRoll,
                        10.0);

            const float errorYaw = angleToTurnAwayFromOrigin * factorPlanarDistance;
            mErrorIntegralYaw += errorYaw * timeDiff;
            const float derivativeYaw = mFirstControllerRun ? 0.0f : (errorYaw - mPrevErrorYaw + 0.00001f)/timeDiff;
            const float outputYaw = ((1.0f * errorYaw) + (0.0f * mErrorIntegralYaw) + (0.3f/*0.5f*/ * derivativeYaw));

            const float currentPitch = mLastKnownVehiclePose.getPitchDegrees() - mImuOffsets.pitch;
            const float errorPitch = desiredPitch - currentPitch;
            mErrorIntegralPitch += errorPitch * timeDiff;
            const float derivativePitch = mFirstControllerRun ? 0.0f : (errorPitch - mPrevErrorPitch + 0.00001f)/timeDiff;
            const float outputPitch = (3.0f * errorPitch) + (0.2f * mErrorIntegralPitch) + (0.0f * derivativePitch);

            const float currentRoll = mLastKnownVehiclePose.getRollDegrees() - mImuOffsets.roll;
            const float errorRoll = desiredRoll - currentRoll;
            mErrorIntegralRoll += errorRoll * timeDiff;
            const float derivativeRoll = mFirstControllerRun ? 0.0f : (errorRoll - mPrevErrorRoll + 0.00001f)/timeDiff;
            const float outputRoll = (3.0f * errorRoll) + (0.2f * mErrorIntegralRoll) + (0.0f * derivativeRoll);

            const float currentHeight = mLastKnownVehiclePose.getPosition().y();
            const float errorHeight = mHoverPosition.y() - currentHeight;
            mErrorIntegralHeight += errorHeight * timeDiff;
            const float derivativeHeight = mFirstControllerRun ? 0.0f : (errorHeight - mPrevErrorHeight + 0.00001f)/timeDiff;
            const float outputThrust = MotionCommand::thrustHover + (25.0f * errorHeight) + (0.001f * mErrorIntegralHeight) + (1.0f * derivativeHeight);

            qDebug() << qSetRealNumberPrecision(4) << "values PRYH:" << currentPitch << "\t" << currentRoll << "\t" << mLastKnownVehiclePose.getYawDegrees() << "\t" << mLastKnownVehiclePose.getPosition().y();
            qDebug() << qSetRealNumberPrecision(4) << "should PRYH:" << desiredPitch << "\t" << desiredRoll << "\t" << atan2(-vectorVehicleToOrigin.x(), -vectorVehicleToOrigin.y()) << "\t" << mHoverPosition.y();
            qDebug() << qSetRealNumberPrecision(4) << "error  PRYH:" << errorPitch << "\t" << errorRoll << "\t" << angleToTurnAwayFromOrigin << "\t" << errorHeight;
            qDebug() << qSetRealNumberPrecision(4) << "derivt PRYH:" << derivativePitch << "\t" << derivativeRoll << "\t" << derivativeYaw << "\t" << derivativeHeight;
            qDebug() << qSetRealNumberPrecision(4) << "prevEr PRYH:" << mPrevErrorPitch << "\t" << mPrevErrorRoll << "\t" << mPrevErrorYaw << "\t" << mPrevErrorHeight;
            qDebug() << qSetRealNumberPrecision(4) << "inteEr PRYH:" << mErrorIntegralPitch << "\t" << mErrorIntegralRoll << "\t" << mErrorIntegralYaw << "\t" << mErrorIntegralHeight;
            qDebug() << qSetRealNumberPrecision(4) << "output PRYH:" << outputPitch << "\t" << outputRoll << "\t" << outputYaw << "\t" << outputThrust;

            mPrevErrorPitch = errorPitch;
            mPrevErrorRoll = errorRoll;
            mPrevErrorYaw = errorYaw;
            mPrevErrorHeight = errorHeight;
            mFirstControllerRun = false;

            mLastFlightControllerValues.motionCommand = MotionCommand(outputThrust, outputYaw, outputPitch, outputRoll);
        }
        else
        {
            qDebug() << "FlightController::slotComputeMotionCommands(): Hover, pose precision" << mLastKnownVehiclePose.precision << "- update is from" << mLastKnownVehiclePose.timestamp << " - age in ms:" << mLastKnownVehiclePose.getAge() << "- not overwriting safe hover values";
        }

        break;
    }

    default:
        qDebug() << "FlightController::slotComputeMotionCommands(): FLIGHTSTATE NOT DEFINED:" << mFlightState.toString();
        Q_ASSERT(false);
    }

    mTimeOfLastControllerUpdate = QTime::currentTime();

    smoothenControllerOutput(mLastFlightControllerValues.motionCommand);

    qDebug() << "FlightController::slotComputeMotionCommands(): after smoothing:" << mLastFlightControllerValues.motionCommand;
    emit motion(mLastFlightControllerValues.motionCommand);

    // Mostly useful in simulator
    emit flightControllerValues(mLastFlightControllerValues);

    logFlightControllerValues();
}

void FlightController::smoothenControllerOutput(MotionCommand& mc)
{
    mc.thrust = (quint8)(((quint16)mc.thrust + (quint16)mLastMotionCommand.thrust) / 2.0f);
    mc.yaw = (quint8)(((quint16)mc.yaw + (quint16)mLastMotionCommand.yaw) / 2.0f);
    mc.pitch = (quint8)(((quint16)mc.pitch + (quint16)mLastMotionCommand.pitch) / 2.0f);
    mc.roll = (quint8)(((quint16)mc.roll + (quint16)mLastMotionCommand.roll) / 2.0f);

    mLastMotionCommand = mc;
}

void FlightController::logFlightControllerValues()
{
    if(mLogFile)
    {
        QTextStream out(mLogFile);
        out << GnssTime::currentTow() << ' ' << mLastFlightControllerValues.toString() << '\n';
    }
}

void FlightController::slotCalibrateImu()
{
    mImuOffsets.pitch = mLastKnownVehiclePose.getPitchDegrees();
    mImuOffsets.roll = mLastKnownVehiclePose.getRollDegrees();
    qDebug() << "FlightController::slotCalibrateImu(): calibrated IMU offsets to pitch" << mImuOffsets.pitch << "and roll" << mImuOffsets.roll << "from pose with age" << mLastKnownVehiclePose.getAge();
}

void FlightController::nextWayPointReached()
{
    Q_ASSERT(mFlightState == FlightState::Value::ApproachWayPoint && "FlightController::nextWayPointReached(): reached waypoint, but am not in ApproachWayPoint state. Expect trouble!");

    // The next waypoint has been reached.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    qDebug() << "FlightController::nextWayPointReached(): distance" << mLastKnownVehiclePose.getPosition().distanceToLine(mWayPointsPassed.last(), QVector3D()) << " - reached waypoint" << mWayPointsPassed.last();

    // First emit this signal for the base to sync the list, THEN call
    // ensureSafeFlightAfterWaypointsChanged(), which might append more waypoints.
    emit wayPointReached(mWayPointsPassed.last());

    ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(mWayPoints);
}

void FlightController::slotWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "FlightController::slotSetNextWayPoint(): state is" << mFlightState.toString() << "inserting waypoint" << wayPoint << "into index" << index;
    mWayPoints.insert(index, wayPoint);
    emit currentWayPoints(mWayPoints);

    // Just in case we were idle before...
    if(mFlightState == FlightState::Value::Idle)
    {
        qDebug() << "FlightController::slotWayPointInsert(): we were idle, switching to ApproachWayPoint";
        setFlightState(FlightState::Value::ApproachWayPoint);
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
        if(mFlightState == FlightState::Value::ApproachWayPoint)
            ensureSafeFlightAfterWaypointsChanged();
    }
    qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, emitting new wpt list of size" << mWayPoints.size();
    emit currentWayPoints(mWayPoints);
}

void FlightController::slotSetWayPoints(const QList<WayPoint>& wayPoints)
{
    mWayPoints = wayPoints;

    // Just in case we were idle before...
    if(mFlightState == FlightState::Value::Idle && mWayPoints.size())
    {
        qDebug() << "FlightController::slotSetWayPoints(): we were idle and got new waypoints, switching to ApproachWayPoint";
        setFlightState(FlightState::Value::ApproachWayPoint);
    }

    // The list might now be empty, so we might have to land.
    if(mFlightState == FlightState::Value::ApproachWayPoint)
        ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(mWayPoints);
}

void FlightController::slotNewVehiclePose(const Pose& pose)
{
    qDebug() << "FlightController::slotNewVehiclePose(): flightstate:" << mFlightState.toString() << ": new pose from" << pose.timestamp << "has age" << GnssTime::currentTow() - pose.timestamp;

    // Whatever precision and flightstate, save the pose. We also save unprecise poses here, which comes in handy
    // for IMU calibration (which doesn't require e.g. RTK). Instead, IMU calibration requires a very recent pose.
    //
    // Of course, this means that we need to check this pose before we actually use it for flight control!
    mLastKnownVehiclePose = pose;

    mLastFlightControllerValues = FlightControllerValues();
    mLastFlightControllerValues.lastKnownPose = mLastKnownVehiclePose;
    mLastFlightControllerValues.flightState = mFlightState;

    switch(mFlightState.state)
    {
    case FlightState::Value::UserControl:
    {
        // Keep ourselves disabled in UserControl.
        logFlightControllerValues();
        Q_ASSERT(!mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): UserControl has active backup timer!");
        break;
    }

    case FlightState::Value::Idle:
    {
        // Lets compute motion (although its really simple in this case)
        slotComputeMotionCommands();

        Q_ASSERT(mBackupTimerComputeMotion->interval() == backupTimerIntervalSlow && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Idle has inactive backup timer or wrong interval!");
        break;
    }

    case FlightState::Value::ApproachWayPoint:
    {
        Q_ASSERT(mBackupTimerComputeMotion->interval() == backupTimerIntervalFast && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): ApproachWayPoint has inactive backup timer or wrong interval!");

        // This method was called with a new pose, so lets use it to compute motion
        slotComputeMotionCommands();

        // Re-set the safety timer, making it fire 50ms in the future - not when its scheduled, which might be sooner.
        // So that when no new pose comes in for too long, we'll compute a backup motion.
        mBackupTimerComputeMotion->start(backupTimerIntervalFast);

        break;
    }

    case FlightState::Value::Hover:
    {
        Q_ASSERT(mBackupTimerComputeMotion->interval() == backupTimerIntervalFast && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Hover has inactive backup timer or wrong interval!");

        // This method was called with a new pose, so lets use it to compute motion
        slotComputeMotionCommands();

        // Re-set the safety timer, making it fire 50ms in the future - not when its scheduled, which might be sooner.
        // So that when no new pose comes in for too long, we'll compute a backup motion.
        mBackupTimerComputeMotion->start(backupTimerIntervalFast);

        break;
    }

    default:
        Q_ASSERT(false && "illegal flightstate in FlightController::slotNewVehiclePose()!");
    }

}

void FlightController::setFlightState(FlightState newFlightState)
{
    qDebug() << "FlightController::setFlightState():" << mFlightState.toString() << "=>" << newFlightState.toString();

    if(mFlightState == newFlightState)
    {
        qDebug() << "FlightController::setFlightState(): switching to same flightstate doesn't make much sense, returning.";
        return;
    }

    switch(newFlightState.state)
    {
    case FlightState::Value::ApproachWayPoint:
    {
        qDebug() << "FlightController::setFlightState(): ApproachWayPoint - initializing controllers, setting backup motion timer to high-freq";

        // We're going to use the controllers, so make sure to initialize them.
        initializeControllers();

        // We don't need to set mTimeOfLastControllerUpdate, as that is qBound()ed anyway.

        mBackupTimerComputeMotion->start(backupTimerIntervalFast);

        mFlightState = newFlightState;

        // Make sure there actually ARE waypoints. If not, create some for landing.
        ensureSafeFlightAfterWaypointsChanged();
        break;
    }

    case FlightState::Value::Hover:
    {
        mHoverPosition = mLastKnownVehiclePose.getPosition();

        if(mLastKnownVehiclePose.getAge() > 100)
            qDebug() << "FlightController::setFlightState(): WARNING: going to hover, but age of pose being set as hover-target is" << mLastKnownVehiclePose.getAge() << "ms!";

        qDebug() << "FlightController::setFlightState(): going to hover, setting backup motion timer to high-freq and staying at" << mHoverPosition;

        // We're going to use the controllers, so make sure to initialize them.
        initializeControllers();

        mBackupTimerComputeMotion->start(backupTimerIntervalFast);
        mFlightState = newFlightState;
        break;
    }

    case FlightState::Value::UserControl:
    {
        // We don't need the timer, as the remote control will overrule our output anyway.
        qDebug() << "FlightController::setFlightState(): disabling backup motion timer.";
        mBackupTimerComputeMotion->stop();
        mFlightState = newFlightState;
        break;
    }

    case FlightState::Value::Idle:
    {
        qDebug() << "FlightController::setFlightState(): setting backup motion timer to low-freq";
        mBackupTimerComputeMotion->start(backupTimerIntervalSlow);
        mFlightState = newFlightState;
        break;
    }

    default:
        Q_ASSERT(false && "FlightController::setFlightState(): undefined flightstate");
    }

    emit flightStateChanged(mFlightState);
}

void FlightController::initializeControllers()
{
    qDebug() << "FlightController::initializeControllers(): initializing controller values to almost-zero.";
    mPrevErrorPitch = 0.1f;
    mPrevErrorRoll = 0.1f;
    mPrevErrorYaw = 0.1f;
    mPrevErrorHeight = 0.1f;

    mErrorIntegralPitch = 0.1f;
    mErrorIntegralRoll = 0.1f;
    mErrorIntegralYaw = 0.1f;
    mErrorIntegralHeight = 0.1;

    mFirstControllerRun = true;

    // We want to approach a target using roll only after the vehicle points at it.
//    mApproachUsingRoll = false;
    mApproachPhase = ApproachPhase::OrientTowardsTarget;
}

void FlightController::slotSetHeightOverGround(const float& beamLength)
{
    // Height is given from vehicle center to ground, but we care about the bottom of the landing gear.
    qDebug() << "FlightController::slotSetHeightOverGround()" << beamLength - 0.16f << "meters";
    mLastKnownHeightOverGroundTimestamp = QTime::currentTime();
    mLastKnownHeightOverGround = beamLength - 0.16f;
}

bool FlightController::isHeightOverGroundValueRecent() const
{
    qDebug() << "FlightController::isHeightOverGroundValueRecent(): age of last heightOverGround measurement is" << mLastKnownHeightOverGroundTimestamp.msecsTo(QTime::currentTime()) << "msecs";
    return mLastKnownHeightOverGroundTimestamp.msecsTo(QTime::currentTime()) < 750;
}

// To be called after waypoints have changed to check for dangerous states:
// If wpt list is empty and state is ApproachWayPoint, will either
//  - idle if low
//  - get a landing waypoint if heightOverGround is known and up-to-date
//  - descend slowly if heightOverGround is unknown
void FlightController::ensureSafeFlightAfterWaypointsChanged()
{
    initializeControllers();

    Q_ASSERT(mFlightState == FlightState::Value::ApproachWayPoint && "FlightController::ensureSafeFlightAfterWaypointsChanged(): flightstate is NOT ApproachWayPoint!");

    if(mWayPoints.size() == 0)
    {
        qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, going to hover";
        setFlightState(FlightState::Value::Hover);

        /* Still too untested
        // The method has debug output, so call it just once for now.
        const bool heightOverGroundValueRecent = isHeightOverGroundValueRecent();

        // The list is now empty and we are still flying.
        if(heightOverGroundValueRecent && mLastKnownHeightOverGround < 0.3f)
        {
            // We're low anyway, just got to idle mode.
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, we're low with valid heightOverGround of" << mLastKnownHeightOverGround << ", idling";
            setFlightState(Idle);
        }
        else if(heightOverGroundValueRecent)
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
        */
    }
}

void FlightController::slotFlightStateSwitchValueChanged(const FlightStateSwitch& fssv)
{
    // This method does nothing more than some sanity checks and verbose flightstae switching.

    qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): flightstate" << mFlightState.toString() << "FlightStateSwitch changed to:" << fssv.toString();

    switch(mFlightState.state)
    {
    case FlightState::Value::UserControl:
        switch(fssv.value)
        {
        case FlightStateSwitch::UserControl:
            // This is an illegal state: We are already in UserControl, and now we're
            // told that the user has disabled computerControl. Thats impossible.
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in UserControl and now user switched to UserControl. Error.");
            break;
        case FlightStateSwitch::Hover:
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::ApproachWayPoint:
            // We are in UserControl, but switching to ComputerControl.
            setFlightState(FlightState::Value::ApproachWayPoint);
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    case FlightState::Value::ApproachWayPoint:
        switch(fssv.value)
        {
        case FlightStateSwitch::UserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Hover:
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::ApproachWayPoint:
            // We are approaching waypoints, and now the user changed to ComputerControl?
            // Thats impossible, because we are already in ComputerControl (ApproachWayPoint)
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in ApproachWayPoint and now user switched to ApproachWayPoint. Error.");
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    case FlightState::Value::Hover:
        switch(fssv.value)
        {
        case FlightStateSwitch::UserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Hover:
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in Hover and now user switched to Hover. Error.");
            break;
        case FlightStateSwitch::ApproachWayPoint:
            setFlightState(FlightState::Value::ApproachWayPoint);
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    case FlightState::Value::Idle:
        switch(fssv.value)
        {
        case FlightStateSwitch::UserControl:
            // We are idling, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Hover:
            // We are idling, but the user wants us to hover. Hovering on the ground is not healthy!
            qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): going from idle to hover, this doesn't seem like a good idea, as the current height will be kept.";
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::ApproachWayPoint:
            // We are in UserControl, but switching to ComputerControl.
            setFlightState(FlightState::Value::ApproachWayPoint);
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    default:
        Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): illegal flightstate!");
    }

    qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): done, new flightstate" << mFlightState.toString();
}

void FlightController::slotEmitFlightState()
{
    emit flightStateChanged(mFlightState);
}
