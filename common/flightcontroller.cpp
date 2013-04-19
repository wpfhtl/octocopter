#include "flightcontroller.h"
#include "motioncommand.h"
#include <profiler.h>

FlightController::FlightController(const QString& logFilePrefix) : QObject()
{
    mLogFile = new LogFile(logFilePrefix + QString("flightcontroller.flt"), LogFile::Encoding::Binary);
    qDebug()<< "Size of FlightControllerValues:" << sizeof(FlightControllerValues);

    mBackupTimerComputeMotion = new QTimer(this);
    connect(mBackupTimerComputeMotion, SIGNAL(timeout()), SLOT(slotComputeBackupMotion()));

    // Initialize default/template controllers
    QMap<QChar, float> weights;
    QMap<PidController*, QMap<QChar, float> > controllerWeights;

    controllerWeights.clear();
    // Hover - Thrust
    weights.clear();
    weights.insert('p', 50.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 20.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerThrust, weights);

    // Hover - Yaw
    weights.clear();
    weights.insert('p', 3.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 1.5f);
    controllerWeights.insert(&mFlightControllerValues.controllerYaw, weights);

    // Hover - Pitch / Roll
    weights.clear();
    weights.insert('p', 5.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 25.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerPitch, weights);
    controllerWeights.insert(&mFlightControllerValues.controllerRoll, weights);
    mFlightControllerWeights.insert(FlightState::Value::Hover, controllerWeights);

/* same values for hover and approachwaypoint, they're doing pretty much the same anyway

    controllerWeights.clear();
    // ApproachWayPoint - Thrust
    weights.clear();
    weights.insert('p', 50.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 20.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerThrust, weights);

    // ApproachWayPoint - Yaw
    weights.clear();
    weights.insert('p', 3.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 1.5f);
    controllerWeights.insert(&mFlightControllerValues.controllerYaw, weights);

    // ApproachWayPoint - Pitch
    weights.clear();
    weights.insert('p', 3.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 10.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerPitch, weights);

    // ApproachWayPoint - Roll
    weights.clear();
    weights.insert('p', 3.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 10.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerRoll, weights);*/

    mFlightControllerWeights.insert(FlightState::Value::ApproachWayPoint, controllerWeights);

    // If we don't use a laserscanner (for whatever reason during testing),
    // set mFlightControllerValues.lastKnownHeightOverGround to 1.0 so it doesn't prevent pitch/roll
    mFlightControllerValues.lastKnownHeightOverGround = 1.0f;

    setFlightState(FlightState(FlightState::Value::UserControl));
}

FlightController::~FlightController()
{
    qDebug() << "FlightController::~FlightController(): shutting down";
    mBackupTimerComputeMotion->stop();
    mBackupTimerComputeMotion->deleteLater();

    qDebug() << "FlightController::~FlightController(): deleting logfile...";
    delete mLogFile;
    qDebug() << "FlightController::~FlightController(): done.";
}

void FlightController::slotComputeBackupMotion()
{
    qDebug() << "FlightController::slotComputeBackupMotion(): no pose for" << mBackupTimerComputeMotion->interval() << "ms, flightstate" << mFlightControllerValues.flightState.toString() << ": starting backup motion computation.";
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::Value::UserControl && "FlightController::slotComputeBackupMotion(): i was called in FlightState UserControl");
    slotComputeMotionCommands();
}

// WARNING: We compute safe motion commands when the pose is not precise or old. If it turns out such poses are unavoidable at times,
// it might be better not to generate motion at all, because a single safe-motion between otherwise valid motion-commands can cause
// the helicopter to start bouncing in the air. We need to test this!
void FlightController::slotComputeMotionCommands()
{
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::Value::UserControl && "FlightController::slotComputeMotionCommands(): i was called in FlightState UserControl");
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::Value::Undefined && "FlightController::slotComputeMotionCommands(): i was called in FlightState Undefined");

    if(mFlightControllerValues.flightState.state == FlightState::Value::Idle)
    {
        qDebug() << "FlightController::slotComputeMotionCommands(): FlightState: Idle, emitting idle-thrust.";
        mFlightControllerValues.motionCommand = MotionCommand((quint8)90, 0.0f, 0.0f, 0.0f);
    }
    else if(mFlightControllerValues.flightState.state == FlightState::Value::ApproachWayPoint || mFlightControllerValues.flightState.state == FlightState::Value::Hover)
    {
        // Can be overridden with better non-default values below
        mFlightControllerValues.motionCommand = MotionCommand(MotionCommand::thrustHover, 0.0f, 0.0f, 0.0f);

        // Only use poses that are sufficiently precise. Else, keep safe hover values.
        if(mFlightControllerValues.lastKnownPose.isSufficientlyPreciseForFlightControl())
        {
            // When hovering, we want to look away fro the origin. When approaching a waypoint, we want to look at the hoverPosition.
            // This variable shall hold the desired yaw angle.
            float angleToYawDegrees = 0.0f;

            if(mFlightControllerValues.flightState.state == FlightState::Value::ApproachWayPoint)
            {
                // positionStart has been set already, lets define positionGoal. As that may change in-flight,
                // we re-set it here in every iteration.
                mFlightControllerValues.trajectoryGoal = mWayPoints.first();

                // Yaw: We want the vehicle to look NOT towards positionGoal, but always look in the same direction as the line from start to goal - even if it has drifted off that line!
                // Corrections will be done by rolling
                const QVector3D trajectory = mFlightControllerValues.trajectoryGoal - mFlightControllerValues.trajectoryStart;
                angleToYawDegrees = RAD2DEG(Pose::getShortestTurnRadians(-Pose::getShortestTurnRadians(- atan2(-trajectory.x(), -trajectory.z())) - mFlightControllerValues.lastKnownPose.getYawRadians()));

                // If we're close to it, move the hoverPosition towards trajectoryGoal
                mFlightControllerValues.hoverPosition = getHoverPosition(
                            mFlightControllerValues.trajectoryStart,
                            mFlightControllerValues.trajectoryGoal,
                            mFlightControllerValues.lastKnownPose.getPosition(),
                            2.0f);
            }
            else if(mFlightControllerValues.flightState.state == FlightState::Value::Hover)
            {
                const QVector3D vectorVehicleToOrigin = -mFlightControllerValues.lastKnownPose.getPosition();

                // We want to point exactly away from the origin, because thats probably where the user is
                // standing - steering is easier if the vehicle's forward arm points away from the user.
                angleToYawDegrees = RAD2DEG(
                            Pose::getShortestTurnRadians(
                                atan2(-vectorVehicleToOrigin.x(), -vectorVehicleToOrigin.z())
                                - DEG2RAD(180.0f)
                                - mFlightControllerValues.lastKnownPose.getYawRadians()
                                )
                            );

//                Q_ASSERT(fabs(angleToYawDegrees - angleToYawDegrees2) < 0.1);
            }

            // If we give the yaw controller our current yaw (e.g. -170 deg) and our desired value (e.g. +170),
            // it would compute an error of 340 degrees - making the kopter turn 340 degrees left. Instead, we
            // want to turn 20 degrees right. So, we need PidController::computeOutputFromError();
            mFlightControllerValues.controllerYaw.setDesiredValue(0.0f);
            const float outputYaw = mFlightControllerValues.controllerYaw.computeOutputFromError(angleToYawDegrees);

            qDebug() << "FlightController::slotComputeMotionCommands():" << getFlightState().toString() << mFlightControllerValues.lastKnownPose << "hoverPos:" << mFlightControllerValues.hoverPosition << "angleToYawDeg:" << angleToYawDegrees << "deg" << (angleToYawDegrees < 0.0f ? "right" : "left");

            // When approaching target, set the hoverPosition's height as desired value
            mFlightControllerValues.controllerThrust.setDesiredValue(mFlightControllerValues.hoverPosition.y());
            const float outputThrust = MotionCommand::thrustHover + mFlightControllerValues.controllerThrust.computeOutputFromValue(mFlightControllerValues.lastKnownPose.getPosition().y());

            // See how we can reach hoverPosition by pitching and rolling
            float lateralOffsetPitch, lateralOffsetRoll;
            getLateralOffsets(mFlightControllerValues.lastKnownPose, mFlightControllerValues.hoverPosition, lateralOffsetPitch, lateralOffsetRoll);

            // Pitch and roll are computed based on position-offsets. Of course, we want the offset to be 0...
            mFlightControllerValues.controllerPitch.setDesiredValue(0.0f);
            const float outputPitch = mFlightControllerValues.controllerPitch.computeOutputFromValue(qBound(-10.0f, -lateralOffsetPitch, 10.0f));

            mFlightControllerValues.controllerRoll.setDesiredValue(0.0f);
            const float outputRoll = mFlightControllerValues.controllerRoll.computeOutputFromValue(qBound(-10.0f, -lateralOffsetRoll, 10.0f));

            mFlightControllerValues.motionCommand = MotionCommand(outputThrust, outputYaw, outputPitch, outputRoll);
            qDebug() << "FlightController::slotComputeMotionCommands(): emitting motion:" << mFlightControllerValues.motionCommand;
            emit motion(&mFlightControllerValues.motionCommand);

            // See whether we've reached the waypoint
            if(mFlightControllerValues.flightState.state == FlightState::Value::ApproachWayPoint && mFlightControllerValues.lastKnownPose.getPosition().distanceToLine(mFlightControllerValues.trajectoryGoal, QVector3D()) < 0.80f) // close to wp
            {
                nextWayPointReached();
            }
        }
        else
        {
            qDebug() << "FlightController::slotComputeMotionCommands():" << getFlightState().toString() << ", pose precision" << mFlightControllerValues.lastKnownPose.precision << "- update is from" << mFlightControllerValues.lastKnownPose.timestamp << " - age in ms:" << mFlightControllerValues.lastKnownPose.getAge() << "- not overwriting safe hover values";
        }
    }
    else
    {
        qDebug() << "FlightController::slotComputeMotionCommands(): FLIGHTSTATE NOT DEFINED:" << mFlightControllerValues.flightState.toString();
        Q_ASSERT(false);
    }

    emit flightControllerValues(&mFlightControllerValues);
    logFlightControllerValues();
}

// pitch: negative: target is in front of vehicle, positive: target is behind vehicle
// roll:  negative: target is right of vehicle, positive: target is left of vehicle
void FlightController::getLateralOffsets(const Pose& vehiclePose, const QVector3D &desiredPosition, float& pitch, float& roll)
{
    QVector3D vectorVehicleToHoverPosition = vehiclePose.getPosition() - desiredPosition;
    const float angleToTurnTowardsDesiredPosition = Pose::getShortestTurnRadians(
                atan2(-vectorVehicleToHoverPosition.x(), -vectorVehicleToHoverPosition.z())
                - vehiclePose.getYawRadians()
                );

    vectorVehicleToHoverPosition.setY(0.0f);

    pitch = cos(angleToTurnTowardsDesiredPosition) * vectorVehicleToHoverPosition.length();
    roll = -sin(angleToTurnTowardsDesiredPosition) * vectorVehicleToHoverPosition.length();

    qDebug() << "FlightController::getLateralOffsets(): lateral offset pitch" << pitch << "roll" << roll;
}

/*void FlightController::smoothenControllerOutput(MotionCommand& mc)
{
    Q_ASSERT(false && "Why the hell was I casting to unsigned ints here? YPR can be negative!!?");
    mc.thrust = (quint8)(((quint16)mc.thrust + (quint16)mLastMotionCommandUsedForSmoothing.thrust) / 2.0f);
    mc.yaw = (quint8)(((quint16)mc.yaw + (quint16)mLastMotionCommandUsedForSmoothing.yaw) / 2.0f);
    mc.pitch = (quint8)(((quint16)mc.pitch + (quint16)mLastMotionCommandUsedForSmoothing.pitch) / 2.0f);
    mc.roll = (quint8)(((quint16)mc.roll + (quint16)mLastMotionCommandUsedForSmoothing.roll) / 2.0f);

    mLastMotionCommandUsedForSmoothing = mc;
}*/

void FlightController::logFlightControllerValues()
{
    mFlightControllerValues.timestamp = GnssTime::currentTow();

    QByteArray magic("FLTCLR");
    mLogFile->write(magic.constData(), magic.size());
    mLogFile->write((const char*)&mFlightControllerValues, sizeof(FlightControllerValues));
}

void FlightController::slotCalibrateImu()
{
    mImuOffsets.doCalibration();
}

void FlightController::nextWayPointReached()
{
    Q_ASSERT(mFlightControllerValues.flightState == FlightState::Value::ApproachWayPoint && "FlightController::nextWayPointReached(): reached waypoint, but am not in ApproachWayPoint state. Expect trouble!");

    mTrajectoryProgress = 0.0f;

    // We have reached the mWayPoints.first() position. Let this point be the start of the next route we travel.
    mFlightControllerValues.trajectoryStart = mWayPoints.first();

    // The next waypoint has been reached. Remove and append it to the list of passed points.
    mWayPointsPassed.append(mWayPoints.takeFirst());

    qDebug() << "FlightController::nextWayPointReached(): distance" << mFlightControllerValues.lastKnownPose.getPosition().distanceToLine(mWayPointsPassed.last(), QVector3D()) << " - reached waypoint" << mWayPointsPassed.last();

    // First emit this signal for the base to sync the list, THEN call
    // ensureSafeFlightAfterWaypointsChanged(), which might append more waypoints.
    emit wayPointReached(mWayPointsPassed.last());

    ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(&mWayPoints);
}

void FlightController::slotWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "FlightController::slotSetNextWayPoint(): state is" << mFlightControllerValues.flightState.toString() << "inserting waypoint" << wayPoint << "into index" << index;
    mWayPoints.insert(index, wayPoint);
    emit currentWayPoints(&mWayPoints);

    // Just in case we were idle before...
    if(mFlightControllerValues.flightState == FlightState::Value::Idle)
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
        if(mFlightControllerValues.flightState == FlightState::Value::ApproachWayPoint)
            ensureSafeFlightAfterWaypointsChanged();
    }
    qDebug() << "FlightController::slotWayPointDelete(): after deleting wpt, emitting new wpt list of size" << mWayPoints.size();
    emit currentWayPoints(&mWayPoints);
}

void FlightController::slotSetWayPoints(const QList<WayPoint>& wayPoints)
{
    mWayPoints = wayPoints;

    // Just in case we were idle before...
    if(mFlightControllerValues.flightState == FlightState::Value::Idle && mWayPoints.size())
    {
        qDebug() << "FlightController::slotSetWayPoints(): we were idle and got new waypoints, switching to ApproachWayPoint";
        setFlightState(FlightState::Value::ApproachWayPoint);
    }

    // The list might now be empty, so we might have to land.
    if(mFlightControllerValues.flightState == FlightState::Value::ApproachWayPoint)
        ensureSafeFlightAfterWaypointsChanged();

    emit currentWayPoints(&mWayPoints);
}

void FlightController::slotNewVehiclePose(const Pose* const pose)
{
    //Profiler p(__PRETTY_FUNCTION__);
    qDebug() << "FlightController::slotNewVehiclePose(): flightstate:" << mFlightControllerValues.flightState.toString() << ": new pose from" << pose->timestamp << "has age" << GnssTime::currentTow() - pose->timestamp;

    // Whatever precision and flightstate, save the pose. We also save unprecise poses here,
    // which might come in handy later.
    //
    // Of course, this means that we need to check this pose before we actually
    // use it for flight control!
    mFlightControllerValues.lastKnownPose = *pose;

    if(mImuOffsets.needsMoreMeasurements()) mImuOffsets.calibrate(pose);

    switch(mFlightControllerValues.flightState.state)
    {
    case FlightState::Value::UserControl:
    {
        // Keep ourselves disabled in UserControl.
        // why should we logFlightControllerValues(); ???
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

        // This method was called with a new pose. If that was precise enough, lets use it to compute motion.
        // If its not precise, let's not compute hover-values now, but wait for a precise pose until the backup
        // timer times out. Sequences of precise-backupmotion-precise can make the vehicle start swinging.
        if(pose->isSufficientlyPreciseForFlightControl())
        {
            slotComputeMotionCommands();

            // Re-set the safety timer, making it fire 50ms in the future - not when its scheduled, which might be sooner.
            // So that when no new pose comes in for too long, we'll compute a backup motion.
            mBackupTimerComputeMotion->start(backupTimerIntervalFast);
        }

        break;
    }

    case FlightState::Value::Hover:
    {
        Q_ASSERT(mBackupTimerComputeMotion->interval() == backupTimerIntervalFast && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Hover has inactive backup timer or wrong interval!");

        // This method was called with a new pose. If that was precise enough, lets use it to compute motion.
        // If its not precise, let's not compute hover-values now, but wait for a precise pose until the backup
        // timer times out. Sequences of precise-backupmotion-precise can make the vehicle start swinging.
        if(pose->isSufficientlyPreciseForFlightControl())
        {
            slotComputeMotionCommands();

            // Re-set the safety timer, making it fire 50ms in the future - not when its scheduled, which might be sooner.
            // So that when no new pose comes in for too long, we'll compute a backup motion.
            mBackupTimerComputeMotion->start(backupTimerIntervalFast);
        }

        break;
    }

    default:
        Q_ASSERT(false && "illegal flightstate in FlightController::slotNewVehiclePose()!");
    }

}

void FlightController::setFlightState(FlightState newFlightState)
{
    qDebug() << "FlightController::setFlightState():" << mFlightControllerValues.flightState.toString() << "=>" << newFlightState.toString();

    if(mFlightControllerValues.flightState == newFlightState)
    {
        qDebug() << "FlightController::setFlightState(): switching to same flightstate doesn't make much sense, returning.";
        return;
    }

    // Set controller weights according to new flightstate
    if(mFlightControllerWeights.contains(newFlightState.state))
    {
        QMutableMapIterator<PidController*,QMap<QChar /*weightName*/,float /*weight*/> > itFlightStateWeights(mFlightControllerWeights[newFlightState.state]);
        while(itFlightStateWeights.hasNext())
        {
            itFlightStateWeights.next();
            itFlightStateWeights.key()->setWeights(&itFlightStateWeights.value());
            qDebug() << "FlightController::setFlightState(): setting weights to" << itFlightStateWeights.value();
        }
    }
    else
    {
        qDebug() << "FlightController::setFlightState(): no weights for state" << newFlightState.toString() << ": setting to 0.";
        mFlightControllerValues.controllerThrust.setWeights(0.0f, 0.0f, 0.0f);
        mFlightControllerValues.controllerYaw.setWeights(0.0f, 0.0f, 0.0f);
        mFlightControllerValues.controllerPitch.setWeights(0.0f, 0.0f, 0.0f);
        mFlightControllerValues.controllerRoll.setWeights(0.0f, 0.0f, 0.0f);
    }


    // When switching from ApproachWaypoint or Hover to something manual, we want these members to be cleared for easier debugging.
    mFlightControllerValues.controllerThrust.reset();
    mFlightControllerValues.controllerYaw.reset();
    mFlightControllerValues.controllerPitch.reset();
    mFlightControllerValues.controllerRoll.reset();

    // Reset positions
    mFlightControllerValues.hoverPosition = mFlightControllerValues.trajectoryStart = mFlightControllerValues.trajectoryGoal = QVector3D();

    mFlightControllerValues.motionCommand = MotionCommand();

    switch(newFlightState.state)
    {
    case FlightState::Value::ApproachWayPoint:
    {

        // Set the new flightstate, even if we possibly revert that next.
        mFlightControllerValues.flightState = newFlightState;

        if(mWayPoints.size() == 0)
        {
            qDebug() << "FlightController::setFlightState(): ApproachWayPoint: no waypoints to approach - setting hover mode!";

            // It is important that flightstate was already set to ApproachWaypoint above. If it was
            // still left in e.g. Hover, then calling setFlightState(Hover) could cause problems.
            setFlightState(FlightState::Value::Hover);
            return;
        }

        mBackupTimerComputeMotion->start(backupTimerIntervalFast);

        mFlightControllerValues.hoverPosition = mFlightControllerValues.lastKnownPose.getPosition();
        mFlightControllerValues.trajectoryStart = mFlightControllerValues.lastKnownPose.getPosition();
        mFlightControllerValues.trajectoryGoal = mWayPoints.first();
        qDebug() << "FlightController::setFlightState(): ApproachWayPoint - initializing controllers, setting backup motion timer to high-freq.";
        qDebug() << "FlightController::setFlightState(): ApproachWayPoint - setting hoverPosition and trajectoryStart to current position" << mFlightControllerValues.trajectoryStart << "and goal to wp" << mFlightControllerValues.trajectoryGoal;

        // We're going to use the controllers, so make sure to initialize them.
        initializeControllers();

        break;
    }

    case FlightState::Value::Hover:
    {
        mFlightControllerValues.hoverPosition = mFlightControllerValues.lastKnownPose.getPosition();

        if(mFlightControllerValues.lastKnownPose.getAge() > 100)
            qDebug() << "FlightController::setFlightState(): WARNING: going to hover, but pose being set as hover-target is" << mFlightControllerValues.lastKnownPose.getAge() << "ms old!";

        qDebug() << "FlightController::setFlightState(): going to hover, setting backup motion timer to high-freq and staying at" << mFlightControllerValues.hoverPosition;

        mBackupTimerComputeMotion->start(backupTimerIntervalFast);
        mFlightControllerValues.flightState = newFlightState;

        // We're going to use the controllers, so make sure to initialize them.
        initializeControllers();
        break;
    }

    case FlightState::Value::UserControl:
    {
        // We don't need the timer, as the remote control will overrule our output anyway.
        qDebug() << "FlightController::setFlightState(): disabling backup motion timer.";
        mBackupTimerComputeMotion->stop();
        mFlightControllerValues.flightState = newFlightState;
        break;
    }

    case FlightState::Value::Idle:
    {
        qDebug() << "FlightController::setFlightState(): setting backup motion timer to low-freq";
        mBackupTimerComputeMotion->start(backupTimerIntervalSlow);
        mFlightControllerValues.flightState = newFlightState;
        mFlightControllerValues.hoverPosition = QVector3D();
        mFlightControllerValues.trajectoryStart = QVector3D();
        mFlightControllerValues.trajectoryGoal = QVector3D();
        break;
    }

    default:
        Q_ASSERT(false && "FlightController::setFlightState(): undefined flightstate");
    }

    emit flightStateChanged(&mFlightControllerValues.flightState);
    emit flightControllerWeightsChanged();
}

void FlightController::initializeControllers()
{
    qDebug() << "FlightController::initializeControllers(): initializing controllers...";

    // Is there a good reason for resettign weights? Only if hover and ApproachWayPoint need
    // different weights?!
    mFlightControllerValues.controllerThrust.reset();
    mFlightControllerValues.controllerYaw.reset();
    mFlightControllerValues.controllerPitch.reset();
    mFlightControllerValues.controllerRoll.reset();

    // If we change weights above, tell basestation about the changes.
//    emit flightControllerValues(&mFlightControllerValues);
//    emit flightControllerWeightsChanged();

    // We want to approach a target using roll only after the vehicle points at it.
//    mApproachPhase = ApproachPhase::OrientTowardsTarget;

    mTrajectoryProgress = 0.0f;
}

void FlightController::slotSetHeightOverGround(const float& beamLength)
{
    // Height is given from vehicle center to ground, but we care about the bottom of the landing gear.
    qDebug() << "FlightController::slotSetHeightOverGround()" << beamLength - 0.16f << "meters";
    mFlightControllerValues.lastKnownHeightOverGroundTimestamp = GnssTime::currentTow();
    mFlightControllerValues.lastKnownHeightOverGround = beamLength - 0.16f;
}
/*
bool FlightController::isHeightOverGroundValueRecent() const
{
    qDebug() << "FlightController::isHeightOverGroundValueRecent(): time of last heightOverGround measurement is" << mFlightControllerValues.lastKnownHeightOverGroundTimestamp;
    return (GnssTime::currentTow() - mFlightControllerValues.lastKnownHeightOverGroundTimestamp) < 500;
}*/

// To be called after waypoints have changed to check for dangerous states:
// If wpt list is empty and state is ApproachWayPoint, will either
//  - idle if low
//  - get a landing waypoint if heightOverGround is known and up-to-date
//  - descend slowly if heightOverGround is unknown
void FlightController::ensureSafeFlightAfterWaypointsChanged()
{

    Q_ASSERT(mFlightControllerValues.flightState == FlightState::Value::ApproachWayPoint && "FlightController::ensureSafeFlightAfterWaypointsChanged(): flightstate is NOT ApproachWayPoint!");

    if(mWayPoints.size() == 0)
    {
        qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, going to hover";
        setFlightState(FlightState::Value::Hover);

        /* Landing and shutdown - still too untested
        // The method has debug output, so call it just once for now.
        const bool heightOverGroundValueRecent = isHeightOverGroundValueRecent();

        // The list is now empty and we are still flying.
        if(heightOverGroundValueRecent && mFlightControllerValues.lastKnownHeightOverGround < 0.3f)
        {
            // We're low anyway, just got to idle mode.
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, we're low with valid heightOverGround of" << mFlightControllerValues.lastKnownHeightOverGround << ", idling";
            setFlightState(Idle);
        }
        else if(heightOverGroundValueRecent)
        {
            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround is known to be" << mFlightControllerValues.lastKnownHeightOverGround << "inserting landing wpt 0.2m above ground";
            mWayPoints.append(WayPoint(mFlightControllerValues.lastKnownPose.getPosition() - QVector3D(0.0, mFlightControllerValues.lastKnownHeightOverGround - 0.2, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
        else
        {
            // TODO: We might instead switch to Hover instead of trying to land here?!

            // Descend with 10cm per second, so thats 0.1m/PoseFrequency, assuming that we'll
            // hit the new waypoint on every controller computation. This is likely, as hitting
            // a waypoints means being closer than 50cm to it.

            qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, heightOverGround unknown, WARNING, next wpt is 0.005m below us.";
            mWayPoints.append(WayPoint(mFlightControllerValues.lastKnownPose.getPosition() - QVector3D(0.0, 0.005, 0.0)));
            emit wayPointInserted(mWayPoints.size()-1, mWayPoints.last());
        }
        */
    }
}

void FlightController::slotFlightStateSwitchValueChanged(const FlightStateSwitch* const fssv)
{
    // This method does nothing more than some sanity checks and verbose flightstae switching.

    qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): flightstate" << mFlightControllerValues.flightState.toString() << "FlightStateSwitch changed to:" << fssv->toString();

    switch(mFlightControllerValues.flightState.state)
    {
    case FlightState::Value::UserControl:
        switch(fssv->value)
        {
        case FlightStateSwitch::Value::UserControl:
            // This is an illegal state: We are already in UserControl, and now we're
            // told that the user has disabled computerControl. Thats impossible.
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in UserControl and now user switched to UserControl. Error.");
            break;
        case FlightStateSwitch::Value::Hover:
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::Value::ApproachWayPoint:
            // We are in UserControl, but switching to ComputerControl.
            setFlightState(FlightState::Value::ApproachWayPoint);
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    case FlightState::Value::ApproachWayPoint:
        switch(fssv->value)
        {
        case FlightStateSwitch::Value::UserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Value::Hover:
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::Value::ApproachWayPoint:
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
        switch(fssv->value)
        {
        case FlightStateSwitch::Value::UserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Value::Hover:
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in Hover and now user switched to Hover. Error.");
            break;
        case FlightStateSwitch::Value::ApproachWayPoint:
            setFlightState(FlightState::Value::ApproachWayPoint);
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined FlightStateSwitchValue!");
            break;
        }
        break;

    case FlightState::Value::Idle:
        switch(fssv->value)
        {
        case FlightStateSwitch::Value::UserControl:
            // We are idling, but the user wants to take over control. Ok.
            setFlightState(FlightState::Value::UserControl);
            break;
        case FlightStateSwitch::Value::Hover:
            // We are idling, but the user wants us to hover. Hovering on the ground is not healthy!
            qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): going from idle to hover, this doesn't seem like a good idea, as the current height will be kept.";
            setFlightState(FlightState::Value::Hover);
            break;
        case FlightStateSwitch::Value::ApproachWayPoint:
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

    qDebug() << "FlightController::slotFlightStateSwitchValueChanged(): done, new flightstate" << mFlightControllerValues.flightState.toString();
}

void FlightController::slotEmitFlightControllerInfo()
{
    // Usually called from BaseConnection::newConnection(), tell base about us...
    qDebug() << "FlightController::slotEmitFlightControllerInfo(): emitting flightcontrollervalues, flightstate, controllerweights and waypoints.";
    emit flightControllerValues(&mFlightControllerValues);
    emit flightControllerWeightsChanged();
    emit flightStateChanged(&mFlightControllerValues.flightState);
    emit currentWayPoints(&mWayPoints);
}

void FlightController::slotSetControllerWeights(const QString* const controllerName, const QMap<QChar,float>* const weights)
{
    qDebug() << "FlightController::slotSetControllerWeights(): setting new weights for flightstate" << mFlightControllerValues.flightState.toString() << "and controller" << *controllerName << ":" << *weights;

//    QMap<PidController*, QMap<QChar,float> > controllerWeights;

    FlightState::Value assignedFlightState;

    // To which flightstate do we want to assing the new weights? By default, we assign to
    // the current flightstate - if we're in usercontrol or idle, assign to ApprochWayPoint
    if(mFlightControllerValues.flightState.state == FlightState::Value::UserControl || mFlightControllerValues.flightState.state == FlightState::Value::Idle)
        assignedFlightState = FlightState::Value::ApproachWayPoint;
    else
        assignedFlightState = mFlightControllerValues.flightState.state;

    if(controllerName->toLower() == "thrust")
    {
        mFlightControllerValues.controllerThrust.setWeights(weights);
        mFlightControllerWeights[assignedFlightState][&mFlightControllerValues.controllerThrust] = *weights;
    }
    else if(controllerName->toLower() == "yaw")
    {
        mFlightControllerValues.controllerYaw.setWeights(weights);
        mFlightControllerWeights[assignedFlightState][&mFlightControllerValues.controllerYaw] = *weights;
    }
    else if(controllerName->toLower() == "pitch")
    {
        mFlightControllerValues.controllerPitch.setWeights(weights);
        mFlightControllerWeights[assignedFlightState][&mFlightControllerValues.controllerPitch] = *weights;
    }
    else if(controllerName->toLower() == "roll")
    {
        mFlightControllerValues.controllerRoll.setWeights(weights);
        mFlightControllerWeights[assignedFlightState][&mFlightControllerValues.controllerRoll] = *weights;
    }

    qDebug() << "FlightController::slotSetControllerWeights(): weights changed, emitting new flightcontrollervalues and then the changed signal.";
    emit flightControllerValues(&mFlightControllerValues);
    emit flightControllerWeightsChanged();
}

// Get the position of the carrot (=hoverPosition) that the mule shall follow...
QVector3D FlightController::getHoverPosition(const QVector3D& trajectoryStart, const QVector3D& trajectoryGoal, const QVector3D& vehiclePosition, const float& desiredDistanceToHoverPosition)
{
    const QVector3D trajectoryUnitVector = (trajectoryGoal-trajectoryStart).normalized();
    const QVector3D vectorFromTrajectoryStartToVehicle = vehiclePosition - trajectoryStart;
    const float trajectoryLength = trajectoryStart.distanceToLine(trajectoryGoal, QVector3D());

    // If we project the vehicle's position on the trajectory, how far along are we? We bound to between 0 and trajectory-length.
    const float projectedTrajectoryProgress = qBound(0.0f, (float)QVector3D::dotProduct(trajectoryUnitVector, vectorFromTrajectoryStartToVehicle), trajectoryLength);

//    mTrajectoryProgress = std::max(projectedTrajectoryProgress, mTrajectoryProgress);
    const QVector3D projectedHoverPosition(trajectoryStart + (trajectoryUnitVector * projectedTrajectoryProgress));

    const float distanceFromVehicleToProjectedHoverPosition = vehiclePosition.distanceToLine(projectedHoverPosition, QVector3D());

    mTrajectoryProgress = std::max(mTrajectoryProgress, projectedTrajectoryProgress);

    if(distanceFromVehicleToProjectedHoverPosition < desiredDistanceToHoverPosition)
    {
        // The vehicle is so close that we should advance the carrot, to keep it desiredDistanceToHoverPosition in front
        const float advanceHoverPosition = sqrt(pow(desiredDistanceToHoverPosition, 2.0f) - pow(distanceFromVehicleToProjectedHoverPosition, 2.0f));
        const float newTrajectoryProgress = std::min(projectedTrajectoryProgress + advanceHoverPosition, trajectoryLength);
        mTrajectoryProgress = std::max(mTrajectoryProgress, newTrajectoryProgress);
    }

    qDebug() << "FlightController::getHoverPosition(): TS" << trajectoryStart << "VP" << vehiclePosition << "TUV" << trajectoryUnitVector << "TL" << trajectoryLength << "VFTSTV" << vectorFromTrajectoryStartToVehicle << "PTP" << projectedTrajectoryProgress << "MTP" << mTrajectoryProgress;

    return QVector3D(trajectoryStart + (trajectoryUnitVector * mTrajectoryProgress));
}
