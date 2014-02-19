#include "flightcontroller.h"
#include "motioncommand.h"
#include <profiler.h>

FlightController::FlightController(const QString& logFilePrefix) : QObject()
{
    mLogFile = new LogFile(logFilePrefix + QString(".flt"), LogFile::Encoding::Binary);
    qDebug()<< "Size of FlightControllerValues:" << sizeof(FlightControllerValues);

    mBackupTimerComputeMotion = new QTimer(this);
    connect(mBackupTimerComputeMotion, SIGNAL(timeout()), SLOT(slotComputeBackupMotion()));

    // Initialize default/template controllers
    QMap<QChar, float> weights;
    QMap<PidController*, QMap<QChar, float> > controllerWeights;

    // 2013-08-20: thrust 40/0/20   yaw 1/0/0   pitchRoll 10/0/6 seem nice.
    controllerWeights.clear();
    // Hover - Thrust
    weights.clear();
    weights.insert('p', 40.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 20.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerThrust, weights);

    // Hover - Yaw
    weights.clear();
    weights.insert('p', 1.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 0.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerYaw, weights);

    // Hover - Pitch / Roll
    // 2014-02-19: Decrease from 10/0/6 to 9/0/4, as the kopter is kinda epileptic today - d is the cause for this.
    weights.clear();
    weights.insert('p', 9.0f);
    weights.insert('i', 0.0f);
    weights.insert('d', 4.0f);
    controllerWeights.insert(&mFlightControllerValues.controllerPitch, weights);
    controllerWeights.insert(&mFlightControllerValues.controllerRoll, weights);
    mFlightControllerWeights.insert(FlightState::State::Hover, controllerWeights);
    mFlightControllerWeights.insert(FlightState::State::ApproachWayPoint, controllerWeights);

    // If we don't use a laserscanner (for whatever reason, during testing),
    // set mFlightControllerValues.lastKnownHeightOverGround to 1.0 so it doesn't prevent pitch/roll
    mFlightControllerValues.lastKnownHeightOverGround = 1.0f;

    mMaxFlightVelPerAxis = 1.0f;

    setFlightState(FlightState(FlightState::State::UserControl));
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
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::State::UserControl && "FlightController::slotComputeBackupMotion(): i was called in FlightState UserControl");
    slotComputeMotionCommands();
}

// WARNING: We compute safe motion commands when the pose is not precise or old. If it turns out such poses are unavoidable at times,
// it might be better not to generate motion at all, because a single safe-motion between otherwise valid motion-commands can cause
// the helicopter to start bouncing in the air. We need to test this!
void FlightController::slotComputeMotionCommands()
{
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::State::UserControl && "FlightController::slotComputeMotionCommands(): i was called in FlightState UserControl");
    Q_ASSERT(mFlightControllerValues.flightState != FlightState::State::Undefined && "FlightController::slotComputeMotionCommands(): i was called in FlightState Undefined");

    if(mFlightControllerValues.flightState.state == FlightState::State::Idle)
    {
        qDebug() << "FlightController::slotComputeMotionCommands(): FlightState: Idle, emitting idle-thrust.";
        mFlightControllerValues.motionCommand = MotionCommand((quint8)90, 0.0f, 0.0f, 0.0f);
    }
    else if(mFlightControllerValues.flightState.state == FlightState::State::ApproachWayPoint || mFlightControllerValues.flightState.state == FlightState::State::Hover)
    {
        // Can be overridden with better non-default values below
        mFlightControllerValues.motionCommand = MotionCommand(MotionCommand::thrustHover, 0.0f, 0.0f, 0.0f);

        // Only use poses that are sufficiently precise. Else, keep safe hover values.
        if(mFlightControllerValues.lastKnownPose.isSufficientlyPreciseForFlightControl())
        {
            // When hovering, we want to look away from the origin. When approaching a waypoint, we want to look at the hoverPosition.
            // This variable shall hold the desired yaw angle.
            float angleToYawDegreesNow = 0.0f;
            float angleToYawDegreesAfterNextWaypoint = 0.0f;

            // See how we can reach hoverPosition by pitching and rolling

            if(mFlightControllerValues.flightState.state == FlightState::State::ApproachWayPoint)
            {
                // positionStart has been set already, lets define positionGoal. As that may change in-flight,
                // we re-set it here in every iteration.
                mFlightControllerValues.trajectoryGoal = mWayPoints.first();

                // Yaw: We want the vehicle to look NOT towards positionGoal, but always look in the same direction as the line from start to goal - even if it has drifted off that line!
                // Corrections will be done by rolling
                const QVector3D trajectory = mFlightControllerValues.trajectoryGoal - mFlightControllerValues.trajectoryStart;
                angleToYawDegreesNow = RAD2DEG(
                            Pose::getShortestTurnRadians(
                                -Pose::getShortestTurnRadians(- atan2(-trajectory.x(), -trajectory.z()))
                                - mFlightControllerValues.lastKnownPose.getYawRadians()
                                )
                            );

                // If there is a next waypoint after the current one, we want to yaw towards it shortly before reaching the current one.
                if(mWayPoints.size() > 1)
                {
                    const QVector3D trajectory = mWayPoints.at(1) - mFlightControllerValues.trajectoryGoal;
                    angleToYawDegreesAfterNextWaypoint = RAD2DEG(
                                Pose::getShortestTurnRadians(
                                    -Pose::getShortestTurnRadians(- atan2(-trajectory.x(), -trajectory.z()))
                                    - mFlightControllerValues.lastKnownPose.getYawRadians()
                                    )
                                );
                }

                // Move the hoverPosition towards trajectoryGoal
                mFlightControllerValues.hoverPosition = getHoverPosition(
                            mFlightControllerValues.trajectoryStart,
                            mFlightControllerValues.trajectoryGoal,
                            mFlightControllerValues.lastKnownPose.getPosition());
            }
            else if(mFlightControllerValues.flightState.state == FlightState::State::Hover)
            {
                // We want to point exactly away from the origin, because thats probably where the user is
                // standing - steering is easier if the vehicle's forward arm points away from the user.
                // Thats fine as long as there is a sufficient distance between vehicle and origin during
                // hover. If the vehicle hovers around the origin, it starts turning quickly, which sucks.
                // To prevent this, we don't consider the vehicle's current position, but rather the chosen
                // hover position.
                const QVector3D vectorOriginToHoverPosition = mFlightControllerValues.hoverPosition;

                angleToYawDegreesNow = RAD2DEG(
                            Pose::getShortestTurnRadians(
                                atan2(vectorOriginToHoverPosition.x(), vectorOriginToHoverPosition.z())
                                - DEG2RAD(180.0f)
                                - mFlightControllerValues.lastKnownPose.getYawRadians()
                                )
                            );
            }

            const float distanceToGoal = mFlightControllerValues.lastKnownPose.getPosition().distanceToLine(mFlightControllerValues.trajectoryGoal, QVector3D());

            // If we're close to the upcoming waypoint, already start orienting towards the next!
            if(angleToYawDegreesAfterNextWaypoint != 0.0f)
            {
                // At 1.0m distance, rotate to current trajectory angle. At 0.0m, rotate to next trajectory angle. In between, fade!
                angleToYawDegreesNow = blend(1.0, 0.0, distanceToGoal, angleToYawDegreesNow, angleToYawDegreesAfterNextWaypoint);
            }

            // angleToYawDegrees needs some tuning: When using a p(d) controller and a range of 0 to (-)180, the vehicle
            // would yaw quite quickly, making the laserscanner give sparse results while turning. So, we do want to
            // turn as fast as the laserscanner allows until shortly before we're oriented correctly. On the other hand,
            // we don't really want to be pointing too far off, so we want to yaw with more than "1" if we're 1 degree off.
            // As it turns out (by asking Manfred :), both arctan and (X/1+abs(x)) are exactly what we need. The latter
            // seems cheaper to compute, so we use that one.
            const float aggressiveness = 0.2f; // how quickly to reach MotionCommand::yawMax when abs(angleToYawDegrees) rises
            const float angleToYawDegreesAdjusted = MotionCommand::yawMax * ((aggressiveness * angleToYawDegreesNow) / (1.0f + fabs(aggressiveness * angleToYawDegreesNow)));

            // If we give the yaw controller our current yaw (e.g. -170 deg) and our desired value (e.g. +170),
            // it would compute an error of 340 degrees - making the kopter turn 340 degrees left. Instead, we
            // want to turn 20 degrees right. So, we need PidController::computeOutputFromError();
            mFlightControllerValues.controllerYaw.setDesiredValue(0.0f);
            const float outputYaw = mFlightControllerValues.controllerYaw.computeOutputFromError(angleToYawDegreesAdjusted);

            // When approaching target, set the hoverPosition's height as desired value
            mFlightControllerValues.controllerThrust.setDesiredValue(mFlightControllerValues.hoverPosition.y());
            const float outputThrust = MotionCommand::thrustHover + mFlightControllerValues.controllerThrust.computeOutputFromValue(mFlightControllerValues.lastKnownPose.getPosition().y());

            // We want to control based on speed, split up on pitch and roll axes. If we're e.g. X pitch-meter away from the target, we'd like to approach it with Xm/s.
            // How fast the vehicle moves in m/s in the world frame
            QVector3D worldFrameSpeedValue(mFlightControllerValues.lastKnownPose.getVelocity());
            worldFrameSpeedValue.setY(0.0f);

            QVector3D pitchTarget = mFlightControllerValues.flightState.state == FlightState::State::ApproachWayPoint ? mFlightControllerValues.trajectoryGoal : mFlightControllerValues.hoverPosition;

            float lateralOffsetPitchToTrajectoryGoal = getLateralOffsetOnVehiclePitchAxisToPosition(
                        mFlightControllerValues.lastKnownPose.getPosition(),
                        mFlightControllerValues.lastKnownPose.getYawRadians(),
                        pitchTarget);

            // We want the offset to the trajectory over the roll axis
            float lateralOffsetRollToTrajectory = getLateralOffsetOnVehicleRollAxisToPosition(
                        mFlightControllerValues.lastKnownPose.getPosition(),
                        mFlightControllerValues.lastKnownPose.getYawRadians(),
                        mFlightControllerValues.hoverPosition);

            const float angleFromVehicleFrontToSpeedVector = Pose::getShortestTurnRadians(
                        atan2(-worldFrameSpeedValue.x(), -worldFrameSpeedValue.z())
                        - mFlightControllerValues.lastKnownPose.getYawRadians()
                        );

            const float velOverPitchValue = -cos(angleFromVehicleFrontToSpeedVector) * worldFrameSpeedValue.length();
            const float velOverRollValue  = -sin(angleFromVehicleFrontToSpeedVector) * worldFrameSpeedValue.length();

            // Slow down when orientation is wrong. In general, only slow down when really close to wp (the 3.0 factor)!
            float velOverPitchDesired = (3.0f * lateralOffsetPitchToTrajectoryGoal) / qBound(1.0, sqrt(fabs(angleToYawDegreesNow)), 15.0);
            velOverPitchDesired = qBound(-mMaxFlightVelPerAxis, velOverPitchDesired, mMaxFlightVelPerAxis);

            // Do not bound desired speed over roll, we want to correct quickly.
            const float velOverRollDesired  = /*qBound(-mMaxFlightVelPerAxis,*/ 2.0f * lateralOffsetRollToTrajectory/*, mMaxFlightVelPerAxis)*/;

            // speedPitch denotes the vehicle's translation speed forward/backward. This speed goes along its Z-axis,
            // which points backwards. A positive speed thus means moving backwards. Positive pitch-control values
            // also mean bending backwards, so this matches.
            mFlightControllerValues.controllerPitch.setDesiredValue(velOverPitchDesired);
            const float outputPitch = mFlightControllerValues.controllerPitch.computeOutputFromValue(velOverPitchValue);

            // speedRoll denotes the vehicle's translation speed left/right. This speed goes along its X-axis,
            // which points right. A positive speed thus means moving right. Positive roll-control values
            // mean bending leftwards, so outputRoll needs to be inverted.
            mFlightControllerValues.controllerRoll.setDesiredValue(-velOverRollDesired);
            const float outputRoll = mFlightControllerValues.controllerRoll.computeOutputFromValue(-velOverRollValue);

            mFlightControllerValues.motionCommand = MotionCommand(outputThrust, outputYaw, outputPitch, outputRoll);

            // See whether we've reached the waypoint. Here, we check that not only the vehicle, but also the hoverposition is close to the waypoint.
            // If the hoverpos is 40cm in front of the goal and we switched to the next waypoint, the hoverposition would jump. This is not
            // good for a D-controller.
            if(
                    mFlightControllerValues.flightState.state == FlightState::State::ApproachWayPoint
                    //&& mFlightControllerValues.hoverPosition.distanceToLine(mFlightControllerValues.trajectoryGoal, QVector3D()) < mDistanceWayPointReached // hoverpos close to wp
                    && mFlightControllerValues.lastKnownPose.getPosition().distanceToLine(mFlightControllerValues.trajectoryGoal, QVector3D()) < mDistanceWayPointReached) // vehicle close to wp
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

    //mFlightControllerValues.motionCommand.adjustThrustToPitchAndRoll();

    qDebug() << "FlightController::slotComputeMotionCommands(): emitting motion:" << mFlightControllerValues.motionCommand << "and fsr" << mFlightControllerValues.flightStateRestriction.toString();
    emit motion(&mFlightControllerValues.motionCommand);

    mFlightControllerValues.timestamp = GnssTime::currentTow();
    emit flightControllerValues(&mFlightControllerValues);
    logFlightControllerValues();
}

// blend between valueToBlend1 and valueToBlend2, depending on the position of value between v1 and v2
// -1, 1, 0, 10, 20:    15
// 10, 20, 9, 0, 1:     0
// 20, 10, 9, 0, 1:     1
float FlightController::blend(const float v1, const float v2, const float value, const float valueToBlend1, const float valueToBlend2)
{
    float diff = v2 - v1;
    float advance = value - v1;

    if(fabs(diff) < 0.00001) diff = copysign(0.00001, diff);
    float factor = advance / diff;

    float result = valueToBlend1  + (valueToBlend2 - valueToBlend1) * factor;

    if(valueToBlend1 > valueToBlend2)
        return qBound(valueToBlend2, result, valueToBlend1);
    else
        return qBound(valueToBlend1, result, valueToBlend2);
}

// Positive values mean that the target is along the direction of the vehicle coordinate system's respective axes.
// pitch: negative: target is in front of vehicle, positive: target is behind vehicle
// roll:  negative: target is left of vehicle, positive: target is right of vehicle
float FlightController::getLateralOffsetOnVehiclePitchAxisToPosition(const QVector3D& vehiclePosition, const float vehicleYaw, const QVector3D &desiredPosition) const
{
    QVector3D vectorVehicleToHoverPosition = vehiclePosition - desiredPosition;
    const float angleToTurnTowardsDesiredPosition = Pose::getShortestTurnRadians(
                atan2(-vectorVehicleToHoverPosition.x(), -vectorVehicleToHoverPosition.z())
                - vehicleYaw
                );

    vectorVehicleToHoverPosition.setY(0.0f);

    return cos(angleToTurnTowardsDesiredPosition) * vectorVehicleToHoverPosition.length();
}

float FlightController::getLateralOffsetOnVehicleRollAxisToPosition(const QVector3D& vehiclePosition, const float vehicleYaw, const QVector3D &desiredPosition) const
{
    QVector3D vectorVehicleToHoverPosition = vehiclePosition - desiredPosition;
    const float angleToTurnTowardsDesiredPosition = Pose::getShortestTurnRadians(
                atan2(-vectorVehicleToHoverPosition.x(), -vectorVehicleToHoverPosition.z())
                - vehicleYaw
                );

    vectorVehicleToHoverPosition.setY(0.0f);

    return sin(angleToTurnTowardsDesiredPosition) * vectorVehicleToHoverPosition.length();
}

void FlightController::logFlightControllerValues()
{
    //qDebug() << "FlightController::logFlightControllerValues(): logging" << mFlightControllerValues.lastKnownPose << "at fcvtime" << mFlightControllerValues.timestamp;

    QByteArray data("FLTCLR"); // start with the magic bytes
    QDataStream ds(&data, QIODevice::WriteOnly);
    ds.skipRawData(data.size()); // position the stream after the magic bytes
    ds << mFlightControllerValues;
    mLogFile->write(data.constData(), data.size());
}

void FlightController::slotLiftHoverPosition()
{
    //mImuOffsets.doCalibration();
    // raise the hoverpos by 1.5m to start the flight.
    if(mFlightControllerValues.flightState == FlightState::State::Hover)
    {
        qDebug() << "FlightController::slotLiftHoverPosition(): hovering, raising hoverpos and trajectorygoal by 1.5m - please step back!";
        mFlightControllerValues.hoverPosition  += QVector3D(0.0f, 1.5f, 0.0f);
        mFlightControllerValues.trajectoryGoal += QVector3D(0.0f, 1.5f, 0.0f);
    }
}

void FlightController::nextWayPointReached()
{
    Q_ASSERT(mFlightControllerValues.flightState == FlightState::State::ApproachWayPoint && "FlightController::nextWayPointReached(): reached waypoint, but am not in ApproachWayPoint state. Expect trouble!");

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

    emit wayPoints(&mWayPoints, WayPointListSource::WayPointListSourceRover);
}

void FlightController::slotSetWayPoints(const QList<WayPoint>& wayPoints, const WayPointListSource source)
{
    if(source != WayPointListSource::WayPointListSourceRover)
    {
        qDebug() << "FlightController::slotSetWayPoints(): accepting" << wayPoints.size() << "waypoints...";

        mWayPoints = wayPoints;

        if(mWayPoints.size())
        {
            // New waypoints arrived!

            if((mFlightControllerValues.flightState == FlightState::State::Idle || mFlightControllerValues.flightState == FlightState::State::Hover) && mFlightControllerValues.flightStateRestriction.allowsFlightState(FlightState::State::ApproachWayPoint))
            {
                qDebug() << "FlightController::slotSetWayPoints(): we were idle or hovering, got new waypoints and FlightStateRestriction" << mFlightControllerValues.flightStateRestriction.toString() << "allows it, switching to ApproachWayPoint";
                setFlightState(FlightState::State::ApproachWayPoint);
            }
        }
        else
        {
            // no waypoints present. they were possibly deleted.
            if(mFlightControllerValues.flightState == FlightState::State::ApproachWayPoint)
            {
                qDebug() << "FlightController::slotSetWayPoints(): we are in ApproachWayPoint, we need to ensureSafeFlightAfterWaypointsChanged()...";
                ensureSafeFlightAfterWaypointsChanged();
            }
        }
    }
    else
        qDebug() << "FlightController::slotSetWayPoints(): ignoring" << wayPoints.size() << "waypoints, as source is Rover!";
}

void FlightController::slotNewVehiclePose(const Pose* const pose)
{
    //Profiler p(__PRETTY_FUNCTION__);
    //qDebug() << "FlightController::slotNewVehiclePose(): flightstate:" << mFlightControllerValues.flightState.toString() << pose->toString(true) << "age" <<pose->getAge();

    // Whatever precision and flightstate, save the pose. We also save unprecise poses here,
    // which might come in handy later.
    //
    // Of course, this means that we need to check this pose before we actually
    // use it for flight control!
    mFlightControllerValues.lastKnownPose = *pose;

    //if(mImuOffsets.needsMoreMeasurements()) mImuOffsets.calibrate(pose);

    switch(mFlightControllerValues.flightState.state)
    {
    case FlightState::State::UserControl:
    {
        // Keep ourselves disabled in UserControl.
        // why should we logFlightControllerValues(); ???
        Q_ASSERT(!mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): UserControl has active backup timer!");
        break;
    }

    case FlightState::State::Idle:
    {
        // Lets compute motion (although its really simple in this case)
        slotComputeMotionCommands();

        Q_ASSERT(mBackupTimerComputeMotion->interval() == backupTimerIntervalSlow && mBackupTimerComputeMotion->isActive() && "FlightController::slotNewVehiclePose(): Idle has inactive backup timer or wrong interval!");
        break;
    }

    case FlightState::State::ApproachWayPoint:
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

    case FlightState::State::Hover:
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

    if(!mFlightControllerValues.flightStateRestriction.allowsFlightState(newFlightState.state))
    {
        qDebug() << "FlightController::setFlightState(): restriction" << mFlightControllerValues.flightStateRestriction.toString() << "does not allow flightstate" << newFlightState.toString();
        Q_ASSERT(false);
    }

    if(mFlightControllerValues.flightState == newFlightState)
    {
        qDebug() << "FlightController::setFlightState(): switching to same flightstate doesn't make much sense, returning.";
        return;
    }

    // Set controller weights according to new flightstate
    if(mFlightControllerWeights.contains(newFlightState.state))
    {
        QMutableMapIterator<PidController*,QMap<QChar,float> > itFlightStateWeights(mFlightControllerWeights[newFlightState.state]);
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
    case FlightState::State::ApproachWayPoint:
    {
        // Set the new flightstate, even if we possibly revert that next.
        mFlightControllerValues.flightState = newFlightState;

        if(mWayPoints.size() == 0)
        {
            qDebug() << "FlightController::setFlightState(): ApproachWayPoint: no waypoints to approach - setting hover mode!";

            // It is important that flightstate was already set to ApproachWaypoint above. If it was
            // still left in e.g. Hover, then calling setFlightState(Hover) could cause problems.
            setFlightState(FlightState::State::Hover);
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

    case FlightState::State::Hover:
    {
        // When going to hover, we want the vehicle to stay almost at its current position, but just a little below.
        // This is to ease the start-procedure: When in UserControl on the ground, we switch to hover, setting the
        // hoverpos a little below ourselves, so the rotors keep quiet. We can then raise the hoverpos by a meter to
        // make it start up. If the hoverpos was at the current pos, the thrust would change quickly due to noise in
        // the GNSS pose's positions.
        const QVector3D offset(0.0f, 0.2f, 0.0f);

        mFlightControllerValues.hoverPosition = mFlightControllerValues.lastKnownPose.getPosition() + offset;

        // Just for testing
        mFlightControllerValues.trajectoryGoal = mFlightControllerValues.lastKnownPose.getPosition() + offset;

        if(mFlightControllerValues.lastKnownPose.getAge() > 100)
            qDebug() << "FlightController::setFlightState(): WARNING: going to hover, but pose being set as hover-target is" << mFlightControllerValues.lastKnownPose.getAge() << "ms old!";

        qDebug() << "FlightController::setFlightState(): going to hover, setting backup motion timer to high-freq and staying at" << mFlightControllerValues.hoverPosition;

        mBackupTimerComputeMotion->start(backupTimerIntervalFast);
        mFlightControllerValues.flightState = newFlightState;

        // We're going to use the controllers, so make sure to initialize them.
        initializeControllers();
        break;
    }

    case FlightState::State::UserControl:
    {
        // We don't need the timer, as the remote control will overrule our output anyway.
        qDebug() << "FlightController::setFlightState(): disabling backup motion timer.";
        mBackupTimerComputeMotion->stop();
        mFlightControllerValues.flightState = newFlightState;
        break;
    }

    case FlightState::State::Idle:
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

    qDebug() << "FlightController::setFlightState(): emitting flightcontrollervalues, flightstate and weights...";
    //emit flightControllerValues(&mFlightControllerValues);
    emit flightState(&mFlightControllerValues.flightState);
    emit flightControllerWeights();
    qDebug() << "...done";
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
    Q_ASSERT(mFlightControllerValues.flightState == FlightState::State::ApproachWayPoint && "FlightController::ensureSafeFlightAfterWaypointsChanged(): flightstate is NOT ApproachWayPoint!");

    if(mWayPoints.size() == 0)
    {
        qDebug() << "FlightController::ensureSafeFlightAfterWaypointsChanged(): wpt list is empty, going to hover";
        setFlightState(FlightState::State::Hover);

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

void FlightController::slotFlightStateRestrictionChanged(const FlightStateRestriction* const fsr)
{
    // This method adapts the flightstate to the new restriction coming in from the user

    qDebug() << __PRETTY_FUNCTION__ << "flightstate" << mFlightControllerValues.flightState.toString() << ": restriction changed from" << mFlightControllerValues.flightStateRestriction.toString() << "to:" << fsr->toString();

    mFlightControllerValues.flightStateRestriction.restriction = fsr->restriction;

    switch(mFlightControllerValues.flightState.state)
    {
    case FlightState::State::UserControl:
        switch(fsr->restriction)
        {
        case FlightStateRestriction::Restriction::RestrictionUserControl:
            // This is an illegal state: We are already in UserControl, and now we're
            // told that the user restricts us to UserControl. Thats impossible.
            Q_ASSERT(false && "We're in FlightState UserControl and now user restricts us to UserControl. Error.");
            break;
        case FlightStateRestriction::Restriction::RestrictionHover:
            qDebug() << __PRETTY_FUNCTION__ << "going to flightstate hover";
            setFlightState(FlightState::State::Hover);
            break;
        case FlightStateRestriction::Restriction::RestrictionNone:
            // We are in UserControl, but restrictions have been lifted. If we have waypoints, approach them. Else, hover.
            if(mWayPoints.size())
            {
                qDebug() << __PRETTY_FUNCTION__ << "no restrictions, waypoints present. Going to ApproachWaypoint.";
                setFlightState(FlightState::State::ApproachWayPoint);
            }
            else
            {
                qDebug() << __PRETTY_FUNCTION__ << "no restrictions, no waypoints. Going to Hover.";
                setFlightState(FlightState::State::Hover);
            }
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined Restriction!");
            break;
        }
        break;

    case FlightState::State::ApproachWayPoint:
        switch(fsr->restriction)
        {
        case FlightStateRestriction::Restriction::RestrictionUserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to UserControl, going to FlightState UserControl";
            setFlightState(FlightState::State::UserControl);
            break;
        case FlightStateRestriction::Restriction::RestrictionHover:
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to Hover, going to FlightState Hover";
            setFlightState(FlightState::State::Hover);
            break;
        case FlightStateRestriction::Restriction::RestrictionNone:
            // We are approaching waypoints, and now the user changed to ComputerControl?
            // Thats impossible, because we are already in ComputerControl (ApproachWayPoint)
            Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): we're in ApproachWayPoint and now user removed restrictions. Error.");
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined Restriction!");
            break;
        }
        break;

    case FlightState::State::Hover:
        switch(fsr->restriction)
        {
        case FlightStateRestriction::Restriction::RestrictionUserControl:
            // We are approaching waypoints, but the user wants to take over control. Ok.
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to UserControl, going from Hover to FlightState UserControl";
            setFlightState(FlightState::State::UserControl);
            break;
        case FlightStateRestriction::Restriction::RestrictionHover:
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to Hover, staying in FlightState Hover";
            break;
        case FlightStateRestriction::Restriction::RestrictionNone:
            if(mWayPoints.size())
            {
                qDebug() << __PRETTY_FUNCTION__ << "no restrictions, waypoints present. Going to ApproachWaypoint.";
                setFlightState(FlightState::State::ApproachWayPoint);
            }
            else
            {
                qDebug() << __PRETTY_FUNCTION__ << "no restrictions, no waypoints. Staying in Hover.";
            }
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined Restriction!");
            break;
        }
        break;

    case FlightState::State::Idle:
        switch(fsr->restriction)
        {
        case FlightStateRestriction::Restriction::RestrictionUserControl:
            // We are idling, but the user wants to take over control. Ok.
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to UserControl, going from Idle to FlightState UserControl";
            setFlightState(FlightState::State::UserControl);
            break;
        case FlightStateRestriction::Restriction::RestrictionHover:
            // We are idling, but the user wants us to hover. Hovering on the ground is not healthy!
            qDebug() << __PRETTY_FUNCTION__ << "Restriction to Hover. Thats fine, staying in Idle";
            break;
        case FlightStateRestriction::Restriction::RestrictionNone:
            qDebug() << __PRETTY_FUNCTION__ << "No Restriction. Thats fine, staying in Idle";
            break;
        default:
            Q_ASSERT("FlightController::slotFlightStateSwitchValueChanged(): Undefined Restriction!");
            break;
        }
        break;

    default:
        Q_ASSERT(false && "FlightController::slotFlightStateSwitchValueChanged(): illegal flightstate!");
    }

    qDebug() << __PRETTY_FUNCTION__ << "done, new flightstate" << mFlightControllerValues.flightState.toString();
    emit flightStateRestriction(&mFlightControllerValues.flightStateRestriction);
}

void FlightController::slotEmitFlightControllerInfo()
{
    // Usually called from BaseConnection::newConnection(), tell base about us...
    qDebug() << "FlightController::slotEmitFlightControllerInfo(): emitting flightcontrollervalues, flightstate, flightstaterestriction, controllerweights and" << mWayPoints.size() << "waypoints.";
    emit flightControllerValues(&mFlightControllerValues);
    emit flightControllerWeights();
    emit flightState(&mFlightControllerValues.flightState);
    emit flightStateRestriction(&mFlightControllerValues.flightStateRestriction);
    emit wayPoints(&mWayPoints, WayPointListSource::WayPointListSourceRover);
}

void FlightController::slotSetControllerWeights(const QString* const controllerName, const QMap<QChar,float>* const weights)
{
    qDebug() << "FlightController::slotSetControllerWeights(): setting new weights for flightstate" << mFlightControllerValues.flightState.toString() << "and controller" << *controllerName << ":" << *weights;

//    QMap<PidController*, QMap<QChar,float> > controllerWeights;

    FlightState::State assignedFlightState;

    // To which flightstate do we want to assing the new weights? By default, we assign to
    // the current flightstate - if we're in usercontrol or idle, assign to ApprochWayPoint
    if(mFlightControllerValues.flightState.state == FlightState::State::UserControl || mFlightControllerValues.flightState.state == FlightState::State::Idle)
        assignedFlightState = FlightState::State::ApproachWayPoint;
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
    emit flightControllerWeights();
}

// Get the position of the carrot (=hoverPosition) that the mule shall follow...
// When the vehicle goes straight down (trajectoryUnitVector is (0,-X,0), then the projection of the vehicle's
// position on the path will give us a trajectoryProgress of 0, thereby NOT sinking the hover-position. This
// is a theoretical problem, as the helicopter never is a 100% over the goal. With wind in simulation, it works.
// I'll have to cdouble-check what happens outside when
QVector3D FlightController::getHoverPosition(const QVector3D& trajectoryStart, const QVector3D& trajectoryGoal, const QVector3D& vehiclePosition)
{
    const QVector3D trajectoryUnitVector = (trajectoryGoal-trajectoryStart).normalized();
    const QVector3D vectorFromTrajectoryStartToVehicle = vehiclePosition - trajectoryStart;
    const float trajectoryLength = trajectoryStart.distanceToLine(trajectoryGoal, QVector3D());

    // If we project the vehicle's position on the trajectory, how far along are we? We bound to between 0 and trajectory-length.
    const float projectedTrajectoryProgress = qBound(0.0f, (float)QVector3D::dotProduct(trajectoryUnitVector, vectorFromTrajectoryStartToVehicle), trajectoryLength);

    // If the vehicle is directly above or below a waypoint, then the projection above won't help. Manually advance hoverPosition in that case.
    QVector3D leftToGo = trajectoryGoal - vehiclePosition;
    //const float leftToGoVertically = leftToGo.y();
    leftToGo.setY(0.0f);
    const float leftToGoHorizontally = leftToGo.length();
    if(leftToGoHorizontally < mDistanceWayPointReached && mTrajectoryProgress < trajectoryLength)
    {
        qDebug() << "just need to go up or down, horizontally fine. artificially moving raspberry by 3cm";
        mTrajectoryProgress += 0.03;
    }

    mTrajectoryProgress = std::max(mTrajectoryProgress, projectedTrajectoryProgress);

    qDebug() << "FlightController::getHoverPosition(): TS" << trajectoryStart << "VP" << vehiclePosition << "TUV" << trajectoryUnitVector << "TL" << trajectoryLength << "VFTSTV" << vectorFromTrajectoryStartToVehicle << "PTP" << projectedTrajectoryProgress << "MTP" << mTrajectoryProgress;

    return QVector3D(trajectoryStart + (trajectoryUnitVector * mTrajectoryProgress));
}

void FlightController::slotSetFlightSpeed(const float flightSpeed)
{
    qDebug() << "FlightController::slotSetFlightSpeed(): changing maximum flightspeed per axis from" << mMaxFlightVelPerAxis << "m/s to" << flightSpeed << "m/s";
    mMaxFlightVelPerAxis = flightSpeed;
}
