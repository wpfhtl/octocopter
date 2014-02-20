#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{
    // Initialize, so we have clean logdata.
    timestamp = 0;
    flightState = FlightState();
    flightStateRestriction = FlightStateRestriction();
    motionCommand = MotionCommand();
    trajectoryStart = trajectoryGoal = hoverPosition = QVector3D();
    lastKnownPose = Pose();
    lastKnownHeightOverGround = 0.0f;
    lastKnownHeightOverGroundTimestamp = 0;
}

QString FlightControllerValues::toString() const{
    return QString("lastKnownPose %1, timestamp %2, hoverPos %3/%4/%5, trajStart %6/%7/%8, trajGoal %9/%10/%11, flightState %12, heightOverGround %13")
            .arg(lastKnownPose.toString(true))
            .arg(timestamp)
            .arg(hoverPosition.x()).arg(hoverPosition.y()).arg(hoverPosition.z())
            .arg(trajectoryStart.x()).arg(trajectoryStart.y()).arg(trajectoryStart.z())
            .arg(trajectoryGoal.x()).arg(trajectoryGoal.y()).arg(trajectoryGoal.z())
            .arg(flightState.toString())
            .arg(lastKnownHeightOverGround);
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    // timestamp must come first, we look for it in the logfiles so that
    // LogPlayer can seek. http://qt-project.org/doc/qt-4.8/datastreamformat.html
    // tells us how the serialization works.
    out
            << fcv.timestamp
            << fcv.lastKnownPose
            << fcv.motionCommand
            << fcv.flightState
            << fcv.flightStateRestriction
            << fcv.trajectoryStart
            << fcv.trajectoryGoal
            << fcv.hoverPosition
            << fcv.lastKnownHeightOverGround
            << fcv.lastKnownHeightOverGroundTimestamp
            << fcv.controllerThrust
            << fcv.controllerYaw
            << fcv.controllerPitch
            << fcv.controllerRoll;

    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.timestamp;
    in >> fcv.lastKnownPose;
    in >> fcv.motionCommand;
    in >> fcv.flightState;
    in >> fcv.flightStateRestriction;
    in >> fcv.trajectoryStart;
    in >> fcv.trajectoryGoal;
    in >> fcv.hoverPosition;
    in >> fcv.lastKnownHeightOverGround;
    in >> fcv.lastKnownHeightOverGroundTimestamp;
    in >> fcv.controllerThrust;
    in >> fcv.controllerYaw;
    in >> fcv.controllerPitch;
    in >> fcv.controllerRoll;
    return in;
}
