#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{
    // Initialize, so we have clean logdata.
    timestamp = 0;
    flightState = FlightState();
    motionCommand = MotionCommand();
    trajectoryStart = trajectoryGoal = hoverPosition = QVector3D();
    lastKnownPose = Pose();
    lastKnownHeightOverGround = 0.0f;
    lastKnownHeightOverGroundTimestamp = 0;
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    out
            << fcv.timestamp
            << fcv.motionCommand
            << fcv.flightState
            << fcv.trajectoryStart
            << fcv.trajectoryGoal
            << fcv.hoverPosition
            << fcv.lastKnownPose
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
    in >> fcv.motionCommand;
    in >> ((quint8&)fcv.flightState);
    in >> fcv.trajectoryStart;
    in >> fcv.trajectoryGoal;
    in >> fcv.hoverPosition;
    in >> fcv.lastKnownPose;
    in >> fcv.lastKnownHeightOverGround;
    in >> fcv.lastKnownHeightOverGroundTimestamp;
    in >> fcv.controllerThrust;
    in >> fcv.controllerYaw;
    in >> fcv.controllerPitch;
    in >> fcv.controllerRoll;
    return in;
}
