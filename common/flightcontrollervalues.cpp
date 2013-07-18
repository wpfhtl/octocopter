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
