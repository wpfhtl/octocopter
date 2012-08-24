#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{

}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    out
            << fcv.motionCommand
            << fcv.flightState
            << fcv.targetPosition
            << fcv.lastKnownPose
            << fcv.lastKnownHeightOverGround
            << fcv.lastKnownHeightOverGroundTimestamp
            << fcv.pidControllers;

    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.motionCommand;
    in >> ((quint8&)fcv.flightState);
    in >> fcv.targetPosition;
    in >> fcv.lastKnownPose;
    in >> fcv.lastKnownHeightOverGround;
    in >> fcv.lastKnownHeightOverGroundTimestamp;
    in >> fcv.pidControllers;
    return in;
}
