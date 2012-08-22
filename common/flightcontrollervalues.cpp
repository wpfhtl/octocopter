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
            << fcv.controllerThrust
            << fcv.controllerYaw
            << fcv.controllerPitch
            << fcv.controllerRoll;

    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.motionCommand;
    in >> ((quint8&)fcv.flightState);
    in >> fcv.targetPosition;
    in >> fcv.lastKnownPose;
    in >> fcv.lastKnownHeightOverGround;

    in >> fcv.controllerThrust;
    in >> fcv.controllerYaw;
    in >> fcv.controllerPitch;
    in >> fcv.controllerRoll;
    return in;
}

/*
QString FlightControllerValues::toString() const
{
    QString out;

    out.append(flightState.toString());
    out.append(SEPARATOR);
    out.append(lastKnownPose.toString());
    out.append(SEPARATOR);
    QString targetPositionString = QString("%1 %2 %3").arg(targetPosition.x()).arg(targetPosition.y()).arg(targetPosition.z());
    out.append(targetPositionString);
    out.append(SEPARATOR);
    out.append(motionCommand.toString());
    out.append(SEPARATOR);
    out.append(controllerThrust.toString());
    out.append(SEPARATOR);
    out.append(controllerYaw.toString());
    out.append(SEPARATOR);
    out.append(controllerPitch.toString());
    out.append(SEPARATOR);
    out.append(controllerRoll.toString());

    return out;
}
*/
