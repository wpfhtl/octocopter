#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{

}

FlightControllerValues::FlightControllerValues(const QString& fcvString)
{
    // The string is of format:
    // timestamp SPACE flightState SEPARATOR pose SEPARATOR waypoint SEPARATOR motioncommand
    // We remove the timestamp at the beginning and split the rest using SEPARATOR
    QStringList list = fcvString.mid(fcvString.indexOf(' ') + 1).split(SEPARATOR, QString::SkipEmptyParts);

    flightState = FlightState::fromString(list.at(0));

    lastKnownPose = Pose(list.at(1));

    // Target position and motion command are always present, just sometimes zero.
    const QStringList targetPositionStringList = list.at(2).split(" ");
    targetPosition.setX(targetPositionStringList.at(0).toDouble());
    targetPosition.setY(targetPositionStringList.at(1).toDouble());
    targetPosition.setZ(targetPositionStringList.at(2).toDouble());

    motionCommand = MotionCommand(list.at(3));
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    out << fcv.motionCommand << fcv.flightState << fcv.targetPosition << fcv.lastKnownPose << fcv.lastKnownHeightOverGround;
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.motionCommand;
    in >> ((quint8&)fcv.flightState);
    in >> fcv.targetPosition;
    in >> fcv.lastKnownPose;
    in >> fcv.lastKnownHeightOverGround;
    return in;
}


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

    return out;
}
