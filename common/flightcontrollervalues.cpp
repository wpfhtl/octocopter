#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{

}

FlightControllerValues::FlightControllerValues(const QString& fcvString)
{
    // timestamp SEPARATOR flightState SEPARATOR pose SEPARATOR waypoint SEPARATOR motioncommand
    QStringList list = fcvString.split(SEPARATOR, QString::KeepEmptyParts);

    flightState = FlightState::fromString(list.at(1));

    lastKnownPose = Pose(list.at(2));

    // Target position and motion command are only present in some flightStates, not all of them.
    // WRONG: always present, sometimes zero.
//    if(list.size() > 3)
//    {
        const QStringList targetPositionStringList = list.at(3).split(" ");
        targetPosition.setX(targetPositionStringList.at(0).toDouble());
        targetPosition.setY(targetPositionStringList.at(1).toDouble());
        targetPosition.setZ(targetPositionStringList.at(2).toDouble());
//    }

//    if(list.size() > 4)
//    {
        motionCommand = MotionCommand(list.at(4));
//    }
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

    out.append(QString::number(lastKnownPose.timestamp));
    out.append(SEPARATOR);
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
