#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{

}

FlightControllerValues::FlightControllerValues(const QString& fcvString)
{
    // timestamp SEPARATOR pose SEPARATOR waypoint SEPARATOR motioncommand
    QStringList list = fcvString.split(SEPARATOR, QString::KeepEmptyParts);

    lastKnownPose = Pose(list.at(1));

    const QStringList targetPositionStringList = list.at(2).split(" ");
    targetPosition.setX(targetPositionStringList.at(0).toDouble());
    targetPosition.setY(targetPositionStringList.at(1).toDouble());
    targetPosition.setZ(targetPositionStringList.at(2).toDouble());

    motionCommand = MotionCommand(list.at(3));
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    out << fcv.motionCommand << fcv.targetPosition << fcv.lastKnownPose << fcv.lastKnownHeightOverGround;
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.motionCommand;
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
    out.append(lastKnownPose.toString());
    out.append(SEPARATOR);
    QString targetPositionString = QString("%1 %2 %3").arg(targetPosition.x()).arg(targetPosition.y()).arg(targetPosition.z());
    out.append(targetPositionString);
    out.append(SEPARATOR);
    out.append(motionCommand.toString());

    return out;
}
