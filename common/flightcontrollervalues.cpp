#include "flightcontrollervalues.h"

FlightControllerValues::FlightControllerValues()
{

}

FlightControllerValues::FlightControllerValues(const QString& fcvString)
{
    // timestamp SEPARATOR pose SEPARATOR waypoint SEPARATOR motioncommand
    QStringList list = fcvString.split(SEPARATOR, QString::KeepEmptyParts);

    lastKnownPose = Pose(list.at(1));
    nextWayPoint = WayPoint(list.at(2));
    motionCommand = MotionCommand(list.at(3));
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv)
{
    out << fcv.motionCommand << fcv.nextWayPoint << fcv.lastKnownPose << fcv.lastKnownHeightOverGround;
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightControllerValues& fcv)
{
    in >> fcv.motionCommand;
    in >> fcv.nextWayPoint;
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
    out.append(nextWayPoint.toString());
    out.append(SEPARATOR);
    out.append(motionCommand.toString());

    return out;
}
