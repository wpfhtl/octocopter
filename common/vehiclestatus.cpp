#include "vehiclestatus.h"

VehicleStatus::VehicleStatus(QObject *parent) :
    QObject(parent)
{
}

QString VehicleStatus::toString() const
{
    return QString("vehiclestatus: runtime %1, voltage %2, baroheight %3, rssi %4").arg(missionRunTime).arg((double)batteryVoltage, 0, 'f', 2).arg(barometricHeight).arg(wirelessRssi);
}

QDataStream& operator<<(QDataStream &out, const VehicleStatus &vs)
{
    out << vs.missionRunTime;
    out << vs.batteryVoltage;
    out << vs.barometricHeight;
    out << vs.wirelessRssi;

    return out;
}

QDataStream& operator>>(QDataStream &in, VehicleStatus &vs)
{
    in >> vs.missionRunTime;
    in >> vs.batteryVoltage;
    in >> vs.barometricHeight;
    in >> vs.wirelessRssi;

    return in;
}

QDebug operator<<(QDebug dbg, const VehicleStatus &vs)
{
    dbg << vs.toString();
    return dbg;
}
