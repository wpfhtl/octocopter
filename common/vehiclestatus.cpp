#include "vehiclestatus.h"
#include <QFile>

VehicleStatus::VehicleStatus(QObject *parent) :
    QObject(parent),
    missionRunTime(0),
    batteryVoltage(-1.0),
    barometricHeight(-1),
    wirelessRssi(-1),
    cpuTemperature(-1)
{
}

QString VehicleStatus::toString() const
{
    return QString("vehiclestatus: runtime %1, voltage %2, baroheight %3, rssi %4, cputemp %5").arg(missionRunTime).arg((double)batteryVoltage, 0, 'f', 2).arg(barometricHeight).arg(wirelessRssi).arg(cpuTemperature);
}

void VehicleStatus::updateCpuTemperature(const QString fileName)
{
    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << __PRETTY_FUNCTION__ << "cannot open" << fileName << "for reading CPU temperature";
        return;
    }

    QTextStream in(&file);
    QString line = in.readLine();
    if(line.isNull())
    {
        qDebug() << __PRETTY_FUNCTION__ << "cannot read line from" << fileName;
        return;
    }

    bool ok = false;
    cpuTemperature = (line.toInt(&ok) / 1000);

    if(!ok)
    {
        qDebug() << __PRETTY_FUNCTION__ << "cannot interpret CPU temmperature" << line;
        cpuTemperature = 0;
        return;
    }
}

QDataStream& operator<<(QDataStream &out, const VehicleStatus &vs)
{
    out << vs.missionRunTime;
    out << vs.batteryVoltage;
    out << vs.barometricHeight;
    out << vs.wirelessRssi;
    out << vs.cpuTemperature;

    return out;
}

QDataStream& operator>>(QDataStream &in, VehicleStatus &vs)
{
    in >> vs.missionRunTime;
    in >> vs.batteryVoltage;
    in >> vs.barometricHeight;
    in >> vs.wirelessRssi;
    in >> vs.cpuTemperature;

    return in;
}

QDebug operator<<(QDebug dbg, const VehicleStatus &vs)
{
    dbg << vs.toString();
    return dbg;
}
