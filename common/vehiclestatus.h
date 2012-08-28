#ifndef VEHICLESTATUS_H
#define VEHICLESTATUS_H

#include <QDebug>
#include <QObject>
#include <QDataStream>

class VehicleStatus : public QObject
{
    Q_OBJECT
public:
    VehicleStatus(QObject *parent = 0);

    QString toString() const;

    quint32 missionRunTime;
    float batteryVoltage;
    qint16 barometricHeight;
    qint8 wirelessRssi;
    
signals:
    
public slots:
    
};


// for using qDebug();
QDebug operator<<(QDebug dbg, const VehicleStatus &mc);

// for streaming
QDataStream& operator<<(QDataStream &out, const VehicleStatus &mc);
QDataStream& operator>>(QDataStream &in, VehicleStatus &mc);

#endif // VEHICLESTATUS_H
