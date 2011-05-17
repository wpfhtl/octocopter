#ifndef WIRELESSDEVICE_H
#define WIRELESSDEVICE_H

#include <QString>
#include <QStringList>
#include <QDebug>
#include <QFile>



class WirelessDevice
{
private:
    QString mInterfaceName;

public:
    WirelessDevice(const QString& interfaceName);

    qint8 getRssi();
};

#endif
