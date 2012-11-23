#ifndef WIRELESSDEVICE_H
#define WIRELESSDEVICE_H

#include <QString>
#include <QStringList>
#include <QTimer>
#include <QDebug>
#include <QFile>

class WirelessDevice : public QObject
{
    Q_OBJECT

private:
    QString mInterfaceName;
    QTimer* mUpdateTimer;

private slots:
    void slotEmitRssi();

signals:
    void rssi(qint8);

public:
    WirelessDevice(const QString& interfaceName);
    ~WirelessDevice();

    qint8 getRssi();
};

#endif
