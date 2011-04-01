#ifndef GPSDEVICE_H
#define GPSDEVICE_H

#include <QCoreApplication>
#include <QFile>
#include <QDebug>

#include "qextserialport/src/qextserialport.h"

class GpsDevice : public QObject
{
    Q_OBJECT

public:
    GpsDevice(QString &serialDeviceUsb, QString &serialDeviceCom, QObject *parent = 0);
    ~GpsDevice();

private:
    int mNumberOfRemainingRepliesUsb;
    QByteArray mLastCommandToDeviceUsb;
    QextSerialPort *mSerialPortUsb, *mSerialPortCom;

    // The ports we use to talk to the receiver have a name on the receiver-side, e.g. COM1 or USB2
    QString mSerialPortOnDeviceUsb, mSerialPortOnDeviceCom;

    QByteArray mReceiveBufferUsb;
    QList<QByteArray> mCommandQueueUsb;

    void determineSerialPortsOnDevice();
    void communicationSetup();
    void communicationStop();

    void sendAsciiCommand(QString command);

private slots:
    void slotFlushCommandQueue();
    void slotSerialPortDataReady();

public slots:
    void slotSetRtkData(const QByteArray &data);

signals:
    void correctionDataReady(QByteArray);

};

#endif // GPSDEVICE_H
