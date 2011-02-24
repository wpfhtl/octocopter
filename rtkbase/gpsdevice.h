#ifndef GPSDEVICE_H
#define GPSDEVICE_H

#include <QCoreApplication>
#include <QObject>
#include <QDebug>

#include "qextserialport/src/qextserialport.h"

class GpsDevice : public QObject
{
    Q_OBJECT

public:
    GpsDevice(QString &serialDeviceFile, QObject *parent = 0);
    ~GpsDevice();

private:
    int mNumberOfRemainingReplies;
    QByteArray mLastCommandToDevice;
    QextSerialPort *mSerialPort;
    QString mSerialPortOnDevice;
    QByteArray mReceiveBuffer;
    QList<QByteArray> mCommandQueue;

    void determineSerialPortOnDevice();
    void rtkOutputInitialize();
    void rtkOutputStart();
    void rtkOutputStop();

    void sendAsciiCommand(QString command);

public slots:
    void slotFlushCommandQueue();
    void slotSerialPortDataReady();

signals:
    void correctionDataReady(QByteArray);

};

#endif // GPSDEVICE_H
