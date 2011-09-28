#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QFile>
#include <QDebug>

#include "port.h"

class QextSerialPort;

class SerialPort : public Port
{
    Q_OBJECT

public:
    SerialPort(QString serialDeviceUsb);
    ~SerialPort();

private:
    QextSerialPort *mSerialPort;

private slots:
    void slotSerialPortDataReady();

public slots:
    void write(const QByteArray &data);

signals:
    void data(const QByteArray&);
};

#endif // SERIALPORT_H
