#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QFile>
#include <QDebug>
#include <QStringList>

#include <QtSerialPort/QSerialPort>
#include "port.h"

class SerialPort : public Port
{
    Q_OBJECT

public:
    SerialPort(QString serialDeviceUsb, QString settings = QString());
    ~SerialPort();

private:
    QSerialPort *mSerialPort;

private slots:
    void slotSerialPortDataReady();
    void slotSerialPortError(const QSerialPort::SerialPortError &error);

public slots:
    void write(const QByteArray &data);

signals:
    void data(const QByteArray&);
};

#endif // SERIALPORT_H
