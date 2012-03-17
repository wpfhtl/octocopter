#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QFile>
#include <QDebug>
#include <QStringList>

#include <abstractserial.h>
#include "port.h"

class SerialPort : public Port
{
    Q_OBJECT

public:
    SerialPort(QString serialDeviceUsb, QString settings = QString());
    ~SerialPort();

private:
    AbstractSerial *mSerialPort;

private slots:
    void slotSerialPortDataReady();
    void slotSerialPortStatusChanged(const QString& status, const QDateTime& time);

public slots:
    void write(const QByteArray &data);

signals:
    void data(const QByteArray&);
};

#endif // SERIALPORT_H
