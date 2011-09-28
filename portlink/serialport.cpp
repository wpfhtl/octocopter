#include "qextserialport/qextserialport.h"

#include "serialport.h"

SerialPort::SerialPort(QString serialDeviceFile) : Port()
{
    // We use the USB port to talk to the GPS receiver and receive poses
    mSerialPort = new QextSerialPort(serialDeviceFile, QextSerialPort::EventDriven);
    mSerialPort->setBaudRate(BAUD115200);
    mSerialPort->setFlowControl(FLOW_OFF);
    mSerialPort->setParity(PAR_NONE);
    mSerialPort->setDataBits(DATA_8);
    mSerialPort->setStopBits(STOP_1);
    mSerialPort->setTimeout(500);

    mSerialPort->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(mSerialPort->isOpen())
    {
        qDebug() << "SerialPort::SerialPort(): Opening serial port" << serialDeviceFile << "succeeded.";
    }
    else
    {
        qDebug() << "SerialPort::SerialPort(): Opening serial port" << serialDeviceFile << "failed:" << mSerialPort->errorString() << "Exiting.";
        exit(1);
    }

    connect(mSerialPort, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));
}

SerialPort::~SerialPort()
{
    qDebug() << "SerialPort::~SerialPort(): stopping output, closing serial ports";
    mSerialPort->close();
}

void SerialPort::write(const QByteArray& data)
{
    mSerialPort->write(data);
}

void SerialPort::slotSerialPortDataReady()
{
    emit data(mSerialPort->readAll());
}
