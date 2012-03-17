#include "serialport.h"

SerialPort::SerialPort(QString serialDeviceFile, QString settings) : Port()
{
    const QStringList settingsList = settings.split(',', QString::KeepEmptyParts);

    // Set default parameters
    QString portBaudRate = "115200";
    QString portDataBits = "8";
    QString portParity = "None";
    QString portStopBits = "1";
    QString portFlowControl = "Disable";

    if(settingsList.size() > 0) portBaudRate = settingsList.at(0);
    if(settingsList.size() > 1) portDataBits = settingsList.at(1);
    if(settingsList.size() > 2) portParity = settingsList.at(2);
    if(settingsList.size() > 3) portStopBits = settingsList.at(3);
    if(settingsList.size() > 4) portFlowControl = settingsList.at(4);

    // Do some replacements
    if(portParity == "None") portParity == "Disable";
    if(portFlowControl.toLower().contains("xo")) portFlowControl == "Xon/Xoff";
    if(portFlowControl.toLower().contains("radwar")) portFlowControl == "Hardware";

    // Show possible settings, unfinished and unneccesary
    //QMapIterator<AbstractSerial::BaudRate, QString> i(mSerialPort->baudRateMap());
    //while (i.hasNext()) { i.next(); qDebug() << i.value() << endl; }

    qDebug() << "SerialPort::SerialPort(): Default settings are 115200,8,None,1,Disable, effective settings are" << portBaudRate << portDataBits << portParity << portStopBits << portFlowControl;

    mSerialPort = new AbstractSerial();
    mSerialPort->enableEmitStatus(true);
    connect(mSerialPort, SIGNAL(signalStatus(QString,QDateTime)), SLOT(slotSerialPortStatusChanged(QString,QDateTime)));
    mSerialPort->setDeviceName(serialDeviceFile);
    if(!mSerialPort->open(AbstractSerial::ReadWrite))
    {
        mSerialPort->close();
        qFatal("SerialPort::SerialPort(): Opening serial port %s failed, exiting.", qPrintable(serialDeviceFile));
    }
    mSerialPort->setBaudRate(portBaudRate);
    mSerialPort->setDataBits(portDataBits + " bit"); // Yes, "8" doesn't work, it needs to be "8 bit"
    mSerialPort->setParity(portParity);
    mSerialPort->setStopBits(portStopBits); // Here 1, 1.5, 2 work fine.
    mSerialPort->setFlowControl(portFlowControl); // "Off" doesn't work, it needs to be "Disable"

    if(mSerialPort->isOpen())
    {
        qDebug() << "SerialPort::SerialPort(): Opening serial port" << serialDeviceFile << "succeeded, settings:"
                 << "speed" << mSerialPort->baudRate()
                 << "data" << mSerialPort->dataBits()
                 << "parity" << mSerialPort->parity()
                 << "stop" << mSerialPort->stopBits()
                 << "flow" << mSerialPort->flowControl();
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

void SerialPort::slotSerialPortStatusChanged(const QString& status, const QDateTime& time)
{
    qDebug() << "SerialPort::slotSerialPortStatusChanged():" << status << ((mSerialPort->errorString() != QString("Unknown error")) ? QString(mSerialPort->errorString()) : "");
}
