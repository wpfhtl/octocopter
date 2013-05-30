#include "serialport.h"

SerialPort::SerialPort(QString serialDeviceFile, QString settings) : Port()
{
    const QStringList settingsList = settings.split(',', QString::KeepEmptyParts);

    // Set default parameters
    QString portBaudRate = "115200";
    QString portDataBits = "8";
    QString portParity = "none";
    QString portStopBits = "1";
    QString portFlowControl = "Off";

    if(settingsList.size() > 0 && !settingsList.at(0).isEmpty()) portBaudRate = settingsList.at(0);
    if(settingsList.size() > 1 && !settingsList.at(1).isEmpty()) portDataBits = settingsList.at(1);
    if(settingsList.size() > 2 && !settingsList.at(2).isEmpty()) portParity = settingsList.at(2);
    if(settingsList.size() > 3 && !settingsList.at(3).isEmpty()) portStopBits = settingsList.at(3);
    if(settingsList.size() > 4 && !settingsList.at(4).isEmpty()) portFlowControl = settingsList.at(4);

    qDebug() << "SerialPort::SerialPort(): Default settings are 115200,8,None,1,Disable, settingsstring was" << settings;
    qDebug() << "SerialPort::SerialPort(): Effective settings are" << portBaudRate << portDataBits << portParity << portStopBits << portFlowControl;

    mSerialPort = new QSerialPort(serialDeviceFile, this);
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)), SLOT(slotSerialPortError(QSerialPort::SerialPortError)));
    if(!mSerialPort->open(QIODevice::ReadWrite))
    {
        mSerialPort->close();
        qFatal("SerialPort::SerialPort(): Opening serial port %s failed, exiting.", qPrintable(serialDeviceFile));
    }

    mSerialPort->setBaudRate(portBaudRate.toInt());

    if(portDataBits == "8") mSerialPort->setDataBits(QSerialPort::Data8);
    else if(portDataBits == "7") mSerialPort->setDataBits(QSerialPort::Data7);
    else if(portDataBits == "6") mSerialPort->setDataBits(QSerialPort::Data6);
    else if(portDataBits == "5") mSerialPort->setDataBits(QSerialPort::Data5);

    if(portParity.toLower() == "none") mSerialPort->setParity(QSerialPort::NoParity);
    else if(portParity.toLower() == "even") mSerialPort->setParity(QSerialPort::EvenParity);
    else if(portParity.toLower() == "odd") mSerialPort->setParity(QSerialPort::OddParity);
    else if(portParity.toLower() == "space") mSerialPort->setParity(QSerialPort::SpaceParity);
    else if(portParity.toLower() == "mark") mSerialPort->setParity(QSerialPort::MarkParity);

    if(portStopBits == "1") mSerialPort->setStopBits(QSerialPort::OneStop);
    else if(portStopBits == "1.5") mSerialPort->setStopBits(QSerialPort::OneAndHalfStop);
    else if(portStopBits == "2") mSerialPort->setStopBits(QSerialPort::TwoStop);

    if(portFlowControl.toLower() == "off") mSerialPort->setFlowControl(QSerialPort::NoFlowControl);
    else if(portFlowControl.toLower() == "hardware") mSerialPort->setFlowControl(QSerialPort::HardwareControl);
    else if(portFlowControl.toLower() == "software") mSerialPort->setFlowControl(QSerialPort::SoftwareControl);

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

void SerialPort::slotSerialPortError(const QSerialPort::SerialPortError& error)
{
    qDebug() << __PRETTY_FUNCTION__ << "QSerialPort::SerialPortError" << error;
}
