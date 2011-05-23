#include "gpsdevice.h"

GpsDevice::GpsDevice(QString &serialDeviceFile, QObject *parent) : QObject(parent)
{
    mNumberOfRemainingReplies = 0;
    mSerialPortOnDevice = "";

    mSerialPort = new QextSerialPort(serialDeviceFile, QextSerialPort::EventDriven);
    mSerialPort->setBaudRate(BAUD115200);
    mSerialPort->setFlowControl(FLOW_OFF);
    mSerialPort->setParity(PAR_NONE);
    mSerialPort->setDataBits(DATA_8);
    mSerialPort->setStopBits(STOP_1);
    mSerialPort->setTimeout(500);

    mSerialPort->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPort->isOpen())
    {
        qDebug() << "Opening serial port" << serialDeviceFile << "failed:" << mSerialPort->errorString() << "Exiting.";
        exit(1);
    }

    determineSerialPortOnDevice();

    connect(mSerialPort, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));

    rtkOutputInitialize();

    rtkOutputStart();
}

GpsDevice::~GpsDevice()
{
    qDebug() << "GpsDevice::~GpsDevice(): stopping rtk output, closing seial port";
    rtkOutputStop();
    mSerialPort->close();
}

void GpsDevice::sendAsciiCommand(QString command)
{
    mCommandQueue.append(command.append("\r\n").toAscii());

    slotFlushCommandQueue();
}

void GpsDevice::slotFlushCommandQueue()
{
    if(mNumberOfRemainingReplies == 0 && mCommandQueue.size())
    {
        mLastCommandToDevice = mCommandQueue.takeFirst();
        qDebug() << "GpsDevice::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDevice.trimmed();
        if(mReceiveBuffer.size() != 0) qDebug() << "GpsDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBuffer;

        mSerialPort->write(mLastCommandToDevice);
        mNumberOfRemainingReplies++;
    }
    else if(mNumberOfRemainingReplies)
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingReplies << "command-replies, not sending.";
    }
    else
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): nothing to send.";
    }
}

void GpsDevice::determineSerialPortOnDevice()
{
    Q_ASSERT(mSerialPort->isOpen());

    // We just send any string causing a reply to the device, so we can see on what port it is talking to us.
    mSerialPort->write("setDataInOut,all,CMD,none\n");
    usleep(100000);
    mSerialPort->write("getReceiverCapabilities\n");
    usleep(100000);
    QCoreApplication::processEvents();

    // After waiting for the reply, read and analyze.
    QByteArray data = mSerialPort->readAll();
    QString port = data.right(5).left(4);

    // Use line sending with e.g. COM2 or USB1 to determine the port on the serial device being used.
    if(data.right(1) == ">" && (port.left(3) == "COM" || port.left(3) == "USB"))
    {
        mSerialPortOnDevice = port;
        qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial port on device is now " << mSerialPortOnDevice;
        qDebug() << "GpsDevice::determineSerialPortOnDevice(): other non-rtk data:" << data;
    }
    else
    {
        qDebug() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialPortOnDevice, data is:" << data;
        exit(1);
    }
}

void GpsDevice::rtkOutputInitialize()
{
    Q_ASSERT(mSerialPortOnDevice.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Base-Mode

    qDebug() << "GpsDevice::rtkOutputInitialize(): sending rtk output init commands from rtk-init.txt";

    QFile file("rtk-init.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "GpsDevice::rtkOutputInitialize(): couldn't read rtk-init.txt to initialize gps device";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "GpsDevice::rtkOutputInitialize(): done sending rtk-init.txt";

/* old, we read the commands from a file now
    sendAsciiCommand("setPVTMode,Static,,auto");
    sendAsciiCommand("setRAIMLevels,on,-2,-2,-3");
    sendAsciiCommand("setAntennaOffset,Main,,,,\"antennadescriptor\", \"12345\""); // for RTCMv3
    sendAsciiCommand("setRTCMv2Interval,RTCM3,10");
    sendAsciiCommand("setRTCMv2Formatting,0"); // basestation id, 0 is default

    // Only real serial ports need baud speed, not usb ones.
    if(mSerialPortOnDevice.contains("COM")) sendAsciiCommand("setCOMSettings," + mSerialPortOnDevice + ",baud115200");

    sendAsciiCommand("setSmoothingInterval,all,900");
    sendAsciiCommand("setClockSyncThreshold,usec500");
//    sendAsciiCommand("setRTCMv2Output," + mSerialPortOnDevice + ",RTCM1+RTCM3+RTCM9+RTCM16+RTCM18+RTCM19+RTCM20+RTCM21+RTCM22+RTCM31+RTCM32"); anythign above 16 seems broken
    sendAsciiCommand("setRTCMv2Output," + mSerialPortOnDevice + ",RTCM1+RTCM3+RTCM9+RTCM16");
*/

}

void GpsDevice::rtkOutputStart()
{
    qDebug() << "GpsDevice::rtkOutputInitialize(): sending rtk output init commands from rtk-start.txt";

    QFile file("rtk-start.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "GpsDevice::rtkOutputInitialize(): couldn't read rtk-start.txt to start rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "GpsDevice::rtkOutputInitialize(): done sending rtk-start.txt";

//    sendAsciiCommand("setDataInOut," + mSerialPortOnDevice + ",,RTCMv2");
}

void GpsDevice::rtkOutputStop()
{
    usleep(100000);
//    mSerialPort->write("SSSSSSSSSS");
    QCoreApplication::processEvents();
//    sleep(11);

    qDebug() << "GpsDevice::rtkOutputInitialize(): sending rtk output init commands from rtk-stop.txt";

    QFile file("rtk-stop.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "GpsDevice::rtkOutputInitialize(): couldn't read rtk-stop.txt to stop rtk output";
        QCoreApplication::quit();
    }

    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        line.replace("$PORT", mSerialPortOnDevice.toAscii());
        sendAsciiCommand(line.trimmed());
    }

    qDebug() << "GpsDevice::rtkOutputInitialize(): done sending rtk-stop.txt";


//    sendAsciiCommand("setDataInOut,all,CMD,none");
}

void GpsDevice::slotSerialPortDataReady()
{
    usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBuffer.append(mSerialPort->readAll());

//    qDebug() << "GpsDevice::slotSerialPortDataReady():" << mReceiveBuffer;

    if(mNumberOfRemainingReplies != 0)
    {
        const int position = mReceiveBuffer.indexOf(mSerialPortOnDevice + QString(">"));
        if(position != -1)
        {
            qDebug() << "GpsDevice::slotSerialPortDataReady(): received reply to" << mLastCommandToDevice.trimmed() << ":" << mReceiveBuffer.left(position).trimmed();
            qDebug() << "GpsDevice::slotSerialPortDataReady(): now sending next command, if any";

            if(mReceiveBuffer.left(position).contains("$R? ASCII commands between prompts were discarded!")) qDebug() << "GpsDevice::slotSerialPortDataReady(): it seems we were talking too fast?!";

            mReceiveBuffer.remove(0, position+5);
            mNumberOfRemainingReplies--;
            slotFlushCommandQueue();
        }
    }
    else
    {
        // We're not waiting for a reply to a command, this must be correction data!
//        qDebug() << "GpsDevice::slotSerialPortDataReady(): emitting" << mReceiveBuffer.size() << "bytes of correction data.";
        emit correctionDataReady(mReceiveBuffer);
        mReceiveBuffer.clear();
    }

}
