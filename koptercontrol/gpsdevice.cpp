#include "gpsdevice.h"

GpsDevice::GpsDevice(QString &serialDeviceFileUsb, QString &serialDeviceFileCom, QObject *parent) : QObject(parent)
{
    qDebug() << "GpsDevice::GpsDevice(): Using usb port" << serialDeviceFileUsb << "and com port" << serialDeviceFileCom;

    mNumberOfRemainingRepliesUsb = 0;
    mRtkDataCounter = 0;
    mSerialPortOnDeviceUsb = "";

    // We use the USB port to talk to the GPS receiver and receive poses
    mSerialPortUsb = new QextSerialPort(serialDeviceFileUsb, QextSerialPort::EventDriven);
    mSerialPortUsb->setBaudRate(BAUD115200);
    mSerialPortUsb->setFlowControl(FLOW_OFF);
    mSerialPortUsb->setParity(PAR_NONE);
    mSerialPortUsb->setDataBits(DATA_8);
    mSerialPortUsb->setStopBits(STOP_1);
    mSerialPortUsb->setTimeout(500);

    mSerialPortUsb->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPortUsb->isOpen())
    {
        qDebug() << "GpsDevice::GpsDevice(): Opening serial port" << serialDeviceFileUsb << "failed:" << mSerialPortUsb->errorString() << "Exiting.";
        exit(1);
    }

    // We use the COM port to feed the device RTK correction data
    mSerialPortCom = new QextSerialPort(serialDeviceFileCom, QextSerialPort::EventDriven);
    mSerialPortCom->setBaudRate(BAUD115200);
    mSerialPortCom->setFlowControl(FLOW_OFF);
    mSerialPortCom->setParity(PAR_NONE);
    mSerialPortCom->setDataBits(DATA_8);
    mSerialPortCom->setStopBits(STOP_1);
    mSerialPortCom->setTimeout(500);

    mSerialPortCom->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPortCom->isOpen())
    {
        qDebug() << "GpsDevice::GpsDevice(): Opening serial port" << serialDeviceFileCom << "failed:" << mSerialPortCom->errorString() << "Exiting.";
        exit(1);
    }

    // initialize the device whenever we get time to do this. By doing it asynchronously, we can give our creator time to connect our signals and fetch them.
    QTimer::singleShot(0, this, SLOT(slotDetermineSerialPortsOnDevice()));
}

GpsDevice::~GpsDevice()
{
    qDebug() << "GpsDevice::~GpsDevice(): stopping output, closing serial ports";
    slotCommunicationStop();
    mSerialPortUsb->close();
    mSerialPortCom->close();
}

void GpsDevice::sendAsciiCommand(QString command)
{
    mCommandQueueUsb.append(command.append("\r\n").toAscii());

    slotFlushCommandQueue();
}

void GpsDevice::slotFlushCommandQueue()
{
    if(mNumberOfRemainingRepliesUsb == 0 && mCommandQueueUsb.size())
    {
        mLastCommandToDeviceUsb = mCommandQueueUsb.takeFirst();
        qDebug() << "GpsDevice::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDeviceUsb.trimmed();
        if(mReceiveBufferUsb.size() != 0) qDebug() << "GpsDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBufferUsb;

        mSerialPortUsb->write(mLastCommandToDeviceUsb);
        mNumberOfRemainingRepliesUsb++;
    }
    else if(mNumberOfRemainingRepliesUsb)
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingRepliesUsb << "command-replies, not sending.";
    }
    else
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): nothing to send.";
    }
}

void GpsDevice::slotDetermineSerialPortsOnDevice()
{
    // This method finds the name of the used communication-port on the GPS-device.
    // For example, we might connect using /dev/ttyUSB1, which is a usb2serial adapter
    // and connects to the devices COM2. This method will chat with the device, analyze
    // the prompt in its replies and set mSerialPortOnDevice to COM2. This can be used
    // lateron to tell the device to output useful info on COM2.
    emit stateChanged(GpsDevice::Initializing);

    // We use two connections to the board, the other one is just for feeding the RTK
    // data that we received from rtkfetcher. But we also need to know that ports name,
    // as we need to tell the receiver to accept RTK data on that port.
    if(!mSerialPortUsb->isOpen() || !mSerialPortCom->isOpen()) emit stateChanged(GpsDevice::Error);
    Q_ASSERT(mSerialPortUsb->isOpen());
    Q_ASSERT(mSerialPortCom->isOpen());

    // We just send any string causing a reply to the device, so we can see on what port it is talking to us.
//    mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
//    usleep(100000);
    mSerialPortUsb->write("setComSettings,all,baud115200,bits8,No,bit1,none\n");
    mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
    mSerialPortCom->write("getReceiverCapabilities\n");
    usleep(100000);
    QCoreApplication::processEvents();

    // After waiting for the reply, read and analyze.
    QByteArray dataUsb = mSerialPortUsb->readAll();
    QString portNameUsb = dataUsb.right(5).left(4);

    // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
    if(dataUsb.right(1) == ">" && (portNameUsb.left(3) == "COM" || portNameUsb.left(3) == "USB"))
    {
        mSerialPortOnDeviceUsb = portNameUsb;
        qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial usb port on device is now " << mSerialPortOnDeviceUsb;
//        qDebug() << "GpsDevice::determineSerialPortOnDevice(): other non-rtk data:" << dataUsb;
    }
    else
    {
        qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialUsbPortOnDevice, data is:" << dataUsb;
        emit stateChanged(GpsDevice::Error);
    }

    // After waiting for the reply, read and analyze.
    QByteArray dataCom = mSerialPortCom->readAll();
    QString portNameCom = dataCom.right(5).left(4);

    // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
    if(dataCom.right(1) == ">" && (portNameCom.left(3) == "COM" || portNameCom.left(3) == "USB"))
    {
        mSerialPortOnDeviceCom = portNameCom;
        qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial com port on device is now " << mSerialPortOnDeviceCom;
//        qDebug() << "GpsDevice::determineSerialPortOnDevice(): other non-rtk data:" << dataCom;
    }
    else
    {
        qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialComPortOnDevice, data is:" << dataCom;
        emit stateChanged(GpsDevice::Error);
    }

    // Now that we know what the ports are named, we can setup the board.
    // We connect this signal not in the c'tor but here, because we don't want the slot
    // to be called for answers to requests made in this method.
    connect(mSerialPortUsb, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));
    slotCommunicationSetup();
}

void GpsDevice::slotCommunicationSetup()
{
    Q_ASSERT(mSerialPortOnDeviceUsb.size() == 4);
    Q_ASSERT(mSerialPortOnDeviceCom.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Base-Mode

    qDebug() << "GpsDevice::setupCommunication(): setting up communication";


    // reset communications
    sendAsciiCommand("setDataInOut,all,CMD,none");

    // make the receiver output SBF blocks on our USB connection
    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceUsb+",,+SBF");

    // make the receiver listen to RTK data on specified port
    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3");

    // configure rover in standalone+rtk mode
    sendAsciiCommand("setPVTMode,Rover,StandAlone+RTK");

    // use a static (not moving) base station
    sendAsciiCommand("setDiffCorrUsage,,,,,off");

    // set up INS/GNSS integration
    sendAsciiCommand("setDataInOut,COM2,MTI");

    // when GPS fails, use IMU for how many seconds?
    sendAsciiCommand("setExtSensorUsage,COM2,,10");

    // specify in what orientation the IMU is attached relative to vehicle reference frame (x front, y right and Z down)
    sendAsciiCommand("setExtSensorCalibration,COM2,90,0,0"); // X pointing forward, Y pointing down and Z pointing left

    // specify offset between GPS antenna ARP and IMU (again, vehicle reference frame)
    sendAsciiCommand("setExtSensorCalibration,COM2,,,,,manual,0.124,-0.052,-0.01"); // IMU is 12.4cm in front, 5.2cm to the left and 1cm above ARP

    // start the integration filter
    sendAsciiCommand("setPVTMode,,,,loosely");

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    sendAsciiCommand("setEventParameters,EventA,High2Low");

    // output IntPVCart, IntAttEuler, and Event-position. ExtSensorMeas is direct IMU measurements
    // We want to know the pose 25 times a second
    sendAsciiCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntAttEuler,msec40");

    // We want to know pose whenever a scan is finished.
    sendAsciiCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",ExtEvent+IntPVAAGeod,OnChange");

    qDebug() << "GpsDevice::setupCommunication(): done setting up communication";
}

void GpsDevice::slotCommunicationStop()
{
    // For some reason, resetting this port with the SDIO command below doesn't work.
    // We need to get it to accept CMDs by sending 10 Ss to it.
    mSerialPortCom->write("SSSSSSSSSS");
    usleep(100000);
    QCoreApplication::processEvents();

    qDebug() << "GpsDevice::communicationStop(): setting datainout to all,cmd,none";

    mSerialPortUsb->write(QString("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",none\n").toAscii());
    usleep(100000);
    mSerialPortUsb->write(QString("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",none\n").toAscii());
    usleep(100000);
    mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
    usleep(100000);
    mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
    usleep(100000);

    slotFlushCommandQueue();

    emit stateChanged(GpsDevice::Stopped);

    QCoreApplication::processEvents();
}

void GpsDevice::slotSerialPortDataReady()
{
    usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBufferUsb.append(mSerialPortUsb->readAll());

//    qDebug() << "GpsDevice::slotSerialPortDataReady():" << mReceiveBufferUsb;

    if(mNumberOfRemainingRepliesUsb != 0)
    {
        const int position = mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">"));
        if(position != -1)
        {
            qDebug() << "GpsDevice::slotSerialPortDataReady(): received reply to" << mLastCommandToDeviceUsb.trimmed() << ":" << mReceiveBufferUsb.left(position).trimmed();
            qDebug() << "GpsDevice::slotSerialPortDataReady(): now sending next command, if any";

            if(mReceiveBufferUsb.left(position).contains("$R? ASCII commands between prompts were discarded!"))
                qDebug() << "GpsDevice::slotSerialPortDataReady(): it seems we were talking too fast?!";

            mReceiveBufferUsb.remove(0, position+5);
            mNumberOfRemainingRepliesUsb--;
            slotFlushCommandQueue();
        }
        else
        {
            Q_ASSERT(false && "Expected a GPS command reply from USB port, but didn't find a prompt at the end");
        }
    }
    else
    {
        // We're not waiting for a reply to a command, this must be SBF data!
//        qDebug() << "GpsDevice::slotSerialPortDataReady(): received" << mReceiveBufferUsb.size() << "bytes of SBF data.";
        processSbfData();
    }

}

quint16 GpsDevice::getCrc(const void *buf, unsigned int length)
{
  quint32  i;
  quint16  crc = 0;
  const quint8  *buf8 = (quint8*)buf; /* Convert the type to access by byte. */

  /* see for example the BINEX web site */
  for (i=0; i < length; i++)
  {
    crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buf8[i] ];
  }

  return crc;
}

void GpsDevice::processSbfData()
{
//    qDebug() << "GpsDevice::processSbfData():" << mReceiveBufferUsb.size() << "bytes present.";

    while(mReceiveBufferUsb.size() > 8)
    {
//        qDebug() << "GpsDevice::processSbfData(): more than 8 data bytes present, processing.";
        const int indexOfSyncMarker = mReceiveBufferUsb.indexOf("$@");

        if(indexOfSyncMarker != 0)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: SBF Sync Marker $@ was not at byte 0!";
            mReceiveBufferUsb.remove(0, indexOfSyncMarker);
        }

        const quint16 msgCrc = *(quint16*)(mReceiveBufferUsb.data() + 2);
        const quint16 msgId = *(quint16*)(mReceiveBufferUsb.data() + 4);
        const quint16 msgIdBlock = msgId & 0x1fff;
        const quint16 msgIdRev = msgId >> 13;
        const quint16 msgLength = *(quint16*)(mReceiveBufferUsb.data() + 6);

        if(mReceiveBufferUsb.size() < msgLength)
        {
            qDebug() << "GpsDevice::processSbfData(): message incomplete, we only have" << mReceiveBufferUsb.size() << "of" << msgLength << "bytes";
            return;
        }

        if(getCrc(mReceiveBufferUsb.data()+4, msgLength-4) != msgCrc)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: CRC in msg" << msgCrc << "computed" << getCrc(mReceiveBufferUsb.data()+4, msgLength-4) << "msgIdBlock" << msgIdBlock;
            continue;
        }

//        const quint32 timeInWeek = *(sbfMessageBody.data());
//        const quint16 timeWeekNumber = *(sbfMessageBody.data()+4);
//        printf("time in week: %8.8f\n\nv", ((float)timeInWeek)/1000.0);
//        fflush(stdout);
//        qDebug() << "GpsDevice::processSbfData(): gps time is" << timeInWeek/1000.0 << "seconds into week" << timeWeekNumber;

        // Process the message if we're interested.
        switch(msgIdBlock)
        {
        case 4006:
        {
            // PVTCartesian
            if(msgIdRev != 2)
            {
                qWarning() << "GpsDevice::processSbfData(): WARNING: invalid revision" << msgIdRev << "for block id" << msgIdBlock;
                break;
            }
            // process
            qDebug() << "SBF: PVTCartesian";
        }
            break;
        case 4043:
        {
            qDebug() << "SBF: BaseVectorCart";
        }
            break;
        case 4045:
        {
            // IntPVAAGeod
            const Sbf_PVAAGeod *block = (Sbf_PVAAGeod*)mReceiveBufferUsb.data();

//            qDebug() << "SBF: IntPVAAGeod: Info" << block->Info << "TOW" << block->TOW << "WNc" << block->WNc << "HPR:" << block->Heading << block->Pitch << block->Roll;
        }
            break;
        case 4070:
        {
            // IntAttEuler
            const Sbf_IntAttEuler *block = (Sbf_IntAttEuler*)mReceiveBufferUsb.data();
            qDebug() << "SBF: IntAttEuler: Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "TOW" << block->TOW << "WNc" << block->WNc << "HPR:" << block->Heading << block->Pitch << block->Roll;;
        }
            break;
        case 5924:
        {
            // ExtEvent
            qDebug() << "SBF: ExtEvent";
        }
            break;
        case 4037:
        {
            // ExtEventPvtCartesian
            qDebug() << "SBF: ExtEventPvtCartesian";
        }
            break;
        case 4050:
        {
            // ExtSensorMeas
            qDebug() << "SBF: ExtSensorMeas";
        }
            break;
        default:
        {
            qDebug() << "GpsDevice::processSbfData(): ignoring block id" << msgIdBlock;
        }
        }

        // Remove the SBF block body from our incoming USB buffer, so it contains either nothing or the next SBF message
        mReceiveBufferUsb.remove(0, msgLength);
    }

    if(mReceiveBufferUsb.size())
        qDebug() << "GpsDevice::processSbfData(): done processing SBF data, bytes left in buffer:" << mReceiveBufferUsb.size();

}

void GpsDevice::slotSetRtkData(const QByteArray &data)
{
    // simply write the RTK data into the com-port
    mRtkDataCounter += data.size();
    qDebug() << "GpsDevice::slotSetRtkData(): forwarding" << data.size() << "bytes of rtk-data to gps device, total is" << mRtkDataCounter;
    mSerialPortCom->write(data);
}
