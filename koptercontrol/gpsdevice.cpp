#include "gpsdevice.h"

/*
 * This is the receiver's bootup-config concerning I/O:
 * > $R: gdio
 * >   DataInOut, DSK1, CMD, SBF+NMEA, (off)
 * >   DataInOut, COM1, CMD, none, (on)
 * >   DataInOut, COM2, MTI, none, (on)
 * >   DataInOut, COM3, RTCMv3, NMEA, (on)
 * >   DataInOut, COM4, CMD, none, (on)
 * >   DataInOut, USB1, CMD, SBF+NMEA, (on)
 * >   DataInOut, USB2, CMD, SBF+NMEA, (off)
 *
 * We'd usually talk to the receiver on its COM-port to find that ports name (COM3 as above),
 * but as you can see, that port is not configured to accept CMDs, so it won't talk to us.
 * Unfortunately, it cannot be set to accept both CMD and RTCMv3. So the port is configured
 * to accept RTCMv3 and we pre-set mSerialPortOnDeviceCom to COM3 below.
 *
 * Obviously, we now rely on the bootup-config being correct, but that should work.
 *
 */

GpsDevice::GpsDevice(QString &serialDeviceFileUsb, QString &serialDeviceFileCom, QObject *parent) : QObject(parent)
{
    qDebug() << "GpsDevice::GpsDevice(): Using usb port" << serialDeviceFileUsb << "and com port" << serialDeviceFileCom;

    mDeviceIsInitialized = false;

    mTimeStampStartup = QDateTime::currentDateTime();

    mPoseClockDivisor = 0;

    mNumberOfRemainingRepliesUsb = 0; // Should never be > 1 as we wait with sending until last command is replied to.
    mRtkDataCounter = 0;
    mSerialPortOnDeviceCom = "COM3";
    mSerialPortOnDeviceUsb = "";

    mLastErrorFromDevice = 24;
    mLastModeFromDevice = 0;
    mLastInfoFromDevice = 0;
    mLastGnssPvtModeFromDevice = 0;
    mLastNumberOfSatellitesUsed = 0;
    mLastGnssAgeFromDevice = 255;

    // We use the USB port to talk to the GPS receiver and receive poses
    mSerialPortUsb = new AbstractSerial();
//    mSerialPortUsb->enableEmitStatus(true);
//    connect(mSerialPortUsb, SIGNAL(signalStatus(QString,QDateTime)), SLOT(slotSerialPortStatusChanged(QString,QDateTime)));
    mSerialPortUsb->setDeviceName(serialDeviceFileUsb);
    if(!mSerialPortUsb->open(AbstractSerial::ReadWrite))
    {
        mSerialPortUsb->close();
        qFatal("GpsDevice::GpsDevice(): Opening serial usb port %s failed, exiting.", qPrintable(serialDeviceFileUsb));
    }
    mSerialPortUsb->setBaudRate(AbstractSerial::BaudRate115200);
    mSerialPortUsb->setDataBits(AbstractSerial::DataBits8);
    mSerialPortUsb->setParity(AbstractSerial::ParityNone);
    mSerialPortUsb->setStopBits(AbstractSerial::StopBits1);
    mSerialPortUsb->setFlowControl(AbstractSerial::FlowControlOff);

    // We use the COM port to feed the device RTK correction data
    mSerialPortCom = new AbstractSerial();
//    mSerialPortCom->enableEmitStatus(true);
//    connect(mSerialPortCom, SIGNAL(signalStatus(QString,QDateTime)), SLOT(slotSerialPortStatusChanged(QString,QDateTime)));
    mSerialPortCom->setDeviceName(serialDeviceFileCom);
    if(!mSerialPortCom->open(AbstractSerial::ReadWrite))
    {
        mSerialPortCom->close();
        qFatal("GpsDevice::GpsDevice(): Opening serial com port %s failed, exiting.", qPrintable(serialDeviceFileCom));
    }
    mSerialPortCom->setBaudRate(AbstractSerial::BaudRate460800);
    mSerialPortCom->setDataBits(AbstractSerial::DataBits8);
    mSerialPortCom->setParity(AbstractSerial::ParityNone);
    mSerialPortCom->setStopBits(AbstractSerial::StopBits1);
    mSerialPortCom->setFlowControl(AbstractSerial::FlowControlOff);
    connect(mSerialPortCom, SIGNAL(readyRead()), SLOT(slotDataReadyOnCom()));

    mStatusTimer = new QTimer(this);
    mStatusTimer->setInterval(1000);
    connect(mStatusTimer, SIGNAL(timeout()), SLOT(slotEmitCurrentGpsStatus()));

    // initialize the device whenever we get time to do this. By doing it asynchronously, we can give our creator time to connect our signals and fetch them.
    QTimer::singleShot(0, this, SLOT(slotDetermineSerialPortsOnDevice()));
}

GpsDevice::~GpsDevice()
{
    mSerialPortUsb->close();
    mSerialPortCom->close();

    qDebug() << "GpsDevice::~GpsDevice(): ports closed, destructed.";
}

void GpsDevice::queueAsciiCommand(QString command)
{
    qDebug() << "GpsDevice::queueAsciiCommand(): queueing command" << command;
    mCommandQueueUsb.append(command.append("\r\n").toAscii());
    slotFlushCommandQueue();
}

quint8 GpsDevice::slotFlushCommandQueue()
{
    if(mNumberOfRemainingRepliesUsb == 0 && mCommandQueueUsb.size())
    {
        mLastCommandToDeviceUsb = mCommandQueueUsb.takeFirst();
        qDebug() << t() << "GpsDevice::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDeviceUsb.trimmed();
        if(mReceiveBufferUsb.size()) qDebug() << t() << "GpsDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBufferUsb;
        usleep(100000);
        mSerialPortUsb->write(mLastCommandToDeviceUsb);
        mNumberOfRemainingRepliesUsb++;
    }
    else if(mNumberOfRemainingRepliesUsb)
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingRepliesUsb << "usb command-replies, not sending.";
    }
    else
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): nothing to send.";
    }

    return mCommandQueueUsb.size();
}

void GpsDevice::slotDetermineSerialPortsOnDevice()
{
    // This method finds the name of the used communication-port on the GPS-device.
    // For example, we might connect using /dev/ttyUSB1, which is a usb2serial adapter
    // and connects to the devices COM2. This method will chat with the device, analyze
    // the prompt in its replies and set mSerialPortOnDevice to COM2. This can be used
    // lateron to tell the device to output useful info on COM2.
    slotEmitCurrentGpsStatus("Setting up communication");

    // We use two connections to the board, the other one is just for feeding the RTK
    // data that we received from rtkfetcher. But we also need to know that ports name,
    // as we need to tell the receiver to accept RTK data on that port.
    if(!mSerialPortUsb->isOpen() || !mSerialPortCom->isOpen())
    {
        slotEmitCurrentGpsStatus("Cannot open GPS serial port(s)");
    }

    Q_ASSERT(mSerialPortUsb->isOpen());
    Q_ASSERT(mSerialPortCom->isOpen());

    // We just send any string causing a reply to the device, so we can see on what port it is talking to us.
    if(mSerialPortOnDeviceUsb.isEmpty())
    {
        mSerialPortUsb->write("getReceiverCapabilities\r\n");

        QByteArray dataUsb;
        while((mSerialPortUsb->bytesAvailable() + dataUsb.size()) < 250)
        {
            mSerialPortUsb->waitForReadyRead(1000);
            dataUsb.append(mSerialPortUsb->readAll());
            qDebug() << "GpsDevice::determineSerialPortOnDevice():" << mSerialPortUsb->bytesAvailable() << "bytes waiting on serial usb port.";
        }

        // After waiting for the reply, read and analyze.
        QString portNameUsb = dataUsb.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataUsb.right(1) == ">" && (portNameUsb.left(3) == "COM" || portNameUsb.left(3) == "USB"))
        {
            mSerialPortOnDeviceUsb = portNameUsb;
            qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial usb port on device is now " << mSerialPortOnDeviceUsb;
        }
        else
        {
            qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialUsbPortOnDevice," << dataUsb.size() << "bytes of data are:" << dataUsb.simplified();
            slotEmitCurrentGpsStatus("Couldn't get serialUsbPortOnDevice");
        }
    }

    if(mSerialPortOnDeviceCom.isEmpty())
    {
        mSerialPortCom->write("getReceiverCapabilities\r\n");

        QByteArray dataCom;
        while((mSerialPortCom->bytesAvailable() + dataCom.size()) < 250)
        {
            mSerialPortCom->waitForReadyRead(1000);
            dataCom.append(mSerialPortCom->readAll());
            qDebug() << "GpsDevice::determineSerialPortOnDevice():" << mSerialPortCom->bytesAvailable() << "bytes waiting on serial com port.";
        }

        // After waiting for the reply, read and analyze.
        QString portNameCom = dataCom.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataCom.right(1) == ">" && (portNameCom.left(3) == "COM" || portNameCom.left(3) == "USB"))
        {
            mSerialPortOnDeviceCom = portNameCom;
            qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial com port on device is now " << mSerialPortOnDeviceCom;
        }
        else
        {
            qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialComPortOnDevice, data is:" << dataCom;
            slotEmitCurrentGpsStatus("Couldn't get serialComPortOnDevice");
        }
    }

    // Now that we know what the ports are named, we can setup the board.
    // We connect this signal not in the c'tor but here, because we don't want the slot
    // to be called for answers to requests made in this method.
    connect(mSerialPortUsb, SIGNAL(readyRead()), SLOT(slotDataReadyOnUsb()));
    slotCommunicationSetup();
}

void GpsDevice::slotCommunicationSetup()
{
    Q_ASSERT(mSerialPortOnDeviceUsb.size() == 4);
    Q_ASSERT(mSerialPortOnDeviceCom.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Rover-Mode

    qDebug() << "GpsDevice::setupCommunication(): setting up communication";

    // use bootup config for now

    // show current config
    queueAsciiCommand("lstConfigFile,Current");

    // reset communications
    queueAsciiCommand("setDataInOut,all,CMD,none");

    // increase com-port speed to 460800 for shorter latency
    queueAsciiCommand("setComSettings,"+mSerialPortOnDeviceCom+",baud460800,bits8,No,bit1,none");

    // make the receiver output SBF blocks on both COM and USB connections
    queueAsciiCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3,SBF");
    queueAsciiCommand("setDataInOut,"+mSerialPortOnDeviceUsb+",CMD,SBF");

    // make the receiver listen to RTK data on specified port
//    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3");

    // we want to know the TOW, because we don't want it to roll over! Answer is parsed below and used to sync time.
    queueAsciiCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");
    queueAsciiCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");

    queueAsciiCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");
    queueAsciiCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");

    // use a static (not moving) base station
    queueAsciiCommand("setDiffCorrUsage,LowLatency,20.0,auto,0,off");

    // xPPSOut is connected to DCD line of mSerialPortCom and uses hardpps() to sync clocks.
    // 0.00 is delay in nanoseconds, higher values cause PPS signal to be generated earlier.
    // 3600 is MaxSyncAge - time with no PVT after which PPS output is stopped. Ignored for RxClock timing system.
    queueAsciiCommand("setPPSParameters,sec1,Low2High,0.00,RxClock,3600");

    // set up INS/GNSS integration
    queueAsciiCommand("setDataInOut,COM2,MTI");

    // set time system to be GPS, not Galileo
    queueAsciiCommand("setTimingSystem,GPS");

    // see Firmware User manual pg. 32: drift is so slow we won't ever touch this
    queueAsciiCommand("setClockSyncThreshold,usec500");

    // when GPS fails, use IMU for how many seconds?
    queueAsciiCommand("setExtSensorUsage,COM2,Accelerations+AngularRates,10");

    // specify vector from GPS antenna ARP to IMU in Vehicle reference frame
    // (vehicle reference frame has X forward, Y right and Z down)
    // IMU is 7cm in front, 6.7cm to the right and 33cm below ARP. Max precision is 1 cm.
    // Specifying orientation is not so easy (=fucking mess, Firmware User manual pg. 41)
    //sendAsciiCommand("setExtSensorCalibration,COM2,manual,0,90,270,manual,0.07,0.07,0.33");
    //sendAsciiCommand("setExtSensorCalibration,COM2,manual,-90,0,270,manual,0.07,0.07,0.33");
    //sendAsciiCommand("setExtSensorCalibration,COM2,manual,0,90,90,manual,0.07,0.07,0.33");
    // Leicht, jetzt wo IMU in der Mitte liegt und unter dem kopter hängt
    queueAsciiCommand("setExtSensorCalibration,COM2,manual,180,00,0,manual,-0.06,0.10,0.26");
    // Sarah Dean says "seem to be ok" about 0 90 270
    //sendAsciiCommand("setExtSensorCalibration,COM2,manual,0,90,270,manual,0.07,0.07,0.33");

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    queueAsciiCommand("setEventParameters,EventA,High2Low");
    queueAsciiCommand("setEventParameters,EventB,High2Low");

    // configure rover in standalone+rtk mode (use "RTKFixed" instead of "all"?)
    queueAsciiCommand("setPVTMode,Rover,all,auto,Loosely");

    // explicitly allow rover to use all RTCMv3 corection messages
    queueAsciiCommand("setRTCMv3Usage,all");

    // output IntPVCart, IntAttEuler, and Event-position. ExtSensorMeas is direct IMU measurements
    // We want to know the pose 25 times a second
    queueAsciiCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec50");

    // We want to know PVTCartesion (4006) for MeanCorrAge (average correction data age) only, so stream it slowly
    queueAsciiCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",PVTCartesian+ReceiverStatus,sec1");

    // We want to know whenever a scan is finished.
    queueAsciiCommand("setSBFOutput,Stream3,"+mSerialPortOnDeviceCom+",ExtEvent,OnChange");

    // We want to know what time it is
    queueAsciiCommand("setSBFOutput,Stream4,"+mSerialPortOnDeviceCom+",ReceiverTime,sec30");

    qDebug() << "GpsDevice::setupCommunication(): done setting up communication";

    // emit status signal periodically.
    mStatusTimer->start();
}

void GpsDevice::slotShutDown()
{
    qDebug() << "GpsDevice::slotShutDown(): starting shutdown sequence after" << mTimeStampStartup.secsTo(QDateTime::currentDateTime()) << "seconds runtime: disabling SBF streams, syncing clock, resetting dataInOut...";
    mDeviceIsInitialized = false;

    queueAsciiCommand("setComSettings,all,baud115200,bits8,No,bit1,none");
    queueAsciiCommand("setSBFOutput,all,"+mSerialPortOnDeviceUsb+",none");
    queueAsciiCommand("setSBFOutput,all,"+mSerialPortOnDeviceCom+",none");
    queueAsciiCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");
    queueAsciiCommand("setDataInOut,all,CMD,none");
    queueAsciiCommand("setPVTMode,Rover,StandAlone");
    queueAsciiCommand("shutdown");

    // no need to update status of a disabled device
    mStatusTimer->stop();
    qDebug() << "GpsDevice::slotShutDown(): shutdown sequence processed, waiting for asynchronous shutdown confirmation...";
}

void GpsDevice::slotDataReadyOnCom()
{
        mReceiveBufferCom.append(mSerialPortCom->readAll());
//        qDebug() << "GpsDevice::slotDataReadyOnCom(): size of SBF data is now" << mReceiveBufferCom.size() << "bytes, processing:" << mReceiveBufferCom;
        processSbfData(mReceiveBufferCom);
}

void GpsDevice::slotDataReadyOnUsb()
{
//    qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb()";
    mReceiveBufferUsb.append(mSerialPortUsb->readAll());

    if(mNumberOfRemainingRepliesUsb != 0)
    {
        // If the receiver replies to "exeSbfOnce" commands, process that data immediately. It might be receivertime, which is time-crucial.
        if(mReceiveBufferUsb.left(2) == QString("$@").toAscii())
        {
            qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): device replied to a request with SBF, processing" << mReceiveBufferUsb.size() << "SBF bytes.";
            Q_ASSERT(mReceiveBufferUsb.indexOf("$@") == 0);
            processSbfData(mReceiveBufferUsb);

            // Its quite possible that AFTER the SBF-data, there was the exeSBFOnce command reply. If so, this reply
            // is still in the buffer and needs to be processed. Luckily, this is done in the while-loop below.
        }

        while(mNumberOfRemainingRepliesUsb != 0 && mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">")) != -1) // while we found a complete chat reply
        {
            const int positionEndOfReply = mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">")) + 5;
            qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): received reply to:" << mLastCommandToDeviceUsb.trimmed() << ":" << mReceiveBufferUsb.size() << "bytes:" << mReceiveBufferUsb.left(positionEndOfReply).trimmed();

            // After sending/receiving the SetPvtMode command, the rover needs to be static for better alignment. Tell the user to wait!
            if(QString(mReceiveBufferUsb.left(positionEndOfReply)).contains("SetPvtMode", Qt::CaseInsensitive))
                slotEmitCurrentGpsStatus("Integration filter started (alignment not ready), vehicle must remain static for 20s starting now.");

            if(mReceiveBufferUsb.left(positionEndOfReply).contains("$R? ASCII commands between prompts were discarded!"))
                qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): we were talking too fast!!";

            if(mReceiveBufferUsb.left(positionEndOfReply).contains("shutdown"))
            {
                slotEmitCurrentGpsStatus("Orderly shutdown finished");
                qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): shutdown confirmed by device, quitting.";
                QCoreApplication::quit();
            }

            mReceiveBufferUsb.remove(0, positionEndOfReply);
            mNumberOfRemainingRepliesUsb--;
        }

        if(mReceiveBufferUsb.size())
        {
            qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): after parsing all replies, rx-buffer not empty, contains:" << mReceiveBufferUsb;
        }
        else
        {
//            qDebug() << "GpsDevice::slotDataReadyOnUsb(): after parsing all input i will send next command, if any";
            slotFlushCommandQueue();
        }
    }
    else
    {
        // We're receiving from the device, and it is not a reply to some request we sent ourselves. Thus, the device is
        // talking to us on its own, which only happens after initializing it
        mDeviceIsInitialized = true;
        // We're not waiting for a reply to a command, this must be SBF data!
//        qDebug() << "GpsDevice::slotDataReadyOnUsb(): received" << mReceiveBufferUsb.size() << "bytes of SBF data, processing...";
        processSbfData(mReceiveBufferUsb);
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

void GpsDevice::processSbfData(QByteArray& receiveBuffer)
{
    //qDebug() << "GpsDevice::processSbfData():" << receiveBuffer.size() << "bytes present.";

    while(receiveBuffer.size() > 8)
    {
//        qDebug() << "GpsDevice::processSbfData(): more than 8 data bytes present, processing.";
        const int indexOfSyncMarker = receiveBuffer.indexOf("$@");

	if(indexOfSyncMarker == -1)
	{
	    // The sync marker wasn't found! This means the whole buffer contains unusable data,
	    // because we cannot use any data without a sync-marker prepended. So the data must
	    // be non-SBF and should be consumed by someone else.
	    return;

            // This may happen when we send a exeSbfOnce command (e.g. for ReceiverTime). The
            // receiver then replies with the SBF and then the prompt. The prompt is processed
            // here and discarded. Thats ok.
            //qWarning() << "GpsDevice::processSbfData(): WARNING: SBF Sync marker not found in buffer of" << receiveBuffer.size() << "bytes. Clearing buffer:" << receiveBuffer;
            //receiveBuffer.clear();
            //return;
        }
        else if(indexOfSyncMarker != 0)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: SBF Sync Marker $@ was not at byte 0, but at" << indexOfSyncMarker;
            receiveBuffer.remove(0, indexOfSyncMarker);
        }

        const quint16 msgCrc = *(quint16*)(receiveBuffer.data() + 2);
        const quint16 msgId = *(quint16*)(receiveBuffer.data() + 4);
        const quint16 msgIdBlock = msgId & 0x1fff;
        const quint16 msgIdRev = msgId >> 13;
        const quint16 msgLength = *(quint16*)(receiveBuffer.data() + 6);

        if(receiveBuffer.size() < msgLength)
        {
//            qDebug() << t() << "GpsDevice::processSbfData(): message incomplete, we only have" << receiveBuffer.size() << "of" << msgLength << "bytes. Processing postponed..";
            return;
        }

        if(getCrc(receiveBuffer.data()+4, msgLength-4) != msgCrc)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: CRC in msg" << msgCrc << "computed" << getCrc(receiveBuffer.data()+4, msgLength-4) << "msgIdBlock" << msgIdBlock;
            // Remove the SBF block body from our incoming USB buffer, so it contains either nothing or the next SBF message
            // Since the CRC is wrong, msgLength might also be off. Thus we delete just two bytes at the beginning, causing
            // a warning about spurious data in the next processing iteration, but thats still more safe.
            receiveBuffer.remove(0, 2);
            continue;
        }

//        const quint32 timeInWeek = *(sbfMessageBody.data());
//        const quint16 timeWeekNumber = *(sbfMessageBody.data()+4);
//        printf("time in week: %8.8f\n\nv", ((float)timeInWeek)/1000.0);
//        fflush(stdout);
//        qDebug() << "GpsDevice::processSbfData(): gps time is" << timeInWeek/1000.0 << "seconds into week" << timeWeekNumber;

        // Process the message if we're interested.
        //qDebug() << "received sbf block" << msgIdBlock;
        switch(msgIdBlock)
        {

        case 4006:
        {
            // PVTCartesian
            const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)receiveBuffer.data();
            mLastMeanCorrAge = std::min(block->MeanCorrAge / 10.0, 255.0);
            //qDebug() << "SBF: PVTCartesian: MeanCorrAge in seconds:" << ((float)block->MeanCorrAge)/100.0;
        }
        break;

        case 4014:
        {
            // ReceiverStatus
            const Sbf_ReceiverStatus *block = (Sbf_ReceiverStatus*)receiveBuffer.data();
            if(block->CPULoad > 80)
            {
                qWarning() << "GpsDevice::processSbfData(): WARNING, receiver CPU load is" << block->CPULoad;
                slotEmitCurrentGpsStatus(QString("Warning, CPU load is too high (%1%)").arg(block->CPULoad));
            }

            if(block->ExtError != 0)
            {
                qWarning() << "GpsDevice::processSbfData(): ExtError is not 0 but" << block->ExtError;
                slotEmitCurrentGpsStatus(QString("Warning, ExtError is not zero (%1)").arg(block->ExtError));
            }

            if(block->RxError != 0)
            {
                qWarning() << "GpsDevice::processSbfData(): RxError is not 0 but" << block->RxError;
                slotEmitCurrentGpsStatus(QString("Warning, RxError is not zero (%1)").arg(block->RxError));

                // According to SBF guide pg. 102, this means software warning or error. Lets see what it is.
                if(block->RxError == 8) queueAsciiCommand("lif, error");
            }
        }
        break;

        case 4045:
        {
            // IntPVAAGeod
//            qDebug() << "SBF: PVAAGeod";
            const Sbf_PVAAGeod *block = (Sbf_PVAAGeod*)receiveBuffer.data();

            // Check the Info-field and emit states if it changes
            if(mLastInfoFromDevice != block->Info)
            {
//                qDebug() << t() << "GpsDevice::processSbfData(): info changed from" << mLastInfoFromDevice << "to" << block->Info;
                const quint16 previousInfoFromDevice = mLastInfoFromDevice;
                mLastInfoFromDevice = block->Info;

                if(!testBitEqual(previousInfoFromDevice, block->Info, 0))
                    slotEmitCurrentGpsStatus(QString("ACLR measurements used: %1").arg(testBit(block->Info, 0) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 1))
                    slotEmitCurrentGpsStatus(QString("GYRO measurements used: %1").arg(testBit(block->Info, 1) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 11))
                    slotEmitCurrentGpsStatus(QString("Heading ambiguity fixed: %1").arg(testBit(block->Info, 11) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 12))
                    slotEmitCurrentGpsStatus(QString("Zero constraint used: %1").arg(testBit(block->Info, 12) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 13))
                    slotEmitCurrentGpsStatus(QString("GNSS position used: %1").arg(testBit(block->Info, 13) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 14))
                    slotEmitCurrentGpsStatus(QString("GNSS velocity used: %1").arg(testBit(block->Info, 14) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 15))
                    slotEmitCurrentGpsStatus(QString("GNSS attitude used: %1").arg(testBit(block->Info, 15) ? "true" : "false"));

                // If mode was 26627 or 30723, that would be 11X100000000011
                if(block->Info == 26627 || block->Info == 30723) Q_ASSERT("Whee, GPS INFO is the way it should be!");
            }

            // Check the Mode-field and emit states if it changes
            if(mLastModeFromDevice != block->Mode)
            {
  //              qDebug() << t() << "GpsDevice::processSbfData(): mode changed from" << mLastModeFromDevice << "to" << block->Mode;
                mLastModeFromDevice = block->Mode;

                switch(block->Mode)
                {
                case 0:
                    slotEmitCurrentGpsStatus("Mode changed, no integrated solution available");
                    break;

                case 1:
                    slotEmitCurrentGpsStatus("Mode changed, using only external sensor");
                    break;

                case 2:
                    slotEmitCurrentGpsStatus("Mode changed, using integrated solution");
                    break;

                default:
                    qWarning() << "GpsDevice::processSbfData(): WARNING: unknown mode code" << block->Mode;
                    slotEmitCurrentGpsStatus(QString("Unknown Mode %1").arg(block->Mode));
                    break;
                }
            }

            // Check the Error-field and emit states if it changes
            if(mLastErrorFromDevice != block->Error)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): error changed from" << mLastErrorFromDevice << "to" << block->Error;
                mLastErrorFromDevice = block->Error;
                slotEmitCurrentGpsStatus(GpsStatusInformation::getError(mLastErrorFromDevice));
            }

            // Check the GnssPvtMode-field and emit states if it changes
            if(mLastGnssPvtModeFromDevice != block->GNSSPVTMode)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): GnssPvtMode changed from" << mLastGnssPvtModeFromDevice << "to" << block->GNSSPVTMode;
                mLastGnssPvtModeFromDevice = block->GNSSPVTMode;
                //Q_ASSERT(mLastGnssPvtModeFromDevice != 4 && "Finally, we have fixed RTK!");
                slotEmitCurrentGpsStatus(GpsStatusInformation::getGnssMode(mLastGnssPvtModeFromDevice));
            }

            // TODO: this might change often in regular usage, really notify?
            if(mLastGnssAgeFromDevice != block->GNSSage)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): GnssAge changed from" << mLastGnssAgeFromDevice << "to" << block->GNSSage;
                mLastGnssAgeFromDevice = block->GNSSage;
//                slotEmitCurrentGpsStatus(QString("No GNSS-PVT for %1 seconds").arg(block->GNSSage));
            }

            const quint8 numberOfSatellitesUsed = (block->NrSVAnt & 31);
            if(numberOfSatellitesUsed != mLastNumberOfSatellitesUsed)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): numSats changed from" << mLastNumberOfSatellitesUsed << "to" << numberOfSatellitesUsed;
                mLastNumberOfSatellitesUsed = numberOfSatellitesUsed;
                slotEmitCurrentGpsStatus(QString("Number of used satellites changed to %1").arg(numberOfSatellitesUsed));
            }

            // Only emit a pose if the values are not set to the do-not-use values.
            if(
                    block->Error == 0
                    && block->Lat != -2147483648
                    && block->Lon != -2147483648
                    && block->Alt != -2147483648
                    && block->Heading != 65535 // This is not to the erroneous (off-by-one) spec (SBF Ref Guide, p. 80).
                    && block->Pitch != -32768
                    && block->Roll != -32768
                    && block->TOW != 4294967295
                    )
            {
                // TODO: we COULD read the sub-cm part, too...
                const float  lon = ((float)block->Lon) / 10000000.0f;
                const float  lat = ((float)block->Lat) / 10000000.0f;
                const float  alt = ((float)block->Alt) / 1000.0f;

                Pose p(
                            convertGeodeticToCartesian(lon, lat, alt),
                            ((float)block->Heading) * 0.01f,
                            ((float)block->Pitch) * 0.01f,
                            ((float)block->Roll) * 0.01f,
                            block->TOW // Receiver time in milliseconds. WARNING: be afraid of WNc rollovers at runtime!
                            );

                qDebug() << "GpsDevice::processSbfData(): new" << p;

                emit newVehiclePose(p);

                mPoseClockDivisor++;
                if(mPoseClockDivisor % 20 == 0) emit newVehiclePoseLowFreq(p);

            }
            else if(block->Error != 0)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): pose from PVAAGeod not valid, error:" << block->Error << " " << GpsStatusInformation::getError(block->Error) ;
            }
            else if(block->Heading == 65535)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): pose from PVAAGeod not valid, heading do-not-use";
            }
            else
            {
                qDebug() << t() << "GpsDevice::processSbfData(): pose from PVAAGeod not valid, do-not-use values found.";
            }

//            qDebug() << "SBF: IntAttEuler: Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "TOW" << block->TOW << "WNc" << block->WNc << "HPR:" << block->Heading << block->Pitch << block->Roll;;
//            qDebug() << "Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "HPR:" << block->Heading << block->Pitch << block->Roll;;
        }
            break;

        case 5914:
        {
            // ReceiverTime
            const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)receiveBuffer.data();

//            qDebug() << t() << "GpsDevice::processSbfData(): received ReceiverTime block: msgid" << msgId << "msgIdBlock" << msgIdBlock << "msgLength" << msgLength << "revision" << msgIdRev;

            if(block->TOW == 4294967295)
            {
                emit message(Error, "GpsDevice::processSbfData()", "GPS Receiver TOW is at its do-not-use-value, give it time to initialize.");
                qWarning() << "GpsDevice::processSbfData(): GPS Receiver TOW is at its do-not-use-value, give it time to initialize.";

                if(mTimeStampStartup.secsTo(QDateTime::currentDateTime()) < 15)
                {
                    qWarning() << "GpsDevice::processSbfData(): GPS Receiver TOW is at its do-not-use-value during startup - quitting.";
                    QCoreApplication::quit();
                }
            }
            else
            {
                // Set system time to gps time. Adding a roundtrip-timer is not a good idea, as the board waits until the
                // second leaps, meaning the time from request to output doesn't equal the time from output to reception.

                // First, what time is it now?
                struct timeval system;
                gettimeofday(&system, NULL);

                // SecondsPerWeek - CurrentSecondInWeek is number of seconds till rollover
                const quint32 secondsToRollOver = (7 * 86400) - (block->TOW / 1000);

                // Apply 15000ms = 15 leapseconds offset? Only when we really sync to UTC, which we don't.
                const qint32 offsetHostToGps = block->TOW - getCurrentGpsTowTime() + 7; // Oscilloscope indicates 7ms offset is a good value.
                qDebug() << "GpsDevice::processSbfData(): time rollover in" << ((float)secondsToRollOver)/86400.0 << "d, offset host time" << getCurrentGpsTowTime() << "to gps time" << block->TOW << "is" << offsetHostToGps/1000 << "s and" << (offsetHostToGps%1000) << "ms";

                // For small clock drifts AND when system is running, adjust clock. Else, set clock
                if(abs(offsetHostToGps) < 10 && mDeviceIsInitialized)
                {
                    qDebug() << "GpsDevice::processSbfData(): offset smaller than 10ms, using adjtime to correct clock drift...";
                    system.tv_sec = 0;
                    system.tv_usec = offsetHostToGps * 1000;
                    if(adjtime(&system, NULL) < 0)
                    {
                        if(errno == EINVAL) qDebug("GpsDevice::processSbfData(): couldn't adjust system time, values invalid.");
                        else if(errno == EPERM) qDebug("GpsDevice::processSbfData(): couldn't adjust system time, insufficient permissions.");
                        else qDebug("GpsDevice::processSbfData(): couldn't adjust system time, no idea why, error %d.", errno);
                    }
                }
                else
                {
                    qDebug() << "GpsDevice::processSbfData(): offset larger than 10ms, using settimeofday to set clock...";
                    system.tv_sec += offsetHostToGps/1000;
                    system.tv_usec += (offsetHostToGps%1000)*1000;

                    // usec can under/overflow; fix it
                    if(system.tv_usec > 1000000)
                    {
                    system.tv_usec -= 1000000;
                    system.tv_sec += 1;
                    }

                    if(settimeofday(&system, NULL) < 0)
                    {
                        if(errno == EFAULT) qDebug("GpsDevice::processSbfData(): couldn't set system time, values outside of range.");
                        else if(errno == EINVAL) qDebug("GpsDevice::processSbfData(): couldn't set system time, values invalid.");
                        else if(errno == EPERM) qDebug("GpsDevice::processSbfData(): couldn't set system time, insufficient permissions.");
                        else qDebug("GpsDevice::processSbfData(): couldn't set system time, no idea why, error %d.", errno);
                    }
                }

                emit gpsTimeOfWeekEstablished(block->TOW);

                qDebug() << "GpsDevice::processSbfData(): after" << mTimeStampStartup.secsTo(QDateTime::currentDateTime()) << "seconds runtime, offset host to gps is" << offsetHostToGps << "ms or" << offsetHostToGps / (mTimeStampStartup.secsTo(QDateTime::currentDateTime()) / 3600.0) << "ms per hour.";
            }
        }
        break;

        case 5924:
        {
            // ExtEvent
            //qDebug() << "SBF: ExtEvent";
            const Sbf_ExtEvent *block = (Sbf_ExtEvent*)receiveBuffer.data();

            // Laserscanner sync signal is soldered to both ports, but port 1 is broken. If it ever starts working again, I want to know.
            Q_ASSERT(block->Source == 2);

	    if(block->TOW != 4294967295)
	    {
//                qDebug() << "sys" << getCurrentGpsTowTime() << "gps" << block->TOW << "gps: SBF of" << sizeof(Sbf_ExtEvent) << "in" << receiveBuffer.size() << "bytes tells us scan finished";
		emit scanFinished(block->TOW);
	    }
	    else
		qDebug() << "GpsDevice::processSbfData(): WARNING: scan finished, but TOW is set to do-not-use!";
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
        receiveBuffer.remove(0, msgLength);
    }

//    if(receiveBuffer.size()) qDebug() << "GpsDevice::processSbfData(): done processing SBF data, bytes left in buffer:" << receiveBuffer.size() << "bytes:" << receiveBuffer;

}

void GpsDevice::slotSetRtkData(const QByteArray &data)
{
    if(mDeviceIsInitialized)
    {
        // simply write the RTK data into the com-port
        mRtkDataCounter += data.size();
        //qDebug() << "GpsDevice::slotSetRtkData(): forwarding" << data.size() << "bytes of rtk-data to gps device, total is" << mRtkDataCounter;
        mSerialPortCom->write(data);
        emit message(
                Information,
                "GpsDevice::slotSetRtkData()",
                QString("Fed %1 bytes of RTK data into rover gps device.").arg(data.size())
                );
    }
    else
    {
        qDebug() << "GpsDevice::slotSetRtkData(): NOT forwarding" << data.size() << "bytes of rtk-data to gps device, its not initialized yet.";
    }
}

void GpsDevice::slotEmitCurrentGpsStatus(const QString& text)
{
    emit gpsStatus(mLastGnssPvtModeFromDevice, mLastModeFromDevice, mLastInfoFromDevice, mLastErrorFromDevice, mLastNumberOfSatellitesUsed, mLastGnssAgeFromDevice, mLastMeanCorrAge, text);
    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("GnssMode %1, IntMode %2, Info %3, Error %4, NumSats %5, GnssAge %6, MeanCorrAge %8").arg(mLastGnssPvtModeFromDevice).arg(mLastModeFromDevice).arg(mLastInfoFromDevice).arg(mLastErrorFromDevice).arg(mLastNumberOfSatellitesUsed).arg(mLastGnssAgeFromDevice).arg(mLastMeanCorrAge)
                );
}

QVector3D GpsDevice::convertGeodeticToCartesian(const double &lon, const double &lat, const float &elevation)
{
    QVector3D co;
    co.setY(elevation);
    co.setZ(-(lat - 53.600669) * 111300.0);
    co.setX((lon - 9.933817l) * 111300.0l * cos(M_PI / 180.0 * 53.600669l));

    return co;
}

void GpsDevice::slotSerialPortStatusChanged(const QString& status, const QDateTime& time)
{
    if(sender() == mSerialPortUsb)
    {
        qDebug() << "GpsDevice::slotSerialPortStatusChanged(): usb port status" << status << "errorstring" << mSerialPortUsb->errorString();
    }
    else if(sender() == mSerialPortCom)
    {
        qDebug() << "GpsDevice::slotSerialPortStatusChanged(): com port status" << status << "errorstring" << mSerialPortCom->errorString();
    }
    else
    {
        qDebug() << "GpsDevice::slotSerialPortStatusChanged(): ??? port status" << status;
    }
}
