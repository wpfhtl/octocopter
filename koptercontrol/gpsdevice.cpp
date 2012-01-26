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

    mLogFileSbf = new QFile(QString("log-%1-%2-gnss.sbf").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
    if(!mLogFileSbf->open(QIODevice::WriteOnly)) qFatal("GpsDevice::GpsDevice(): couldn't open sbf log file for writing, exiting");

    mLogFileCmd = new QFile(QString("log-%1-%2-cmd.txt").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
    if(!mLogFileCmd->open(QIODevice::WriteOnly)) qFatal("GpsDevice::GpsDevice(): couldn't open cmd log file for writing, exiting");

//    mLogStreamSbf = QDataStream(mLogFileSbf);

    mSbfParser = new SbfParser;
    // If our SBF parser has a question to the device, forward it.
    connect(mSbfParser, SIGNAL(receiverCommand(QString)), SLOT(slotQueueCommand(QString)));
    connect(mSbfParser, SIGNAL(gpsTimeOfWeekEstablished(quint32)), SLOT(slotSetSystemTime(qint32)));

    mDeviceIsInitialized = false;


    mNumberOfRemainingRepliesUsb = 0; // Should never be > 1 as we wait with sending until last command is replied to.
    mRtkDataCounter = 0;
    mSerialPortOnDeviceCom = "COM3";
    mSerialPortOnDeviceUsb = "";

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
    mSerialPortCom->setBaudRate(AbstractSerial::BaudRate115200);
    mSerialPortCom->setDataBits(AbstractSerial::DataBits8);
    mSerialPortCom->setParity(AbstractSerial::ParityNone);
    mSerialPortCom->setStopBits(AbstractSerial::StopBits1);
    mSerialPortCom->setFlowControl(AbstractSerial::FlowControlOff);
    connect(mSerialPortCom, SIGNAL(readyRead()), SLOT(slotDataReadyOnCom()));


    // initialize the device whenever we get time to do this. By doing it asynchronously, we can give our creator time to connect our signals and fetch them.
    QTimer::singleShot(0, this, SLOT(slotDetermineSerialPortsOnDevice()));
}

GpsDevice::~GpsDevice()
{
    mSerialPortUsb->close();
    mSerialPortCom->close();

    mLogFileSbf->flush();
    mLogFileSbf->close();

    qDebug() << "GpsDevice::~GpsDevice(): ports closed, SBF file flushed, destructed.";
}

void GpsDevice::slotQueueCommand(QString command)
{
    qDebug() << "GpsDevice::slotQueueAsciiCommand(): queueing command" << command;
    command.replace("#USB#", mSerialPortOnDeviceUsb);
    command.replace("#COM#", mSerialPortOnDeviceCom);
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

        QTextStream commandLog(mLogFileCmd);
        commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") << " HOST -> DEV: " << mLastCommandToDeviceUsb << endl;
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
    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Setting up communication");

    // We use two connections to the board, the other one is just for feeding the RTK
    // data that we received from rtkfetcher. But we also need to know that ports name,
    // as we need to tell the receiver to accept RTK data on that port.
    if(!mSerialPortUsb->isOpen() || !mSerialPortCom->isOpen())
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Cannot open GPS serial port(s)");
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
            emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Couldn't get serialUsbPortOnDevice");
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
            emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Couldn't get serialComPortOnDevice");
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
    slotQueueCommand("lstConfigFile,Current");

    // reset communications
    slotQueueCommand("setDataInOut,all,CMD,none");

    // increase com-port speed to 460800 for shorter latency
    slotQueueCommand("setComSettings,"+mSerialPortOnDeviceCom+",baud115200,bits8,No,bit1,none");

    // make the receiver output SBF blocks on both COM and USB connections
    slotQueueCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3,SBF");
    slotQueueCommand("setDataInOut,"+mSerialPortOnDeviceUsb+",CMD,SBF");

    // make the receiver listen to RTK data on specified port
//    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3");

    // we want to know the TOW, because we don't want it to roll over! Answer is parsed below and used to sync time.
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");

    // use a static (not moving) base station
    slotQueueCommand("setDiffCorrUsage,LowLatency,20.0,auto,0,off");

    // xPPSOut is connected to DCD line of mSerialPortCom and uses hardpps() to sync clocks.
    // 0.00 is delay in nanoseconds, higher values cause PPS signal to be generated earlier.
    // 3600 is MaxSyncAge - time with no PVT after which PPS output is stopped. Ignored for RxClock timing system.
    slotQueueCommand("setPPSParameters,sec1,Low2High,0.00,RxClock,3600");

    // set up INS/GNSS integration
    slotQueueCommand("setDataInOut,COM2,MTI");

    // set time system to be GPS, not Galileo
    slotQueueCommand("setTimingSystem,GPS");

    // see Firmware User manual pg. 32: drift is so slow we won't ever touch this
    slotQueueCommand("setClockSyncThreshold,usec500");

    // when GPS fails, use IMU for how many seconds?
    slotQueueCommand("setExtSensorUsage,COM2,Accelerations+AngularRates,10");

    // specify vector from GPS antenna ARP to IMU in Vehicle reference frame
    // (vehicle reference frame has X forward, Y right and Z down)
    // IMU is 6cm in back, 10cm to the right and 26cm below ARP. Max precision is 1 cm.
    // Specifying orientation is not so easy (=fucking mess, Firmware User manual pg. 41)
    slotQueueCommand("setExtSensorCalibration,COM2,manual,180,0,0,manual,-0.06,-0.10,-0.26");

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    slotQueueCommand("setEventParameters,EventA,High2Low"); // Hokuyo
    slotQueueCommand("setEventParameters,EventB,High2Low"); // Camera

    // From Geert Dierckx:
    // By default, the parameter is set to Moderate (Let’s say moving vehicles like cars,
    // boats, trucks). Low is for pedestrian applications. In your case, it seems to be an
    // airplane-application, which may need the setting to be on High. In my opinion, it
    // will less smooth and take into account more the real movement of the application.
    // Just as a test, you could run the Max-value (as explained above, take care!).
    slotQueueCommand("setReceiverDynamics,High");

    // Usually, we'd configure as rover in standalone+rtk mode. Due to a firmware bug, the receiver only initializes the attitude
    // correctly when it starts in NON-RTK mode. Thus, we start in non-RTK mode, and when init succeeded, we enable RTK later.
    //slotQueueAsciiCommand("setPVTMode,Rover,all,auto,Loosely");
    slotQueueCommand("setPVTMode,Rover,StandAlone+SBAS+DGPS,auto,Loosely");

    // explicitly allow rover to use all RTCMv3 correction messages
    slotQueueCommand("setRTCMv3Usage,all");

    // Send IntPVAAGeod on Stream1. First slowly, then with 25 Hz when things are initialized and fixed.
    // We want to know the pose 25 times a second
    slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec200");

    // We want to know PVTCartesion for MeanCorrAge (average correction data age), ReceiverStatus for CPU Load and IntAttCovEuler for Covariances (sigma-values)
    slotQueueCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",PVTCartesian+ReceiverStatus+IntAttCovEuler,sec1");

    // We want to know whenever a scan is finished.
    slotQueueCommand("setSBFOutput,Stream3,"+mSerialPortOnDeviceCom+",ExtEvent,OnChange");

    // We want to know what time it is
    slotQueueCommand("setSBFOutput,Stream4,"+mSerialPortOnDeviceCom+",ReceiverTime,sec30");

    // show current config
    slotQueueCommand("lstConfigFile,Current");

    qDebug() << "GpsDevice::setupCommunication(): done setting up communication";
}

void GpsDevice::slotShutDown()
{
    qDebug() << "GpsDevice::slotShutDown(): starting shutdown sequence, disabling SBF streams, syncing clock, resetting dataInOut...";
    mDeviceIsInitialized = false;

    slotQueueCommand("setComSettings,all,baud115200,bits8,No,bit1,none");
    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceUsb+",none");
    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceCom+",none");
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");
    slotQueueCommand("setDataInOut,all,CMD,none");
    slotQueueCommand("setPVTMode,Rover,StandAlone");
    slotQueueCommand("shutdown");

    qDebug() << "GpsDevice::slotShutDown(): shutdown sequence processed, waiting for asynchronous shutdown confirmation...";
}

void GpsDevice::slotDataReadyOnCom()
{
    // Copy all new bytes into our log
    QDataStream s(mLogFileSbf); s << mSerialPortCom->peek(mSerialPortCom->bytesAvailable());

    // Move all new bytes into our SBF buffer
    mReceiveBufferCom.append(mSerialPortCom->readAll());

    //qDebug() << "GpsDevice::slotDataReadyOnCom(): size of SBF data is now" << mReceiveBufferCom.size() << "bytes, processing:" << mReceiveBufferCom;

    // Process as much SBF as possible.
    while(mSbfParser->processSbfData(mReceiveBufferCom));
}

void GpsDevice::slotDataReadyOnUsb()
{
    //qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb()";

    // Copy all new bytes into our log
    QDataStream s(mLogFileSbf);
    s << mSerialPortUsb->peek(mSerialPortUsb->bytesAvailable());

    // Move all new bytes into our SBF buffer
    mReceiveBufferUsb.append(mSerialPortUsb->readAll());

    if(mNumberOfRemainingRepliesUsb != 0)
    {
        // If the receiver replies to "exeSbfOnce" commands, process that data immediately. It might be receivertime, which is time-crucial.
        if(mReceiveBufferUsb.left(2) == QString("$@").toAscii())
        {
            qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): device replied to a request with SBF, processing" << mReceiveBufferUsb.size() << "SBF bytes.";
            //Q_ASSERT(mReceiveBufferUsb.indexOf("$@") == 0);
            while(mSbfParser->processSbfData(mReceiveBufferUsb));

            // Its quite possible that AFTER the SBF-data, there was the exeSBFOnce command reply. If so, this reply
            // is still in the buffer and needs to be processed. Luckily, this is done in the while-loop below.
        }

        while(mNumberOfRemainingRepliesUsb != 0 && mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">")) != -1) // while we found a complete chat reply
        {
            const int positionEndOfReply = mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">")) + 5;
            qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): received reply to:" << mLastCommandToDeviceUsb.trimmed() << ":" << mReceiveBufferUsb.size() << "bytes:" << mReceiveBufferUsb.left(positionEndOfReply).trimmed();

            QTextStream commandLog(mLogFileCmd);
            commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") << " DEV -> HOST: " << mReceiveBufferUsb.left(positionEndOfReply).trimmed() << endl;

            // After sending/receiving the SetPvtMode command, the rover needs to be static for better alignment. Tell the user to wait!
            if(QString(mReceiveBufferUsb.left(positionEndOfReply)).contains("SetPvtMode", Qt::CaseInsensitive))
                emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Integration filter started (alignment not ready), vehicle must remain static for 20s starting now.");

            if(mReceiveBufferUsb.left(positionEndOfReply).contains("$R? ASCII commands between prompts were discarded!"))
                qDebug() << t() <<  "GpsDevice::slotDataReadyOnUsb(): we were talking too fast!!";

            if(mReceiveBufferUsb.left(positionEndOfReply).contains("shutdown"))
            {
                emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Orderly shutdown finished");
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
        while(mSbfParser->processSbfData(mReceiveBufferUsb));
    }
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

void GpsDevice::slotSetSystemTime(const quint32& tow)
{
    // First, what time is it now?
    struct timeval system;
    gettimeofday(&system, NULL);

    // SecondsPerWeek - CurrentSecondInWeek is number of seconds till rollover
    const quint32 secondsToRollOver = (7 * 86400) - (tow / 1000);

    // Apply 15000ms = 15 leapseconds offset? Only when we really sync to UTC, which we don't.
    const qint32 offsetHostToGps = tow - getCurrentGpsTowTime() + 7; // Oscilloscope indicates 7ms offset is a good value.
    qDebug() << "GpsDevice::slotSetSystemTime(): time rollover in" << ((float)secondsToRollOver)/86400.0 << "d, offset host time" << getCurrentGpsTowTime() << "to gps time" << tow << "is" << offsetHostToGps/1000 << "s and" << (offsetHostToGps%1000) << "ms";

    // For small clock drifts AND when system is running, adjust clock. Else, set clock
    if(abs(offsetHostToGps) < 10 && mDeviceIsInitialized)
    {
        qDebug() << "GpsDevice::slotSetSystemTime(): offset smaller than 10ms, using adjtime to correct clock drift...";
        system.tv_sec = 0;
        system.tv_usec = offsetHostToGps * 1000;
        if(adjtime(&system, NULL) < 0)
        {
            if(errno == EINVAL) qDebug("GpsDevice::slotSetSystemTime(): couldn't adjust system time, values invalid.");
            else if(errno == EPERM) qDebug("GpsDevice::slotSetSystemTime(): couldn't adjust system time, insufficient permissions.");
            else qDebug("GpsDevice::slotSetSystemTime(): couldn't adjust system time, no idea why, error %d.", errno);
        }
    }
    else
    {
        qDebug() << "GpsDevice::slotSetSystemTime(): offset larger than 10ms or device startup, using settimeofday to set clock...";
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
            if(errno == EFAULT) qDebug("GpsDevice::slotSetSystemTime(): couldn't set system time, values outside of range.");
            else if(errno == EINVAL) qDebug("GpsDevice::slotSetSystemTime(): couldn't set system time, values invalid.");
            else if(errno == EPERM) qDebug("GpsDevice::slotSetSystemTime(): couldn't set system time, insufficient permissions.");
            else qDebug("GpsDevice::slotSetSystemTime(): couldn't set system time, no idea why, error %d.", errno);
        }
    }

    qDebug() << t() << "GpsDevice::slotSetSystemTime(): offset host to gps is" << offsetHostToGps;
}
