#include "gnssdevice.h"

/*
 * This is the receiver's bootup-config concerning I/O:
 * > $R: gdio
 * >   DataInOut, DSK1, CMD, SBF+NMEA, (off)
 * >   DataInOut, COM1 RS232, MTI, none, (on)
 * >   DataInOut, COM2 RS232, CMD, none, (on)
 * >   DataInOut, COM3 TTL, RTCMv3, NMEA, (on)
 * >   DataInOut, COM4 TTL, CMD, none, (on)
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

GnssDevice::GnssDevice(const QString &serialDeviceFileUsb, const QString &serialDeviceFileCom, QString logFilePrefix, QObject *parent) : QObject(parent)
{
    qDebug() << "GnssDevice::GnssDevice(): Using usb port" << serialDeviceFileUsb << "and com port" << serialDeviceFileCom;

    mLogFileSbf = new QFile(logFilePrefix + QString("gnssdata.sbf"));
    if(!mLogFileSbf->open(QIODevice::WriteOnly)) qFatal("GnssDevice::GnssDevice(): couldn't open sbf log file for writing, exiting");

    mLogFileCmd = new QFile(logFilePrefix + QString("gnsscommands.txt"));
    if(!mLogFileCmd->open(QIODevice::WriteOnly)) qFatal("GnssDevice::GnssDevice(): couldn't open cmd log file for writing, exiting");

    mSbfParser = new SbfParser;
    // If our SBF parser has a question to the device, forward it.
    connect(mSbfParser, SIGNAL(receiverCommand(QString)), SLOT(slotQueueCommand(QString)));
    connect(mSbfParser, SIGNAL(gnssTimeOfWeekEstablished(qint32)), SLOT(slotSetSystemTime(qint32)));
    connect(mSbfParser, SIGNAL(gnssDeviceWorkingPrecisely(bool)), SLOT(slotSetPoseFrequency(bool)));

    // We first feed SBF data to SbfParser, then we get it back from it via the signal below. A little complicated,
    // but the alternative is to watch two ports for incoming SBF, which might lead to mixing of SBF in between
    // packets.
    connect(mSbfParser, SIGNAL(processedPacket(qint32,const char*,quint16)), SLOT(slotLogProcessedSbfPacket(qint32,const char*,quint16)));

    mDeviceIsReadyToReceiveDiffCorr = false;
    mWaitingForCommandReply = false;
    mDiffCorrDataCounter = 0;
    mSerialPortOnDeviceCom = "COM3";
    mSerialPortOnDeviceUsb = "";

    // We use the USB port to talk to the GNSS receiver and receive poses
    mSerialPortUsb = new AbstractSerial();
//    mSerialPortUsb->enableEmitStatus(true);
//    connect(mSerialPortUsb, SIGNAL(signalStatus(QString,QDateTime)), SLOT(slotSerialPortStatusChanged(QString,QDateTime)));
    mSerialPortUsb->setDeviceName(serialDeviceFileUsb);
    if(!mSerialPortUsb->open(AbstractSerial::ReadWrite))
    {
        mSerialPortUsb->close();
        qFatal("GnssDevice::GnssDevice(): Opening serial usb port %s failed, exiting.", qPrintable(serialDeviceFileUsb));
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
        qFatal("GnssDevice::GnssDevice(): Opening serial com port %s failed, exiting.", qPrintable(serialDeviceFileCom));
    }
    mSerialPortCom->setBaudRate(AbstractSerial::BaudRate115200);
    mSerialPortCom->setDataBits(AbstractSerial::DataBits8);
    mSerialPortCom->setParity(AbstractSerial::ParityNone);
    mSerialPortCom->setStopBits(AbstractSerial::StopBits1);
    mSerialPortCom->setFlowControl(AbstractSerial::FlowControlOff);
    connect(mSerialPortCom, SIGNAL(readyRead()), SLOT(slotDataReadyOnCom()));

    mStatusTimer = new QTimer(this);
    connect(mStatusTimer, SIGNAL(timeout()), mSbfParser, SLOT(slotEmitCurrentGnssStatus()));
    mStatusTimer->start(2000); // emit status signal periodically.

    // initialize the device whenever we get time to do this. By doing it asynchronously, we can give our creator time to connect our signals and fetch them.
    QTimer::singleShot(0, this, SLOT(slotDetermineSerialPortsOnDevice()));
}

GnssDevice::~GnssDevice()
{
    qDebug() << "GnssDevice::~GnssDevice(): closing ports and logfiles...";

    mSerialPortUsb->close();
    mSerialPortUsb->deleteLater();

    mSerialPortCom->close();
    mSerialPortCom->deleteLater();

    mLogFileCmd->close();
    mLogFileCmd->deleteLater();

    mLogFileSbf->close();
    mLogFileSbf->deleteLater();

    mStatusTimer->stop();
    mStatusTimer->deleteLater();

    mSbfParser->deleteLater();

    qDebug() << "GnssDevice::~GnssDevice(): ports closed, log files flushed, destructed after receiving" << mDiffCorrDataCounter << "bytes diffcorr";
}

void GnssDevice::slotQueueCommand(QString command)
{
    qDebug() << "GnssDevice::slotQueueCommand(): queueing command" << command;
    command.replace("#USB#", mSerialPortOnDeviceUsb);
    command.replace("#COM#", mSerialPortOnDeviceCom);
    mCommandQueueUsb.append(command.append("\r\n").toAscii());

    if(!mWaitingForCommandReply)
        slotFlushCommandQueue();
}

quint8 GnssDevice::slotFlushCommandQueue()
{
    if(!mWaitingForCommandReply && mCommandQueueUsb.size())
    {
        mLastCommandToDeviceUsb = mCommandQueueUsb.takeFirst();
        qDebug() << "GnssDevice::slotFlushCommandQueue(): no pending replies, sending next command:" << mLastCommandToDeviceUsb.trimmed();
        if(mReceiveBufferUsb.size()) qDebug() << "GnssDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << SbfParser::readable(mReceiveBufferUsb);
        //usleep(100000);
        mSerialPortUsb->write(mLastCommandToDeviceUsb);
        mWaitingForCommandReply = true;

        QTextStream commandLog(mLogFileCmd);
        commandLog << '\n' << '\n' << "################################################################################" << '\n' << '\n';
        commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz") << " HOST -> DEV: " << mLastCommandToDeviceUsb << '\n';
        commandLog << "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV" << '\n' << '\n';
    }
    else if(mWaitingForCommandReply)
    {
        qDebug() << "GnssDevice::slotFlushCommandQueue(): still waiting for a usb command-reply, not sending.";
    }
    else
    {
        qDebug() << "GnssDevice::slotFlushCommandQueue(): nothing to send.";
    }

    // ben 2012-03-21: For some reason, when subscribing at msec20, we don't get called in slotDataReadyOnUsb() because th buffer is still (or already)
    // filled with crap. So, lets call ourselves every second.
//    QTimer::singleShot(1000, this, SLOT(slotFlushCommandQueue()));

    return mCommandQueueUsb.size();
}

void GnssDevice::slotLogProcessedSbfPacket(const qint32 tow, const char* sbfData, quint16 length)
{
    // Copy all new SBF bytes into our log. Don't use a datastream,
    // as that would add record-keeping-bytes in between the packets

    mLogFileSbf->write(sbfData, length);
}

void GnssDevice::slotDetermineSerialPortsOnDevice()
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
            mSerialPortUsb->waitForReadyRead(2000);
            dataUsb.append(mSerialPortUsb->readAll());
            qDebug() << "GnssDevice::determineSerialPortOnDevice():" << mSerialPortUsb->bytesAvailable() << "bytes waiting on serial usb port.";
        }

        // After waiting for the reply, read and analyze.
        QString portNameUsb = dataUsb.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataUsb.right(1) == ">" && (portNameUsb.left(3) == "COM" || portNameUsb.left(3) == "USB"))
        {
            mSerialPortOnDeviceUsb = portNameUsb;
            qDebug() << "GnssDevice::determineSerialPortOnDevice(): serial usb port on device is now" << mSerialPortOnDeviceUsb;
        }
        else
        {
            qWarning() << "GnssDevice::determineSerialPortOnDevice(): couldn't get serialUsbPortOnDevice," << dataUsb.size() << "bytes of data are:" << SbfParser::readable(dataUsb);
            qFatal("aborting.");
            emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Couldn't get serialUsbPortOnDevice");
        }

//        if(mSerialPortUsb->waitForReadyRead(200) || mSerialPortUsb->bytesAvailable())
//            qWarning() << "GnssDevice::determineSerialPortOnDevice(): after detecting" << mSerialPortOnDeviceUsb << "there are" << mSerialPortUsb->bytesAvailable() << "bytes left:" << mSerialPortUsb->readAll();
    }

    if(mSerialPortOnDeviceCom.isEmpty())
    {
        mSerialPortCom->write("getReceiverCapabilities\r\n");

        QByteArray dataCom;
        while((mSerialPortCom->bytesAvailable() + dataCom.size()) < 250)
        {
            mSerialPortCom->waitForReadyRead(2000);
            dataCom.append(mSerialPortCom->readAll());
            qDebug() << "GnssDevice::determineSerialPortOnDevice():" << mSerialPortCom->bytesAvailable() << "bytes waiting on serial com port.";
        }

        // After waiting for the reply, read and analyze.
        QString portNameCom = dataCom.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataCom.right(1) == ">" && (portNameCom.left(3) == "COM" || portNameCom.left(3) == "USB"))
        {
            mSerialPortOnDeviceCom = portNameCom;
            qDebug() << "GnssDevice::determineSerialPortOnDevice(): serial com port on device is now " << mSerialPortOnDeviceCom;
        }
        else
        {
            qWarning() << "GnssDevice::determineSerialPortOnDevice(): couldn't get serialComPortOnDevice, data is:" << dataCom;
            emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Couldn't get serialComPortOnDevice");
        }

//        if(mSerialPortCom->waitForReadyRead(200) || mSerialPortCom->bytesAvailable())
//            qWarning() << "GnssDevice::determineSerialPortOnDevice(): after detecting" << mSerialPortOnDeviceCom << "there are" << mSerialPortCom->bytesAvailable() << "bytes left:" << mSerialPortCom->readAll();
    }

    // Now that we know what the ports are named, we can setup the board.
    // We connect this signal not in the c'tor but here, because we don't want the slot
    // to be called for answers to requests made in this method.
    qDebug() << "GnssDevice::determineSerialPortOnDevice(): activating GNSS device output parsing.";
    connect(mSerialPortUsb, SIGNAL(readyRead()), SLOT(slotDataReadyOnUsb()));
    slotCommunicationSetup();
}

void GnssDevice::slotCommunicationSetup()
{
    Q_ASSERT(mSerialPortOnDeviceUsb.size() == 4);
    Q_ASSERT(mSerialPortOnDeviceCom.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Rover-Mode

    qDebug() << "GnssDevice::setupCommunication(): setting up communication";

    // use bootup config for now

    // show current config
    slotQueueCommand("lstConfigFile,Current");

    slotQueueCommand("lstInternalFile,Identification");

    slotQueueCommand("lstInternalFile,Permissions");

//    slotQueueCommand("lstInternalFile,Debug");

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
    slotQueueCommand("setDataInOut,COM1,MTI");

    // set time system to be GPS, not Galileo
    slotQueueCommand("setTimingSystem,GPS");

    // see Firmware User manual pg. 32: drift is so slow we won't ever touch this
    slotQueueCommand("setClockSyncThreshold,usec500");

    // when GPS fails, use IMU for how many seconds?
    slotQueueCommand("setExtSensorUsage,COM1,Accelerations+AngularRates,10");

    // specify vector from GPS antenna ARP to IMU in Vehicle reference frame
    // (vehicle reference frame has X forward, Y right and Z down)
    // IMU is 2cm in front, 10cm to the right and 47cm below ARP. Max precision is 1 cm.
    // Specifying orientation is not so easy (=fucking mess, Firmware User manual pg. 41)
    slotQueueCommand("setExtSensorCalibration,COM1,manual,180,0,0,manual,0.02,-0.10,-0.47");

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    //slotQueueCommand("setEventParameters,EventA,High2Low"); // Hokuyo
    slotQueueCommand("setEventParameters,EventB,High2Low"); // Camera

    // From Geert Dierckx:
    // By default, the parameter is set to Moderate (Let’s say moving vehicles like cars,
    // boats, trucks). Low is for pedestrian applications. In your case, it seems to be an
    // airplane-application, which may need the setting to be on High. In my opinion, it
    // will less smooth and take into account more the real movement of the application.
    // Just as a test, you could run the Max-value (as explained above, take care!).
    slotQueueCommand("setReceiverDynamics,Moderate");

    // Configure as rover in StandAlone+RTK mode.
    slotQueueCommand("setPVTMode,Rover,all,auto,Loosely");

    // After sending/receiving the SetPvtMode command, the rover needs to be static for better alignment. Tell the user to wait!
    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Integration filter started (alignment not ready), vehicle must remain static for 20s starting now.");

    // explicitly allow rover to use all RTCMv3 correction messages
    slotQueueCommand("setRTCMv3Usage,all");

    // Possible intervals:
    //  msec: 10, 20, 40, 50, 100, 200, 500
    //  sec:  1, 2, 5, 10, 15, 30, 60
    //  min:  2, 5, 10, 15, 30, 60

    // We want to know the pose - often. But on startup, we ask for slower data and then raise to msec20 when
    // the poses are of higher quality. See slotSetPoseFrequency()
    slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec500");

    // We want to know PVTCartesion for MeanCorrAge (average correction data age), ReceiverStatus for CPU Load and IntAttCovEuler for Covariances (sigma-values)
    slotQueueCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",PVTCartesian+ReceiverStatus+IntAttCovEuler,msec500");

    // We want to know whenever a scan is finished.
    slotQueueCommand("setSBFOutput,Stream3,"+mSerialPortOnDeviceCom+",ExtEvent,OnChange");

    // We want to know what time it is
    // Update: No, we don't. Re-setting system-time in times of high system loads does NOT improve
    // the system clock's accuracy. Offsets of ~700ms(!) have been observed. So, skip this for now.
    //slotQueueCommand("setSBFOutput,Stream4,"+mSerialPortOnDeviceCom+",ReceiverTime,sec30");

    // For now, record support messages for septentrio
    slotQueueCommand("setSBFOutput,Stream5,"+mSerialPortOnDeviceUsb+",Support,msec500"); // septentrio wants msec100, but that kills the cpu

    // Why doesn't BBSamples work? It seems the device has never seen it?!
    slotQueueCommand("setSBFOutput,Stream6,"+mSerialPortOnDeviceUsb+",BBSamples,sec1"); // septentrio wants msec100, but that kills the cpu

    //slotQueueCommand("setSBFOutput,Stream7,"+mSerialPortOnDeviceUsb+",ExtSensorMeas,msec20");

    slotQueueCommand("setSBFOutput,Stream8,"+mSerialPortOnDeviceUsb+",MeasEpoch,msec200");

    // show current config
    slotQueueCommand("lstConfigFile,Current");

    qDebug() << "GnssDevice::setupCommunication(): done setting up communication";
}

void GnssDevice::slotSetPoseFrequency(bool highSpeed)
{
    qDebug() << "GnssDevice::slotSetPoseFrequency(): setting new interval, highSpeed is" << highSpeed;

    /*
      Different IntPVAAGeod intervals and the consequences:

       - msec20 means 50 poses per second, like this: IEEEEIEEEEIEEEEIEEEEI...
         There's an integrated pose every 100 msec, in between are 4 INS only poses.

            399969.500,1693,Loosely-integrated solution (INS+GNSS)
            399969.520,1693,Loosely-integrated solution (INS only)
            399969.540,1693,Loosely-integrated solution (INS only)
            399969.560,1693,Loosely-integrated solution (INS only)
            399969.580,1693,Loosely-integrated solution (INS only)
            399969.600,1693,Loosely-integrated solution (INS+GNSS)
            399969.620,1693,Loosely-integrated solution (INS only)
            399969.640,1693,Loosely-integrated solution (INS only)
            399969.660,1693,Loosely-integrated solution (INS only)
            399969.680,1693,Loosely-integrated solution (INS only)
            399969.700,1693,Loosely-integrated solution (INS+GNSS)

       - msec40 means 25 poses per second, like this: IEEEEIEEEEIEEEEIEEEEI...
         There's an integrated pose every 200 msec, in between are 4 INS only poses.

            306591.000,1704,Loosely-integrated solution (INS+GNSS)
            306591.040,1704,Loosely-integrated solution (INS only)
            306591.080,1704,Loosely-integrated solution (INS only)
            306591.120,1704,Loosely-integrated solution (INS only)
            306591.160,1704,Loosely-integrated solution (INS only)
            306591.200,1704,Loosely-integrated solution (INS+GNSS)
            306591.240,1704,Loosely-integrated solution (INS only)
            306591.280,1704,Loosely-integrated solution (INS only)
            306591.320,1704,Loosely-integrated solution (INS only)
            306591.360,1704,Loosely-integrated solution (INS only)
            306591.400,1704,Loosely-integrated solution (INS+GNSS)

       - msec50 means 20 poses per second, like this: IEIEIEIEIEIEIEIEIEIEI...
         There's an integrated pose every 100 msec, in between is 1 INS only pose.

            219478.000,1704,Loosely-integrated solution (INS+GNSS)
            219478.050,1704,Loosely-integrated solution (INS only)
            219478.100,1704,Loosely-integrated solution (INS+GNSS)
            219478.150,1704,Loosely-integrated solution (INS only)

      I is an integrated pose (GNSS and IMU)
      E is an extrapolated pose (GNSS from the past plus IMU readings)

      According to the sequences above, msec20 gives us 10 integrated packets per
      second, which is ideal for forwarding only those to the flightcontroller, which
      will then have 10Hz input (the kopter cannot deal with 50Hz controller-input??!)

      As for the msec50 option, the result is the same: 10 integrated packets per second.
      If that is enough for sensorfuser (to be decided), then lets use msec50!
    */

    if(highSpeed)
    {
        // Maybe use msec50 instead of msec20 to avoid RxError 64 (congestion on line)
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec40");
    }
    else
    {
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec200");
    }
}

/* Same as above, for testing
void GnssDevice::slotTogglePoseFrequencyForTesting()
{
    qDebug() << "switching pose frequency!";

    static bool state = true;
    if(state)
    {
        // Maybe use msec40 instead of msec20 to avoid RxError 64 (congestion on line)
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec50");
    }
    else
    {
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec200");
    }

    state = !state;
}*/

void GnssDevice::slotShutDown()
{
    qDebug() << "GnssDevice::slotShutDown(): starting shutdown sequence, disabling SBF streams, syncing clock, resetting dataInOut...";

    slotQueueCommand("setComSettings,all,baud115200,bits8,No,bit1,none");
    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceUsb+",none");
    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceCom+",none");
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");
    slotQueueCommand("setDataInOut,all,CMD,none");
    slotQueueCommand("setPVTMode,Rover,StandAlone");
    slotQueueCommand("shutdown");

    qDebug() << "GnssDevice::slotShutDown(): shutdown sequence processed, waiting for asynchronous shutdown confirmation...";
}

void GnssDevice::slotDataReadyOnCom()
{
    // Move all new bytes into our SBF buffer
    mReceiveBufferCom.append(mSerialPortCom->readAll());

    //qDebug() << "GnssDevice::slotDataReadyOnCom(): size of SBF data is now" << mReceiveBufferCom.size() << "bytes, processing:" << mReceiveBufferCom;

    // Process as much SBF as possible.
    while(mSbfParser->getNextValidPacketInfo(mReceiveBufferCom))
          mSbfParser->processNextValidPacket(mReceiveBufferCom);
}

void GnssDevice::slotDataReadyOnUsb()
{
    while(true)
    {
        // Move all new bytes into our SBF buffer
        if(mSerialPortUsb->bytesAvailable())
            mReceiveBufferUsb.append(mSerialPortUsb->readAll());

        // $@<header/><body>...$@...</body>...padding...$R;listCurrentConfig\n\n...........\nUS
        // $@<header/><body>...$@...</body>...padding...$R;listCurrentConfig\n\n...........\nUSB1>$@<header/><body>...

        if(mWaitingForCommandReply)
        {
            // Check for command replies before every packet
            const qint32 positionReplyStart = mReceiveBufferUsb.indexOf("$R");
            qint32 positionReplyStop  = mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QChar('>'));
            if(positionReplyStart != -1 && positionReplyStop != -1)
            {
                positionReplyStop += mSerialPortOnDeviceUsb.length() + 1; // make sure we also include the "USB1>" at the end!
                const QByteArray commandReply = mReceiveBufferUsb.mid(positionReplyStart, positionReplyStop - positionReplyStart);

                qDebug() << "GnssDevice::slotDataReadyOnUsb(): received reply to:" << mLastCommandToDeviceUsb.trimmed() << "-" << commandReply.size() << "bytes:" << commandReply.trimmed();

                QTextStream commandLog(mLogFileCmd);
                commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz") << " DEV -> HOST: " << commandReply.trimmed() << '\n';
                commandLog << '\n' << "################################################################################" << '\n' << '\n' << '\n' << '\n';

                if(commandReply.contains("$R? ASCII commands between prompts were discarded!"))
                    qDebug() << "GnssDevice::slotDataReadyOnUsb(): WARNING, we were talking too fast!!";

                if(commandReply.contains(QString("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3,SBF").toAscii()))
                {
                    qDebug() << "GnssDevice::slotDataReadyOnUsb(): gnss device now configured to accept diffcorr";
                    mDeviceIsReadyToReceiveDiffCorr = true;
                }

                if(commandReply.contains("shutdown"))
                {
                    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Orderly shutdown confirmed by gnss device");
                    qDebug() << "GnssDevice::slotDataReadyOnUsb(): shutdown confirmed by device, quitting.";
                    QCoreApplication::quit();
                }

                const int bytesToCut = positionReplyStop - positionReplyStart;
                mReceiveBufferUsb.remove(positionReplyStart, bytesToCut);
                mWaitingForCommandReply = false;

                slotFlushCommandQueue();
            }
        }

        if(mSbfParser->getNextValidPacketInfo(mReceiveBufferUsb))
        {
            // There is at least one valid packet in the buffer, process it.
            mSbfParser->processNextValidPacket(mReceiveBufferUsb);
        }
        else
        {
            // There is no valid packet in the buffer. Check whether its a coming packet
//            qDebug() << "GnssDevice::slotDataReadyOnUsb():" << mReceiveBufferUsb.size() << "bytes left after parsing, but this is no valid packet yet:" << SbfParser::readable(mReceiveBufferUsb);
            return;
        }
    }
}

void GnssDevice::slotSetDifferentialCorrections(const QByteArray* const differentialCorrections)
{
    if(mDeviceIsReadyToReceiveDiffCorr)
    {
        // simply write the RTK data into the com-port
        mDiffCorrDataCounter += differentialCorrections->size();
//        qDebug() << "GnssDevice::slotSetDifferentialCorrections(): forwarding" << data.size() << "bytes of diffcorr to gps device, total is" << mRtkDataCounter;
        mSerialPortCom->write(*differentialCorrections);
//        emit message(
//                Information,
//                "GnssDevice::slotSetDifferentialCorrections()",
//                QString("Fed %1 bytes of RTK data into rover gps device.").arg(data.size())
//                );
    }
    else
    {
        qDebug() << "GnssDevice::slotSetRtkData(): NOT forwarding" << differentialCorrections->size() << "bytes of rtk-data to gnss device, its not initialized yet.";
    }
}

void GnssDevice::slotSerialPortStatusChanged(const QString& status, const QDateTime& time)
{
    if(sender() == mSerialPortUsb)
    {
        qDebug() << "GnssDevice::slotSerialPortStatusChanged(): usb port status" << status << "errorstring" << mSerialPortUsb->errorString();
    }
    else if(sender() == mSerialPortCom)
    {
        qDebug() << "GnssDevice::slotSerialPortStatusChanged(): com port status" << status << "errorstring" << mSerialPortCom->errorString();
    }
    else
    {
        qDebug() << "GnssDevice::slotSerialPortStatusChanged(): ??? port status" << status;
    }
}

void GnssDevice::slotSetSystemTime(const qint32& tow)
{
    // First, what time is it now?
    struct timeval system;
    gettimeofday(&system, NULL);

    // SecondsPerWeek - CurrentSecondInWeek is number of seconds till rollover
    const quint32 secondsToRollOver = (7 * 86400) - (tow / 1000);

    // Apply 15000ms = 15 leapseconds offset? Only when we really sync to UTC, which we don't.
    const qint32 offsetHostToGps = tow - GnssTime::currentTow() + 7; // Oscilloscope indicates 7ms offset is a good value.
    qDebug() << "GnssDevice::slotSetSystemTime(): time rollover in" << ((float)secondsToRollOver)/86400.0f << "d, offset host time" << GnssTime::currentTow() << "to gps time" << tow << "is" << offsetHostToGps/1000 << "s and" << (offsetHostToGps%1000) << "ms";

    // For small clock drifts, adjust clock. Else, set clock
    if(abs(offsetHostToGps) < 10)
    {
        qDebug() << "GnssDevice::slotSetSystemTime(): offset smaller than 10ms, using adjtime to correct clock drift...";
        system.tv_sec = 0;
        system.tv_usec = offsetHostToGps * 1000;
        if(adjtime(&system, NULL) < 0)
        {
            if(errno == EINVAL) qDebug("GnssDevice::slotSetSystemTime(): couldn't adjust system time, values invalid.");
            else if(errno == EPERM) qDebug("GnssDevice::slotSetSystemTime(): couldn't adjust system time, insufficient permissions.");
            else qDebug("GnssDevice::slotSetSystemTime(): couldn't adjust system time, no idea why, error %d.", errno);
        }
    }
    else
    {
        qDebug() << "GnssDevice::slotSetSystemTime(): offset larger than 10ms or device startup, using settimeofday to set clock...";
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
            if(errno == EFAULT) qDebug("GnssDevice::slotSetSystemTime(): couldn't set system time, values outside of range.");
            else if(errno == EINVAL) qDebug("GnssDevice::slotSetSystemTime(): couldn't set system time, values invalid.");
            else if(errno == EPERM) qDebug("GnssDevice::slotSetSystemTime(): couldn't set system time, insufficient permissions.");
            else qDebug("GnssDevice::slotSetSystemTime(): couldn't set system time, no idea why, error %d.", errno);
        }
    }

    qDebug() << "GnssDevice::slotSetSystemTime(): time synchronized, offset host to gps was" << offsetHostToGps << "ms";
}
