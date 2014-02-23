#include "gnssdevice.h"
#include <profiler.h>

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
 */

GnssDevice::GnssDevice(const QString &serialDeviceFileUsb, const QString &serialDeviceFileCom, QString logFilePrefix, QObject *parent) : QObject(parent)
{
    qDebug() << "GnssDevice::GnssDevice(): Using usb port" << serialDeviceFileUsb << "and com port" << serialDeviceFileCom;

    mLogFileSbf = new LogFile(logFilePrefix + QString(".ins"), LogFile::Encoding::Binary);
    mLogFileCmd = new LogFile(logFilePrefix + QString(".inscmd"), LogFile::Encoding::Text);

    mSbfParser = new SbfParser;
    // If our SBF parser has a question to the device, forward it.
    connect(mSbfParser, &SbfParser::receiverCommand, this, &GnssDevice::slotQueueCommand);
    connect(mSbfParser, &SbfParser::gnssTimeOfWeekEstablished, this, &GnssDevice::slotSetSystemTime);
    connect(mSbfParser, &SbfParser::gnssDeviceWorkingPrecisely, this, &GnssDevice::slotSetPoseFrequency);

    mGnssDeviceIsConfigured = false;
    mDiffCorrDataCounter = 0;

    mSerialPortOnDeviceCom = ""; // was set to COM3 previously
    mSerialPortOnDeviceUsb = "";
    mDataCursorUsb = 0;
    mDataCursorCom = 0;
    mReceiveBufferCom.reserve(mMaximumDataBufferSize);
    mReceiveBufferUsb.reserve(mMaximumDataBufferSize);

    // We use the USB port to talk to the GNSS receiver and receive poses
    mSerialPortUsb = new QSerialPort(serialDeviceFileUsb, this);
    //mSerialPortUsb->enableEmitStatus(true);
    //connect(mSerialPortUsb, &QSerialPort::signalStatus, this, &GnssDevice::slotSerialPortStatusChanged);
    //mSerialPortUsb->setDeviceName(serialDeviceFileUsb);
    if(!mSerialPortUsb->open(QIODevice::ReadWrite))
    {
        mSerialPortUsb->close();
        qFatal("GnssDevice::GnssDevice(): Opening serial usb port %s failed, exiting.", qPrintable(serialDeviceFileUsb));
    }
    mSerialPortUsb->setBaudRate(QSerialPort::Baud115200);
    mSerialPortUsb->setDataBits(QSerialPort::Data8);
    mSerialPortUsb->setParity(QSerialPort::NoParity);
    mSerialPortUsb->setStopBits(QSerialPort::OneStop);
    mSerialPortUsb->setFlowControl(QSerialPort::NoFlowControl);

    // We use the COM port to feed the device RTK correction data
    mSerialPortCom = new QSerialPort(serialDeviceFileCom, this);
    //mSerialPortCom->enableEmitStatus(true);
    //connect(mSerialPortCom, &QSerialPort::signalStatus, this, &GnssDevice::slotSerialPortStatusChanged);
    //mSerialPortCom->setDeviceName(serialDeviceFileCom);
    if(!mSerialPortCom->open(QIODevice::ReadWrite))
    {
        mSerialPortCom->close();
        qFatal("GnssDevice::GnssDevice(): Opening serial com port %s failed, exiting.", qPrintable(serialDeviceFileCom));
    }
    mSerialPortCom->setBaudRate(QSerialPort::Baud115200);
    mSerialPortCom->setDataBits(QSerialPort::Data8);
    mSerialPortCom->setParity(QSerialPort::NoParity);
    mSerialPortCom->setStopBits(QSerialPort::OneStop);
    mSerialPortCom->setFlowControl(QSerialPort::NoFlowControl);

    mStatusTimer = new QTimer(this);
    connect(mStatusTimer, &QTimer::timeout, mSbfParser, &SbfParser::slotEmitCurrentGnssStatus);
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

    delete mLogFileCmd;
    delete mLogFileSbf;

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
    mCommandQueueUsb.append(command.append("\r\n").toLatin1());

    if(mLastCommandToGnssDevice[mSerialPortOnDeviceUsb].isEmpty())
        slotFlushCommandQueue();
}

quint8 GnssDevice::slotFlushCommandQueue()
{
    if(mLastCommandToGnssDevice[mSerialPortOnDeviceUsb].isEmpty() && mCommandQueueUsb.size())
    {
        mLastCommandToGnssDevice[mSerialPortOnDeviceUsb] = mCommandQueueUsb.takeFirst();
        qDebug() << "GnssDevice::slotFlushCommandQueue(): no pending replies, sending next command:" << mLastCommandToGnssDevice[mSerialPortOnDeviceUsb].trimmed();
        if(mReceiveBufferUsb.size() > mDataCursorUsb + 1) qDebug() << "GnssDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << SbfParser::readable(mReceiveBufferUsb.mid(mDataCursorUsb));
        //usleep(100000);
        mSerialPortUsb->write(mLastCommandToGnssDevice[mSerialPortOnDeviceUsb]);

        QByteArray text;
        QTextStream commandLog(&text);
        commandLog << '\n' << '\n' << "################################################################################" << '\n' << '\n';
        commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz") << "HOST ->" << mSerialPortOnDeviceUsb << ":" << mLastCommandToGnssDevice[mSerialPortOnDeviceUsb] << '\n';
        commandLog << "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV" << '\n' << '\n';
        commandLog.flush();
        mLogFileCmd->write(text);
    }
    else if(!mLastCommandToGnssDevice[mSerialPortOnDeviceUsb].isEmpty())
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

//void GnssDevice::slotLogProcessedSbfPacket(const qint32 tow, const char* sbfData, quint16 length)
//{
    // Copy all new SBF bytes into our log. Don't use a datastream,
    // as that would add record-keeping-bytes in between the packets


//}

void GnssDevice::slotDetermineSerialPortsOnDevice()
{
    // This method finds the name of the used communication-port on the GNSS-device.
    // For example, we might connect using /dev/ttyUSB1, which is a usb2serial adapter
    // and connects to the devices COM2. This method will chat with the device, analyze
    // the prompt in its replies and set mSerialPortOnDevice to COM2. This can be used
    // lateron to tell the device to output useful info on COM2.
    // We use two connections to the board, the other one is just for feeding the RTK
    // data that we received from rtkfetcher. But we also need to know that ports name,
    // as we need to tell the receiver to accept RTK data on that port.
    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Setting up communication");

    if(!mSerialPortUsb->isOpen() || !mSerialPortCom->isOpen())
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Cannot open GNSS serial port(s)");
        QCoreApplication::quit();
    }

    // Historically, we would boot the GNSS receiver in a default configuration and then
    // set it up here and in slotCommunicationSetup() to emit useful info. This procedure
    // included (re)starting the INS filters. As it turns out, this is not so smart because
    // we WANT the INS filters to run long before we start flying, so they can calibrate
    // and converge to better precision, which is sometimes noticeable in lower covariances.
    //
    // This means that we want to start koptercontrol, initialize the GNSS and then, when
    // the filters work well, quit the program. On the next start, we do NOT want to re-
    // configure the GNSS board but just use it in its pre-configured state.
    //
    // As a consequence, the following setup procedure needs to work even if the device is
    // spitting out SBF at a rate of 50Hz. We try to achieve this by issuing a text command
    // and looking for the command prompt using regexp.

    // We send a string causing a reply from the device, so we can see on what port it is talking to us.
    if(mSerialPortOnDeviceUsb.isEmpty() || mSerialPortOnDeviceCom.isEmpty())
    {
        // We want to find strings like COM4> and USB1>
        QRegExp regexpPortNameFromPrompt("[\\r\\n](COM|USB)([1-4])>");
        // Make sure our regexp will capture two things: name and number
        Q_ASSERT(regexpPortNameFromPrompt.captureCount() == 2);

        QByteArray dataUsb;
        quint8 attemptCounter = 1;
        do
        {
            if(attemptCounter % 5 == 0)
            {
                qDebug() << "GnssDevice::determineSerialPortOnDevice(): sending portreset to device on USB...";
                mSerialPortUsb->write("SSSSSSSSSS\r\n");
            }
            else
            {
                mSerialPortUsb->write("getDataInOut\r\n");
            }

            QTime t; t.start();
            while(t.elapsed() < 300)
            {
                mSerialPortUsb->waitForReadyRead(100);
                dataUsb.append(mSerialPortUsb->readAll());
            }

            if(regexpPortNameFromPrompt.indexIn(QString::fromLatin1(dataUsb.constData(), dataUsb.size())) != -1)
            {
                mSerialPortOnDeviceUsb = QString(regexpPortNameFromPrompt.cap(1) + regexpPortNameFromPrompt.cap(2)).trimmed();
                qDebug() << "GnssDevice::determineSerialPortOnDevice(): serial usb port on device changed to" << mSerialPortOnDeviceUsb;

                if(mSerialPortOnDeviceCom.isEmpty())
                {
                    // We received an answer to getDataInOut and now know the USB port's name. We also received
                    // the gdio-output. If the device was already configured, this output will tell us the name
                    // of the COM-port used for SBF input. Try to fetch it!

                    // We want to find strings like "  DataInOut, COM2, RTCMv3, SBF, (on)" and match the COM2
                    QRegExp regexpPortNameFromDataInOutTable("DataInOut,\\s(COM|USB)([1-4]),\\sRTCMv3,\\sSBF,\\s\\(on\\)");
                    // Make sure our regexp will capture two things: name and number
                    Q_ASSERT(regexpPortNameFromDataInOutTable.captureCount() == 2);

                    if(regexpPortNameFromDataInOutTable.indexIn(QString::fromLatin1(dataUsb.constData(), dataUsb.size())) != -1)
                    {
                        mSerialPortOnDeviceCom = QString(regexpPortNameFromDataInOutTable.cap(1) + regexpPortNameFromDataInOutTable.cap(2)).trimmed();
                        qDebug() << "GnssDevice::determineSerialPortOnDevice(): getDataInOut indicates serial com port on device is" << mSerialPortOnDeviceCom;
                        qDebug() << "GnssDevice::determineSerialPortOnDevice(): device is pre-configured, thats fine.";
                        mGnssDeviceIsConfigured = true;
                    }
                    else
                    {
                        qDebug() << "GnssDevice::determineSerialPortOnDevice(): couldn't find com port name from getDataInOut, device isn't configured yet. String was:" << SbfParser::readable(dataUsb);
                    }
                }
            }
            else
            {
                qDebug() << "GnssDevice::determineSerialPortOnDevice(): couldn't find usb port name in prompt (retrying), data was" << dataUsb.size() << "bytes:" << SbfParser::readable(dataUsb);
                attemptCounter++;
            }

        } while(mSerialPortOnDeviceUsb.isEmpty());
    }

    // If the device was NOT preconfigured, we couldn't possibly have found mSerialPortOnDeviceCom above.
    // So, lets try to detect it using the same procedure as for the USB port.
    QByteArray dataCom;
    // We want to find strings like COM4> and USB1>
    QRegExp regexpPortNameFromPrompt("[\\r\\n](COM|USB)([1-4])>");
    // Make sure our regexp will capture two things: name and number
    Q_ASSERT(regexpPortNameFromPrompt.captureCount() == 2);

    quint8 attemptCounter = 1;
    while(mSerialPortOnDeviceCom.isEmpty())
    {
        if(attemptCounter % 20 == 0)
        {
            qDebug() << "GnssDevice::determineSerialPortOnDevice(): sending portreset to device on COM...";
            mSerialPortCom->write("SSSSSSSSSS\r\n");
        }
        else
        {
            mSerialPortCom->write("getReceiverCapabilities\r\n");
        }

        QTime t; t.start();
        while(t.elapsed() < 300)
        {
            mSerialPortCom->waitForReadyRead(100);
            dataCom.append(mSerialPortCom->readAll());
        }

        if(regexpPortNameFromPrompt.indexIn(QString::fromLatin1(dataCom.constData(), dataCom.size())) != -1)
        {
            mSerialPortOnDeviceCom = QString(regexpPortNameFromPrompt.cap(1) + regexpPortNameFromPrompt.cap(2)).trimmed();
            qDebug() << "GnssDevice::determineSerialPortOnDevice(): serial com port on device changed to" << mSerialPortOnDeviceCom;
        }
        else
        {
            qDebug() << "GnssDevice::determineSerialPortOnDevice(): couldn't find com port name in prompt (retrying), dataCom is" << dataCom.size() << "bytes :" << SbfParser::readable(dataCom);
            attemptCounter++;
        }
    }

    // Because of using do...while(), we potentially sent many commands to the GNSS receiver. Lets wait and clear all incoming data
    QTime t; t.start();
    while(t.elapsed() < 500)
    {
        mSerialPortCom->waitForReadyRead(100);
        mSerialPortCom->readAll();
        mSerialPortUsb->waitForReadyRead(100);
        mSerialPortUsb->readAll();
    }

    mLastCommandToGnssDevice[mSerialPortOnDeviceCom] = QByteArray();
    mLastCommandToGnssDevice[mSerialPortOnDeviceUsb] = QByteArray();

    // Now that we know what the ports are named, we can setup the board. We connect this signal not in the c'tor but here,
    // because we don't want the slot to be called for answers to requests made in this method.
    qDebug() << "GnssDevice::determineSerialPortOnDevice(): ports are" << mSerialPortOnDeviceCom << mSerialPortOnDeviceUsb << ": activating GNSS device output parsing.";
    connect(mSerialPortUsb, &QSerialPort::readyRead, this, &GnssDevice::slotDataReadyOnUsb);
    connect(mSerialPortCom, &QSerialPort::readyRead, this, &GnssDevice::slotDataReadyOnCom);

    if(mGnssDeviceIsConfigured)
    {
        // We still need to ask for the time to set system and laserscanner clock
        qDebug() << "GnssDevice::determineSerialPortOnDevice(): GNSS device is already configured, skipping configuration, asking for time.";
        slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");
        // slotSetPoseFrequency(true) is not needed, SBFParser will cause this method to be called when the data is precise.
    }
    else
    {
        qDebug() << "GnssDevice::determineSerialPortOnDevice(): GNSS device is not configured, starting configuration procedure.";
        slotCommunicationSetup();
    }
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

    // increase com-port speed to 460800 for shorter latency? No.
    slotQueueCommand("setComSettings,"+mSerialPortOnDeviceCom+",baud115200,bits8,No,bit1,none");

    // make the receiver output SBF blocks on both COM and USB connections
    slotQueueCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3,SBF");
    slotQueueCommand("setDataInOut,"+mSerialPortOnDeviceUsb+",CMD,SBF");

    // we want to know the TOW, because we don't want it to roll over! Answer is parsed below and used to sync time.
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");
    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceCom+",ReceiverTime");

    // use a static (not moving) base station
    slotQueueCommand("setDiffCorrUsage,LowLatency,20.0,auto,0,off");

    // xPPSOut is connected to DCD line of mSerialPortCom and uses hardpps() to sync clocks - NOT true, the com port is connected via USB :(
    // 0.00 is delay in nanoseconds, higher values cause PPS signal to be generated earlier.
    // 3600 is MaxSyncAge - time with no PVT after which PPS output is stopped. Ignored for RxClock timing system.
    slotQueueCommand("setPPSParameters,sec1,Low2High,0.00,RxClock,3600");

    // set up INS/GNSS integration
    slotQueueCommand("setDataInOut,COM1,MTI");

    // set time system to be GPS, not Galileo
    slotQueueCommand("setTimingSystem,GPS");

    // see Firmware User manual pg. 32: drift is so slow we won't ever touch this
    slotQueueCommand("setClockSyncThreshold,usec500");

    // when GNSS fails, use IMU for how many seconds?
    slotQueueCommand("setExtSensorUsage,COM1,Accelerations+AngularRates,10");

    // Specify offset from GNSS antenna ARP to IMU in Vehicle reference frame
    // (vehicle reference frame has X forward, Y right and Z down)
    //
    // Specifying orientation is not so easy (=fucking mess, Firmware User manual pg. 41)
    //
    // What I consider default (orange hat on top, aluminum-base down, plug towards rear)
    // is 180,0,0 (flipped on X, because Septentrio thinks it belongs with the aluminum base on top).
    // This means that the drawings on that page are correct, and the text is fishy:
    //
    // "By default, the INS/GNSS integration filter assumes that the IMU is mounted horizontally,
    // *upside up* and...
    //
    // So, upside is the aluminum base!
    // Since the aluminum base *is* currently on top and we have another wicked rotation
    // (Y points forward), we add the -90 deg rotation on Z and hope it'll work.

    // IMU is 4cm in front, 3cm to the right and 34cm below ARP. Max precision is 1 cm.
    //slotQueueCommand("setExtSensorCalibration,COM1,manual,0,0,270,manual,-0.05,0.14,0.35");
    slotQueueCommand("setExtSensorCalibration,COM1,manual,0,0,0,manual,0.04,0.03,0.36");

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    //slotQueueCommand("setEventParameters,EventA,High2Low"); // Hokuyo
    slotQueueCommand("setEventParameters,EventB,High2Low"); // Camera

    // From Geert Dierckx:
    // By default, the parameter is set to Moderate (Let’s say moving vehicles like cars,
    // boats, trucks). Low is for pedestrian applications. In your case, it seems to be an
    // airplane-application, which may need the setting to be on High. In my opinion, it
    // will less smooth and take into account more the real movement of the application.
    // Just as a test, you could run the Max-value (as explained above, take care!).
    //slotQueueCommand("setReceiverDynamics,Moderate");

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
    slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec100");

    // We want to know PVTCartesion for MeanCorrAge (average correction data age), ReceiverStatus for CPU Load and IntAttCovEuler for Covariances (sigma-values)
    slotQueueCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",PVTCartesian+ReceiverStatus+IntAttCovEuler,msec500");

    // We want to know whenever a scan is finished.
    slotQueueCommand("setSBFOutput,Stream3,"+mSerialPortOnDeviceUsb+",ExtEvent,OnChange");

    // We want to know what time it is
    // Update: No, we don't. Re-setting system-time in times of high system loads does NOT improve
    // the system clock's accuracy. Offsets of ~700ms(!) have been observed. So, skip this for now.
    //slotQueueCommand("setSBFOutput,Stream4,"+mSerialPortOnDeviceCom+",ReceiverTime,sec30");

    // For now, record support messages for septentrio
    slotQueueCommand("setSBFOutput,Stream5,"+mSerialPortOnDeviceUsb+",Support,sec2"); // septentrio wants msec100, but that kills the cpu

    // BBSamples contains data for the spectrum view, but Vim (not the editor, the RF-guy of Septentrio fame) said this is a VERY heavy packet.
    // So we skip this for now to keep the CPU load low (it was over 80%!)
    //slotQueueCommand("setSBFOutput,Stream6,"+mSerialPortOnDeviceUsb+",BBSamples,sec1"); // septentrio wants msec100, but that kills the cpu

    // Needed for septentrio to debug IMU problems (ExtSensorMeas+AttEuler) - and for me to analyze sensor platform vibrations
    slotQueueCommand("setSBFOutput,Stream7,"+mSerialPortOnDeviceUsb+",ExtSensorMeas,msec20");

    // To get signal strength (Carrier over Noise)
    slotQueueCommand("setSBFOutput,Stream8,"+mSerialPortOnDeviceUsb+",MeasEpoch,sec1");

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

      2013-04-23: It SEEMS(!), that when using msec20, the 4 IMU-integrated poses can
      drift off badly, whereas the single IMU-integrated pose in msec50 seems quite precise.

      Because the INS only poses in msec20 are unusable for both sensorfusion and flight-
      control, we use msec50. Flightcontrol works with 10Hz, but maybe 20Hz is even better.
    */

    if(highSpeed)
    {
        // Maybe use msec50 instead of msec20 to avoid RxError 64 (congestion on line)
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec50");
    }
    else
    {
        slotQueueCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec100");
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
    // Long ago, we wanted to truly shutdown the receiver. Now, we just want to slow down its output,
    // so it can be re-used with the same configuration and pre-initialized filters. When kopter-
    // control restarts, we can just increase the frequency again and be done with it.

    qDebug() << "GnssDevice::slotShutDown(): starting shutdown sequence.";

    slotSetPoseFrequency(false);

    //    slotQueueCommand("setComSettings,all,baud115200,bits8,No,bit1,none");
    //    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceUsb+",none");
    //    slotQueueCommand("setSBFOutput,all,"+mSerialPortOnDeviceCom+",none");
    //    slotQueueCommand("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime");
    //    slotQueueCommand("setDataInOut,all,CMD,none");
    //    slotQueueCommand("setPVTMode,Rover,StandAlone");

    // This is still needed to acknowledge the shutdown.
    slotQueueCommand("shutdown");

    qDebug() << "GnssDevice::slotShutDown(): shutdown sequence processed, waiting for asynchronous shutdown confirmation...";
}

bool GnssDevice::parseCommandReply(const QString& portNameOnDevice, const QByteArray* const receiveBuffer, quint32* const dataCursorToAdvance)
{
    if(!mLastCommandToGnssDevice[portNameOnDevice].isEmpty())
    {
        //qDebug() << "GnssDevice::parseCommandReply(): last command to" << portNameOnDevice << "was" << mLastCommandToGnssDevice[portNameOnDevice] << "looking for reply in recBuf of" << receiveBuffer->size() << "bytes, cursor is at" << *dataCursorToAdvance << "and following bytes are:" << SbfParser::readable(receiveBuffer->mid(*dataCursorToAdvance));
        // Check for command replies before every packet
        const qint32 positionReplyStart = receiveBuffer->indexOf("$R", *dataCursorToAdvance);
        qint32 positionReplyStop  = receiveBuffer->indexOf(portNameOnDevice + QChar('>'), *dataCursorToAdvance);
        //if(positionReplyStart != -1 && positionReplyStop != -1)
        //qDebug() << "GnssDevice::parseCommandReply(): replyStart" << positionReplyStart << "stop" << positionReplyStop;
        if(positionReplyStart == *dataCursorToAdvance && positionReplyStop != -1)
        {
            positionReplyStop += portNameOnDevice.length() + 1; // make sure we also include the "USB1>" at the end!
            const QByteArray commandReply = receiveBuffer->mid(positionReplyStart, positionReplyStop - positionReplyStart);

            qDebug() << "GnssDevice::parseCommandReply(): port" << portNameOnDevice << "received reply to:" << mLastCommandToGnssDevice[portNameOnDevice].trimmed() << "-" << commandReply.size() << "bytes:" << commandReply.trimmed();

            QByteArray text;
            QTextStream commandLog(&text);
            commandLog << QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz") << portNameOnDevice << "-> HOST: " << commandReply.trimmed() << '\n';
            commandLog << '\n' << "################################################################################" << '\n' << '\n' << '\n' << '\n';
            commandLog.flush();
            mLogFileCmd->write(text);

            if(commandReply.contains("$R? ASCII commands between prompts were discarded!"))
                qDebug() << "GnssDevice::parseCommandReply(): WARNING, we were talking too fast on port" << portNameOnDevice;

            if(commandReply.contains(QString("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3,SBF").toLatin1()))
            {
                qDebug() << "GnssDevice::parseCommandReply(): gnss device now configured to accept diffcorr";
                mGnssDeviceIsConfigured = true;
            }

            if(commandReply.contains("shutdown"))
            {
                emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Orderly shutdown confirmed by gnss device");
                qDebug() << "GnssDevice::parseCommandReply(): shutdown confirmed by device, quitting.";
                QCoreApplication::quit();
            }

//            const int bytesToCut = positionReplyStop - positionReplyStart;
//            receiveBuffer->remove(positionReplyStart, bytesToCut);
            *dataCursorToAdvance += (positionReplyStop - positionReplyStart); // move the cursor forward by the length of the processed reply.
            mLastCommandToGnssDevice[portNameOnDevice].clear();

            slotFlushCommandQueue();

            return true;
        }
    }

    return false;
}

void GnssDevice::slotDataReadyOnCom()
{
//    qDebug() << __PRETTY_FUNCTION__ << mSerialPortCom->bytesAvailable() << "bytes available. We need" << mMinimumDataPacketSize;
    quint32 offsetToValidPacket;

    if(mSerialPortCom->bytesAvailable() < 24) return; // ReceiverTime has only 24!

    while(true)
    {
        // Move all new bytes into our SBF buffer
        mReceiveBufferCom.append(mSerialPortCom->readAll());

        //qDebug() << __PRETTY_FUNCTION__ << "com buffer contains:" << mReceiveBufferCom;

        // If we're waiting for a command-reply, try to find and parse it. When done, advance the dataCursor.
        while(parseCommandReply(mSerialPortOnDeviceCom, &mReceiveBufferCom, &mDataCursorCom));

        // Process as much SBF as possible.
        if(mSbfParser->getNextValidPacketInfo(mReceiveBufferCom, mDataCursorCom, &offsetToValidPacket))
        {
            if(mDataCursorCom != offsetToValidPacket)
            {
                qDebug() << __PRETTY_FUNCTION__ << "dataCursor is at" << mDataCursorCom << "but valid packet starts at non-zero" << offsetToValidPacket << "- they shouldn't differ!";
                mDataCursorCom = offsetToValidPacket;
            }

            // Process the SBF
            const quint32 bytesProcessed = mSbfParser->processNextValidPacket(mReceiveBufferCom, offsetToValidPacket);

            // Log the processed packet
            mLogFileSbf->write(mReceiveBufferCom.data() + offsetToValidPacket, bytesProcessed);

            // And advance the data pointer
            mDataCursorCom += bytesProcessed;

            if(mDataCursorCom > mMaximumDataBufferSize - 4096)
            {
                qDebug() << __PRETTY_FUNCTION__ << "processed" << mDataCursorCom << "bytes, cutting begining of array...";
                mReceiveBufferCom.remove(0, mDataCursorCom);
                mDataCursorCom = 0;
            }
        }
        else
        {
            //qDebug() << "GnssDevice::slotDataReadyOnCom():" << mReceiveBufferCom.size() - mDataCursorCom << "bytes left after parsing, but this is no valid packet yet:" << SbfParser::readable(mReceiveBufferCom);
            return;
        }
    }
}

void GnssDevice::slotDataReadyOnUsb()
{
    //qDebug() << __PRETTY_FUNCTION__;
    quint32 offsetToValidPacket;

    if(mSerialPortUsb->bytesAvailable() < mMinimumDataPacketSize) return;

    while(true)
    {
        // Move all new bytes into our SBF buffer
        mReceiveBufferUsb.append(mSerialPortUsb->readAll());

        // If we're waiting for a command-reply, try to find and parse it. When done, advance the dataCursor.
        while(parseCommandReply(mSerialPortOnDeviceUsb, &mReceiveBufferUsb, &mDataCursorUsb));

        // Process as much SBF as possible.
        if(mSbfParser->getNextValidPacketInfo(mReceiveBufferUsb, mDataCursorUsb, &offsetToValidPacket))
        {
            if(mDataCursorUsb != offsetToValidPacket)
            {
                qDebug() << __PRETTY_FUNCTION__ << "dataCursor is at" << mDataCursorUsb << "but valid packet starts at non-zero" << offsetToValidPacket << "- they shouldn't differ!";
                mDataCursorUsb = offsetToValidPacket;
            }

            // Process the SBF
            //qDebug() << __PRETTY_FUNCTION__ << "processing sbf starting at" << offsetToValidPacket << "of" << mReceiveBufferUsb.size();
            const quint32 bytesProcessed = mSbfParser->processNextValidPacket(mReceiveBufferUsb, offsetToValidPacket);

            // Log the processed packet
            mLogFileSbf->write(mReceiveBufferUsb.data() + offsetToValidPacket, bytesProcessed);

            // And advance the data pointer
            mDataCursorUsb += bytesProcessed;

            if(mDataCursorUsb > mMaximumDataBufferSize - 4096)
            {
                qDebug() << __PRETTY_FUNCTION__ << "processed" << mDataCursorUsb << "bytes, cutting begining of array...";
                mReceiveBufferUsb.remove(0, mDataCursorUsb);
                mDataCursorUsb = 0;
            }
        }
        else
        {
            //qDebug() << "GnssDevice::slotDataReadyOnUsb():" << mReceiveBufferUsb.size() - mDataCursorUsb << "bytes left after parsing, but this is no valid packet yet:" << SbfParser::readable(mReceiveBufferUsb.mid(mDataCursorUsb));
            return;
        }
    }
}

void GnssDevice::slotSetDifferentialCorrections(const QByteArray* const differentialCorrections)
{
    if(mGnssDeviceIsConfigured)
    {
        // simply write the RTK data into the com-port
        mDiffCorrDataCounter += differentialCorrections->size();
        qDebug() << "GnssDevice::slotSetDifferentialCorrections(): forwarding" << differentialCorrections->size() << "bytes of diffcorr to GNSS device, total is" << mDiffCorrDataCounter/1048576.0f << "mb";
        mSerialPortCom->write(*differentialCorrections);
        //        emit message(
        //                Information,
        //                "GnssDevice::slotSetDifferentialCorrections()",
        //                QString("Fed %1 bytes of RTK data into rover GNSS device.").arg(data.size())
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
    // The Host uses UTC time, but currentTow() applies the leap-second offset (currently 16s).
    const qint32 currentHostTow = GnssTime::currentTow();

    // First, what time is it now?
    struct timeval system;
    gettimeofday(&system, NULL);

    // SecondsPerWeek - CurrentSecondInWeek is number of seconds till rollover
    const quint32 secondsToRollOver = (7 * 86400) - (tow / 1000);

    const qint32 offsetHostToGnss = tow - currentHostTow + 7; // Oscilloscope indicates 7ms offset is a good value.
    qDebug() << "GnssDevice::slotSetSystemTime(): time rollover in" << ((float)secondsToRollOver)/86400.0f << "d, offset host time" << currentHostTow << "to gnss time" << tow << "is" << offsetHostToGnss/1000 << "s and" << (offsetHostToGnss%1000) << "ms";

    // For small clock drifts, adjust clock. Else, set clock
    if(abs(offsetHostToGnss) < 10)
    {
        qDebug() << __PRETTY_FUNCTION__ << "unix-time now is" << QDateTime::currentMSecsSinceEpoch() / 1000 << "- offset smaller than 10ms, using adjtime to correct clock drift...";
        system.tv_sec = 0;
        system.tv_usec = offsetHostToGnss * 1000;
        if(adjtime(&system, NULL) < 0)
        {
            qDebug() << __PRETTY_FUNCTION__ << "adjtime failed, tried to set to" << system.tv_sec << "s and" << system.tv_usec << "usecs";
            if(errno == EINVAL) qFatal("couldn't adjust system time, values invalid.");
            else if(errno == EPERM) qFatal("couldn't adjust system time, insufficient permissions.");
            else qFatal("couldn't adjust system time, no idea why, error %d.", errno);
        }
    }
    else
    {
        qDebug() << "GnssDevice::slotSetSystemTime(): unix-time now is" << QDateTime::currentMSecsSinceEpoch() / 1000 << "- offset larger than 10ms, using settimeofday to set clock...";
        system.tv_sec += offsetHostToGnss/1000;
        system.tv_usec += (offsetHostToGnss%1000)*1000;

        // usec can under/overflow; fix it
        if(system.tv_usec > 1000000)
        {
            qDebug() << __PRETTY_FUNCTION__ << "system.tv" << system.tv_sec << "system.tv_usec > 1000000:" << system.tv_usec << "subtracting 1000000";
            system.tv_usec -= 1000000;
            system.tv_sec += 1;
            qDebug() << __PRETTY_FUNCTION__ << "system.tv" << system.tv_sec << "system.tv_usec" << system.tv_usec;
        }

        if(settimeofday(&system, NULL) < 0)
        {
            qDebug() << __PRETTY_FUNCTION__ << "settimeofday failed, tried to set to" << system.tv_sec << "s and" << system.tv_usec << "usecs";
            if(errno == EFAULT) qFatal("couldn't set system time, values outside of range.");
            else if(errno == EINVAL) qFatal("couldn't set system time, values invalid.");
            else if(errno == EPERM) qFatal("couldn't set system time, insufficient permissions.");
            else qFatal("couldn't set system time, no idea why, error %d.", errno);
        }
    }

    qDebug() << __PRETTY_FUNCTION__ << "time synchronized, offset host to gnss was" << offsetHostToGnss << "ms, unix-time now is" << QDateTime::currentMSecsSinceEpoch() / 1000;
    emit systemTimeSynchronized();
}
