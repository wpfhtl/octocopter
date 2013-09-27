#include "koptercontrol.h"
#include <math.h>

// for renicing in main()
#include <sys/resource.h>

int KopterControl::signalFd[] = {0,0};

void setupUnixSignalHandlers()
{
    // Set up all signals to call KopterControl::signalHandler()
    struct sigaction intr, hup, term;

    intr.sa_handler = KopterControl::signalHandler;
    sigemptyset(&intr.sa_mask);
    intr.sa_flags = 0;
    intr.sa_flags |= SA_RESTART;

    if(sigaction(SIGINT, &intr, 0) != 0) qFatal("Couldn't set up signal handler for SIGINT");

    hup.sa_handler = KopterControl::signalHandler;
    sigemptyset(&hup.sa_mask);
    hup.sa_flags = 0;
    hup.sa_flags |= SA_RESTART;

    if(sigaction(SIGHUP, &hup, 0) != 0) qFatal("Couldn't set up signal handler for SIGHUP");

    term.sa_handler = KopterControl::signalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;

    if(sigaction(SIGTERM, &term, 0) != 0) qFatal("Couldn't set up signal handler for SIGTERM");
}

KopterControl::KopterControl(int argc, char **argv) : QCoreApplication(argc, argv)
{
    // Set up signal handling
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signalFd)) qFatal("Couldn't create INT socketpair");

    snSignalPipe = new QSocketNotifier(signalFd[1], QSocketNotifier::Read, this);
    connect(snSignalPipe, SIGNAL(activated(int)), SLOT(slotHandleSignal()));

    // Make sure logging dir exists
    if(!QDir::current().mkpath("log"))
        qFatal("KopterControl::KopterControl(): couldn't create log/ subdirectory, please do it for me!");

    // Create a logfile-prefix
    QString logFilePrefix = QString("log/kopterlog-%1-%2")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss"))
            .arg(QString::number(QCoreApplication::applicationPid()));
    const QStringList args = QCoreApplication::arguments();
    if(args.size() == 2) logFilePrefix.append(QString("-%1").arg(args.last()));
    qDebug() << "KopterControl::KopterControl(): logfile prefix is" << logFilePrefix;

    mMessageHandler = new MessageHandler(logFilePrefix);

    mTimestampStartup = QDateTime::currentDateTime();

    QString networkInterface = "wlan0";
    QString deviceCamera = "/dev/video0";
    QString deviceSerialKopter = "/dev/serial/by-id/usb-FTDI_Dual_RS232-if00-port0";
    QString deviceSerialGnssPort1 = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if00"; // used for one-port mode
    QString deviceSerialGnssPort2 = "/dev/serial/by-id/usb-FTDI_Dual_RS232-if01-port0"; // used for one-port mode
    //QString deviceSerialGnssPort1 = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if01-port0";
    //QString deviceSerialGnssPort2 = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if03-port0";
    QString deviceSerialLidarDown = "/dev/hokuyo_H1004271"; // using udev-rule and hokuyoid program
    QString deviceSerialLidarFrnt = "/dev/hokuyo_H1102762"; // using udev-rule and hokuyoid program

    QStringList commandLine = arguments();

    if(commandLine.lastIndexOf("-dc") != -1 && commandLine.size() > commandLine.lastIndexOf("-dc") + 1)
    {
        deviceCamera = commandLine.at(commandLine.lastIndexOf("-dc") + 1);
    }

    if(commandLine.lastIndexOf("-sk") != -1 && commandLine.size() > commandLine.lastIndexOf("-sk") + 1)
    {
        deviceSerialKopter = commandLine.at(commandLine.lastIndexOf("-sk") + 1);
    }

    if(commandLine.lastIndexOf("-sld") != -1 && commandLine.size() > commandLine.lastIndexOf("-sld") + 1)
    {
        deviceSerialLidarDown = commandLine.at(commandLine.lastIndexOf("-sld") + 1);
    }

    if(commandLine.lastIndexOf("-slf") != -1 && commandLine.size() > commandLine.lastIndexOf("-slf") + 1)
    {
        deviceSerialLidarFrnt = commandLine.at(commandLine.lastIndexOf("-slf") + 1);
    }

    if(commandLine.lastIndexOf("-sgp1") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgp1") + 1)
    {
        deviceSerialGnssPort1 = commandLine.at(commandLine.lastIndexOf("-sgp1") + 1);
    }

    if(commandLine.lastIndexOf("-sgp2") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgp2") + 1)
    {
        deviceSerialGnssPort2 = commandLine.at(commandLine.lastIndexOf("-sgp2") + 1);
    }

    if(commandLine.lastIndexOf("-netiface") != -1 && commandLine.size() > commandLine.lastIndexOf("-netiface") + 1)
    {
        networkInterface = commandLine.at(commandLine.lastIndexOf("-netiface") + 1).toLower();
    }

    qDebug() << "KopterControl::KopterControl(): using serial ports: kopter" << deviceSerialKopter << "gnss port1" << deviceSerialGnssPort1 << "gnss port2" << deviceSerialGnssPort2;
    qDebug() << "KopterControl::KopterControl(): using laserscanners down" << deviceSerialLidarDown << "and front" << deviceSerialLidarFrnt;
    qDebug() << "KopterControl::KopterControl(): reading RSSI at interface" << networkInterface;

    mFlightController = new FlightController(logFilePrefix);
    mLaserScannerDown = new LaserScanner(deviceSerialLidarDown, logFilePrefix);
    mLaserScannerDown->slotSetRelativeScannerPose(
                Pose(
                    QVector3D(      // Offset from vehicle center to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.000,      // From vehicle left/right to laser, positive is moved to right "wing"
                        -0.040,      // From vehicle up/down to laser, negative is down to laser
                        -0.120),     // From vehicle 14cm forward, towards the front arm.
                    +000.0,         // Yaw
                    -090.0,         // 90 deg pitched down
                    +000.0,         // Roll
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    )
                );

    mLaserScannerFrnt = new LaserScanner(deviceSerialLidarFrnt, logFilePrefix);
    mLaserScannerFrnt->slotSetRelativeScannerPose(
                Pose(
                    QVector3D(      // Offset from vehicle center to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.000,      // From vehicle left/right to laser, positive is moved to right "wing"
                        -0.105,      // From vehicle up/down to laser, negative is down to laser
                        -0.015),     // From vehicle 1.5cm forward, towards the front arm.
                    +000.0,         // Yaw
                    +000.0,         // Pitch
                    +180.0,         // Rolled upside down
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    )
                );

    mGnssDevice = new GnssDevice(deviceSerialGnssPort1, deviceSerialGnssPort2, logFilePrefix, this);
    mSensorFuser = new SensorFuser(1); // Really lo-res data for septentrio postprocessing tests.

    mBaseConnection = new BaseConnection(networkInterface);
    mKopter = new Kopter(deviceSerialKopter, this);

    connect(mLaserScannerDown, &LaserScanner::message, mBaseConnection, &BaseConnection::slotSendLogMessage);
    connect(mLaserScannerFrnt, &LaserScanner::message, mBaseConnection, &BaseConnection::slotSendLogMessage);
    connect(mKopter, &Kopter::vehicleStatus, mBaseConnection, &BaseConnection::slotSendVehicleStatus);
    connect(mKopter, &Kopter::flightStateRestrictionChanged, mFlightController, &FlightController::slotFlightStateRestrictionChanged);
    connect(mKopter, &Kopter::pushButtonToggled, mFlightController, &FlightController::slotLiftHoverPosition);
    connect(mKopter, &Kopter::flightSpeedChanged, mFlightController, &FlightController::slotSetFlightSpeed);
    connect(mKopter, &Kopter::fatalError, this, &KopterControl::slotFatalError);

    connect(mLaserScannerDown, &LaserScanner::distanceAtFront, mFlightController, &FlightController::slotSetHeightOverGround);
    connect(mBaseConnection, &BaseConnection::scannerState, mLaserScannerDown, &LaserScanner::slotEnableScanning);
    // disable until INS is precise
    connect(mBaseConnection, &BaseConnection::scannerState, mLaserScannerFrnt, &LaserScanner::slotEnableScanning);
    connect(mBaseConnection, &BaseConnection::differentialCorrections, mGnssDevice, &GnssDevice::slotSetDifferentialCorrections);
    connect(mBaseConnection, &BaseConnection::wayPoints, mFlightController, &FlightController::slotSetWayPoints);
    connect(mBaseConnection, &BaseConnection::newConnection, mFlightController, &FlightController::slotEmitFlightControllerInfo);
    connect(mBaseConnection, &BaseConnection::controllerWeights, mFlightController, &FlightController::slotSetControllerWeights);

    connect(mGnssDevice->getSbfParser(), &SbfParser::message, mBaseConnection, &BaseConnection::slotSendLogMessage);
    connect(mGnssDevice, &GnssDevice::message, mBaseConnection, &BaseConnection::slotSendLogMessage);

    connect(mGnssDevice, &GnssDevice::systemTimeSynchronized, mLaserScannerDown, &LaserScanner::slotSetScannerTimeStamp);
    connect(mGnssDevice, &GnssDevice::systemTimeSynchronized, mLaserScannerFrnt, &LaserScanner::slotSetScannerTimeStamp);

    connect(mGnssDevice->getSbfParser(),&SbfParser::status, mBaseConnection, &BaseConnection::slotSendGnssStatus);
    connect(mGnssDevice->getSbfParser(), &SbfParser::fatalError, this, &KopterControl::slotFatalError);

    // distribute poses from gnssdevice
    connect(mGnssDevice->getSbfParser(), &SbfParser::newVehiclePose, mFlightController, &FlightController::slotNewVehiclePose);
    connect(mGnssDevice->getSbfParser(), &SbfParser::newVehiclePoseSensorFuser, mSensorFuser, &SensorFuser::slotNewVehiclePose);
    connect(mGnssDevice->getSbfParser(), &SbfParser::newVehiclePoseStatus, mBaseConnection, &BaseConnection::slotSendVehiclePose);

    connect(mGnssDevice->getSbfParser(), &SbfParser::scanFinished, mSensorFuser, &SensorFuser::slotScanFinished);

    connect(mLaserScannerDown->getHokuyo(), &Hokuyo::scanRaw, mSensorFuser, &SensorFuser::slotNewScanRaw);
    connect(mLaserScannerFrnt->getHokuyo(), &Hokuyo::scanRaw, mSensorFuser, &SensorFuser::slotNewScanRaw);
    connect(mSensorFuser, SIGNAL(scanFused(const float*const,const quint32,const QVector3D*const)), mBaseConnection, SLOT(slotSendScanFused(const float*const,const quint32,const QVector3D*const)));

    // Lots of traffic - for what?
    connect(mFlightController, &FlightController::flightControllerValues, mBaseConnection, &BaseConnection::slotSendFlightControllerValues);
    connect(mFlightController, &FlightController::flightState, mBaseConnection, &BaseConnection::slotSendFlightState);
    connect(mFlightController, &FlightController::flightStateRestriction, mBaseConnection, &BaseConnection::slotSendFlightStateRestriction);
    connect(mFlightController, &FlightController::wayPointReached, mBaseConnection, &BaseConnection::slotSendWayPointReached);
    connect(mFlightController, &FlightController::wayPoints, mBaseConnection, &BaseConnection::slotSendWayPoints);
    connect(mFlightController, &FlightController::message, mBaseConnection, &BaseConnection::slotSendLogMessage);
    connect(mFlightController, &FlightController::flightControllerWeights, mBaseConnection, &BaseConnection::slotSendFlightControllerWeights);

    //    WARNING! THIS ENABLES MOTION!
    connect(mBaseConnection, &BaseConnection::motion, mKopter, &Kopter::slotSetMotion);
    connect(mFlightController, &FlightController::motion, mKopter, &Kopter::slotSetMotion);
}

KopterControl::~KopterControl()
{
    qDebug() << "KopterControl::~KopterControl(): shutting down, deleting objects.";
    delete mGnssDevice;
    delete mLaserScannerDown;
    delete mLaserScannerFrnt;
    delete mFlightController;
    delete mSensorFuser;
    delete mKopter;
    delete snSignalPipe;
    delete mMessageHandler;

    // Delete logfiles with a size of 0 (emtpty) or 100 (just ply header, no data)
    const QFileInfoList list = QDir().entryInfoList((QStringList() << "scannerdata-*" << "pointcloud-*"), QDir::Files | QDir::NoSymLinks);
    for(int i = 0; i < list.size(); ++i)
    {
        const QFileInfo fileInfo = list.at(i);
        if(fileInfo.size() == 0 || fileInfo.size() == 100)
        {
            qDebug() << "LaserScanner::~LaserScanner(): moving useless logfile to /tmp:" << fileInfo.fileName();
            QFile::rename(fileInfo.canonicalFilePath(), fileInfo.fileName().prepend("/tmp/"));
        }
    }
}

void KopterControl::signalHandler(int signal)
{
    qDebug() << "KopterControl::signalHandler(): received signal" << signal;
    static int abortCounter = 0;
    abortCounter++;
    char a = 1;
    // We write into the pipe, thereby calling KoperControl::slotHandleSignal() safely in its own thread.
    ::write(signalFd[0], &a, sizeof(a));

    // If the user sends more signals, lets kill it hard.
    if(abortCounter == 2)
    {
        qDebug() << "KopterControl::signalHandler(): received signal" << signal << "for" << abortCounter << "times, quit()ing.";
        QCoreApplication::quit();
    }
    else if(abortCounter > 2)
    {
        qDebug() << "KopterControl::signalHandler(): received signal" << signal << "for" << abortCounter << "times, comitting suicide now.";
        exit(1);
    }
}

void KopterControl::slotFatalError(const QString& message)
{
    qDebug() << message;

    if(mTimestampStartup.msecsTo(QDateTime::currentDateTime()) < 20000)
    {
        qFatal("Fatal error during startup, quitting.");
    }
    else
    {
        qDebug() << __PRETTY_FUNCTION__ << "fatal error during flight, prepare for the worst!!!";
        mBaseConnection->slotSendLogMessage(LogImportance::Desaster, "", message);
    }
}

void KopterControl::slotHandleSignal()
{
    snSignalPipe->setEnabled(false);
    char tmp;
    ::read(signalFd[1], &tmp, sizeof(tmp));

    qDebug() << "KopterControl::slotHandleSignal(): caught unix-signal, quitting...";

    snSignalPipe->setEnabled(true);

    // shutdown orderly
    qDebug() << "KopterControl::slotHandleSignal(): shutting down scanner...";
    mLaserScannerDown->slotEnableScanning(false);
    mLaserScannerFrnt->slotEnableScanning(false);

    // When mGnssDevice finishes device-shutdown, it will quit(), starting the rest of the shutdown sequence.
    qDebug() << "KopterControl::slotHandleSignal(): shutting down GNSS device...";
    mGnssDevice->slotShutDown();
}

int main(int argc, char **argv)
{
    if(getuid() != 0) qFatal("main(): must be run as root to synchronize system time to GNSS time.");

    // Renice
    if(setpriority(/*what*/PRIO_PROCESS, /*who*/0, /*prio*/-20) != 0)
    {
        qFatal("main(): unable to renice, quitting.");
    }

    setupUnixSignalHandlers();

    KopterControl KopterControl(argc, argv);

    return KopterControl.exec();
}
