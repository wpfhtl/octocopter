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
                        +0.00,      // From vehicle left/right to laser, positive is moved to right "wing"
                        -0.04,      // From vehicle up/down to laser, negative is down to laser
                        -0.14),     // From vehicle 14cm forward, towards the front arm.
                    +000.0,         // No yawing
                    -091.0,         // 90 deg pitched down
                    +000.0,         // No rolling
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    )
                );

    mLaserScannerFrnt = new LaserScanner(deviceSerialLidarFrnt, logFilePrefix);
    mLaserScannerFrnt->slotSetRelativeScannerPose(
                Pose(
                    QVector3D(      // Offset from vehicle center to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        0.00,      // From vehicle left/right to laser, positive is moved to right "wing"
                        0.10,      // From vehicle up/down to laser, negative is down to laser
                        0.00),     // From vehicle 14cm forward, towards the front arm.
                    0.0,         // No yawing
                    0.0,         // No pitching
                    0.0,         // No rolling
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    )
                );

    mGnssDevice = new GnssDevice(deviceSerialGnssPort1, deviceSerialGnssPort2, logFilePrefix, this);
    mSensorFuser = new SensorFuser(1); // Really lo-res data for septentrio postprocessing tests.

    mBaseConnection = new BaseConnection(networkInterface);
    mKopter = new Kopter(deviceSerialKopter, this);

    // For testing motion with a joystick from basestation
    //connect(mBaseConnection, SIGNAL(motion(quint8,qint8,qint8,qint8,qint8)), mKopter, SLOT(slotSetMotion(quint8,qint8,qint8,qint8,qint8)));

    connect(mLaserScannerDown, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mLaserScannerFrnt, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mKopter, SIGNAL(vehicleStatus(const VehicleStatus* const)), mBaseConnection, SLOT(slotNewVehicleStatus(const VehicleStatus* const)));
    connect(mKopter, SIGNAL(flightStateSwitchValueChanged(const FlightStateSwitch* const)), mFlightController, SLOT(slotFlightStateSwitchValueChanged(const FlightStateSwitch* const)));
    connect(mKopter, SIGNAL(pushButtonToggled()), mFlightController, SLOT(slotLiftHoverPosition()));
    connect(mKopter, SIGNAL(flightSpeedChanged(float)), mFlightController, SLOT(slotSetFlightSpeed(float)));

    connect(mLaserScannerDown, SIGNAL(heightOverGround(const float)), mFlightController, SLOT(slotSetHeightOverGround(const float)));
    connect(mBaseConnection, SIGNAL(enableScanning(const bool)), mLaserScannerDown, SLOT(slotEnableScanning(const bool)));
    connect(mBaseConnection, SIGNAL(enableScanning(const bool)), mLaserScannerFrnt, SLOT(slotEnableScanning(const bool)));
    connect(mBaseConnection, SIGNAL(differentialCorrections(const QByteArray*const)), mGnssDevice, SLOT(slotSetDifferentialCorrections(const QByteArray* const)));
    connect(mBaseConnection, SIGNAL(wayPoints(QList<WayPoint>,WayPointListSource)), mFlightController, SLOT(slotSetWayPoints(QList<WayPoint>,WayPointListSource)));
    connect(mBaseConnection, SIGNAL(newConnection()), mFlightController, SLOT(slotEmitFlightControllerInfo()));
    connect(mBaseConnection, SIGNAL(controllerWeights(const QString* const,const QMap<QChar,float>* const)), mFlightController, SLOT(slotSetControllerWeights(const QString* const,const QMap<QChar,float>* const)));

    connect(mGnssDevice->getSbfParser(), SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mGnssDevice, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mGnssDevice, SIGNAL(systemTimeSynchronized()), mLaserScannerDown, SLOT(slotSetScannerTimeStamp()));
    connect(mGnssDevice, SIGNAL(systemTimeSynchronized()), mLaserScannerFrnt, SLOT(slotSetScannerTimeStamp()));

    connect(mGnssDevice->getSbfParser(),SIGNAL(status(const GnssStatus* const)), mBaseConnection, SLOT(slotNewGnssStatus(const GnssStatus* const)));

    // distribute poses from gnssdevice
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePose(const Pose* const)), mFlightController, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePoseSensorFuser(const Pose* const)), mSensorFuser, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePoseStatus(const Pose* const)), mBaseConnection, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(scanFinished(quint32)), mSensorFuser, SLOT(slotScanFinished(quint32)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(gnssDeviceWorkingPrecisely(bool)), mLaserScannerDown, SLOT(slotEnableScanning(bool)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(gnssDeviceWorkingPrecisely(bool)), mLaserScannerFrnt, SLOT(slotEnableScanning(bool)));
    connect(mLaserScannerDown, SIGNAL(scanData(qint32,Pose*const,quint16*,quint16)), mSensorFuser, SLOT(slotNewScanData(qint32,Pose*const,quint16*,quint16)));
    connect(mLaserScannerFrnt, SIGNAL(scanData(qint32,Pose*const,quint16*,quint16)), mSensorFuser, SLOT(slotNewScanData(qint32,Pose*const,quint16*,quint16)));
    connect(mSensorFuser, SIGNAL(scanData(float*const,quint32,QVector3D*const)), mBaseConnection, SLOT(slotNewScannedPoints(float*const,quint32,QVector3D*const)));

    // Lots of traffic - for what?
    connect(mFlightController, SIGNAL(flightControllerValues(const FlightControllerValues* const)), mBaseConnection, SLOT(slotNewFlightControllerValues(const FlightControllerValues* const)));
    connect(mFlightController, SIGNAL(flightStateChanged(FlightState*const)), mBaseConnection, SLOT(slotFlightStateChanged(FlightState*const)));
    connect(mFlightController, SIGNAL(wayPointReached(WayPoint)), mBaseConnection, SLOT(slotWayPointReached(WayPoint)));
    connect(mFlightController, SIGNAL(wayPoints(QList<WayPoint>* const, WayPointListSource)), mBaseConnection, SLOT(slotSetWayPoints(QList<WayPoint> *const, WayPointListSource)));
    connect(mFlightController, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mFlightController, SIGNAL(flightControllerWeightsChanged()), mBaseConnection, SLOT(slotFlightControllerWeightsChanged()));

    //    WARNING! THIS ENABLES MOTION!
    connect(mBaseConnection, SIGNAL(motion(const MotionCommand* const)), mKopter, SLOT(slotSetMotion(const MotionCommand* const)));
    connect(mFlightController, SIGNAL(motion(const MotionCommand* const)), mKopter, SLOT(slotSetMotion(const MotionCommand* const)));
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
