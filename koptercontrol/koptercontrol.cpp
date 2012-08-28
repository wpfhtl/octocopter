#include "koptercontrol.h"
#include <math.h>

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
    QString logFilePrefix = QString("log/kopterlog-%1-%2-")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss"))
            .arg(QString::number(QCoreApplication::applicationPid()));
    const QStringList args = QCoreApplication::arguments();
    if(args.size() == 2) logFilePrefix.append(QString("%1-").arg(args.last()));
    qDebug() << "KopterControl::KopterControl(): logfile prefix is" << logFilePrefix;

    mMessageHandler = new MessageHandler(logFilePrefix);

    QString networkInterface = "wlan0";
    QString deviceCamera = "/dev/video0";
    QString deviceSerialKopter = "/dev/ttyUSB0";
    QString deviceSerialGnssCom = "/dev/ttyUSB1";
    QString deviceSerialGnssUsb = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if00"; //"/dev/ttyACM0";
    QString deviceSerialLaserScanner = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"; // "/dev/ttyACM1";

    QStringList commandLine = arguments();

    if(commandLine.lastIndexOf("-dc") != -1 && commandLine.size() > commandLine.lastIndexOf("-dc") + 1)
    {
        deviceCamera = commandLine.at(commandLine.lastIndexOf("-dc") + 1);
    }

    if(commandLine.lastIndexOf("-sk") != -1 && commandLine.size() > commandLine.lastIndexOf("-sk") + 1)
    {
        deviceSerialKopter = commandLine.at(commandLine.lastIndexOf("-sk") + 1);
    }

    if(commandLine.lastIndexOf("-sl") != -1 && commandLine.size() > commandLine.lastIndexOf("-sl") + 1)
    {
        deviceSerialLaserScanner = commandLine.at(commandLine.lastIndexOf("-sl") + 1);
    }

    if(commandLine.lastIndexOf("-sgu") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgu") + 1)
    {
        deviceSerialGnssUsb = commandLine.at(commandLine.lastIndexOf("-sgu") + 1);
    }

    if(commandLine.lastIndexOf("-sgc") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgc") + 1)
    {
        deviceSerialGnssCom = commandLine.at(commandLine.lastIndexOf("-sgc") + 1);
    }

    if(commandLine.lastIndexOf("-netiface") != -1 && commandLine.size() > commandLine.lastIndexOf("-netiface") + 1)
    {
        networkInterface = commandLine.at(commandLine.lastIndexOf("-netiface") + 1).toLower();
    }

    qDebug() << "KopterControl::KopterControl(): using serial ports: kopter" << deviceSerialKopter << "gnss com" << deviceSerialGnssCom << "gnss usb" << deviceSerialGnssUsb;
    qDebug() << "KopterControl::KopterControl(): using laserscanner at" << deviceSerialLaserScanner;
    qDebug() << "KopterControl::KopterControl(): reading RSSI at interface" << networkInterface;

    mFlightController = new FlightController(logFilePrefix);
    mLaserScanner = new LaserScanner(
                deviceSerialLaserScanner,
                Pose(
                    QVector3D(      // Offset from Antenna to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.09,      // From antenna positive is right to laser
                        -0.38,      // From Antenna negative is down to laser
                        -0.11),     // From Antenna negative forward to laser
                    +000.0,         // No yawing
                    -090.0,         // 90 deg pitched down
                    +000.0,         // No rolling
                    10             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                    ),
                logFilePrefix
                );
    mGnssDevice = new GnssDevice(deviceSerialGnssUsb, deviceSerialGnssCom, logFilePrefix, this);
    mSensorFuser = new SensorFuser(1); // Really lo-res data for septentrio postprocessing tests.
    mSensorFuser->setLaserScannerRelativePose(mLaserScanner->getRelativePose());

    mBaseConnection = new BaseConnection(networkInterface);
    mKopter = new Kopter(deviceSerialKopter, this);

    // For testing motion with a joystick from basestation
    //connect(mBaseConnection, SIGNAL(motion(quint8,qint8,qint8,qint8,qint8)), mKopter, SLOT(slotSetMotion(quint8,qint8,qint8,qint8,qint8)));

    connect(mLaserScanner, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mKopter, SIGNAL(vehicleStatus(const VehicleStatus* const)), mBaseConnection, SLOT(slotNewVehicleStatus(const VehicleStatus* const)));
    connect(mKopter, SIGNAL(flightStateSwitchValueChanged(const FlightStateSwitch* const)), mFlightController, SLOT(slotFlightStateSwitchValueChanged(const FlightStateSwitch* const)));
    connect(mKopter, SIGNAL(calibrationSwitchToggled()), mFlightController, SLOT(slotCalibrateImu()));

    connect(mLaserScanner, SIGNAL(heightOverGround(const float)), mFlightController, SLOT(slotSetHeightOverGround(const float)));
    connect(mBaseConnection, SIGNAL(enableScanning(const bool)), mLaserScanner, SLOT(slotEnableScanning(const bool)));
    connect(mBaseConnection, SIGNAL(differentialCorrections(const QByteArray*const)), mGnssDevice, SLOT(slotSetDifferentialCorrections(const QByteArray* const)));
    connect(mBaseConnection, SIGNAL(wayPointInsert(quint16, WayPoint)), mFlightController, SLOT(slotWayPointInsert(quint16, WayPoint)));
    connect(mBaseConnection, SIGNAL(wayPointDelete(quint16)), mFlightController, SLOT(slotWayPointDelete(quint16)));
    connect(mBaseConnection, SIGNAL(wayPoints(QList<WayPoint>)), mFlightController, SLOT(slotSetWayPoints(QList<WayPoint>)));
    connect(mBaseConnection, SIGNAL(newConnection()), mFlightController, SLOT(slotEmitFlightControllerInfo()));
    connect(mBaseConnection, SIGNAL(controllerWeights(const QString* const,const QMap<QString,float>* const)), mFlightController, SLOT(slotSetControllerWeights(const QString* const,const QMap<QString,float>* const)));

    connect(mGnssDevice->getSbfParser(), SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mGnssDevice, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(gnssTimeOfWeekEstablished(qint32)), mLaserScanner, SLOT(slotSetScannerTimeStamp(qint32)));

    connect(mGnssDevice->getSbfParser(),SIGNAL(status(const GnssStatus* const)), mBaseConnection, SLOT(slotNewGnssStatus(const GnssStatus* const)));

    // distribute poses from gnssdevice
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePose(const Pose* const)), mFlightController, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePoseSensorFuser(const Pose* const)), mSensorFuser, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(newVehiclePoseStatus(const Pose* const)), mBaseConnection, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(scanFinished(quint32)), mSensorFuser, SLOT(slotScanFinished(quint32)));
    connect(mGnssDevice->getSbfParser(), SIGNAL(gnssDeviceWorkingPrecisely(bool)), mLaserScanner, SLOT(slotEnableScanning(bool)));
    connect(mLaserScanner, SIGNAL(newScanData(qint32, std::vector<long>*const)), mSensorFuser, SLOT(slotNewScanData(qint32,std::vector<long>*const)));
    connect(mSensorFuser, SIGNAL(newScannedPoints(const QVector<QVector3D>* const, const QVector3D* const)), mBaseConnection, SLOT(slotNewScannedPoints(const QVector<QVector3D>* const, const QVector3D* const)));

    // Lots of traffic - for what?
    connect(mFlightController, SIGNAL(flightControllerValues(const FlightControllerValues* const)), mBaseConnection, SLOT(slotNewFlightControllerValues(const FlightControllerValues* const)));

    connect(mFlightController, SIGNAL(wayPointReached(WayPoint)), mBaseConnection, SLOT(slotWayPointReached(WayPoint)));
    connect(mFlightController, SIGNAL(wayPointInserted(quint16,WayPoint)), mBaseConnection, SLOT(slotRoverWayPointInserted(quint16,WayPoint)));
    connect(mFlightController, SIGNAL(currentWayPoints(QList<WayPoint>)), mBaseConnection, SLOT(slotFlightControllerWayPointsChanged(QList<WayPoint>)));
    connect(mFlightController, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mFlightController, SIGNAL(flightControllerWeightsChanged()), mBaseConnection, SLOT(slotFlightControllerWeightsChanged()));

    //    WARNING! THIS ENABLES MOTION!
    connect(mFlightController, SIGNAL(motion(const MotionCommand* const)), mKopter, SLOT(slotSetMotion(const MotionCommand* const)));
}

KopterControl::~KopterControl()
{
    qDebug() << "KopterControl::~KopterControl(): shutting down, deleting objects.";
    delete mGnssDevice;
    delete mLaserScanner;
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
    static int abortCounter = 0;
    abortCounter++;
    char a = 1;
    qDebug() << "KopterControl::signalHandler(): received signal" << signal;
    ::write(signalFd[0], &a, sizeof(a));

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
    mLaserScanner->slotEnableScanning(false);

    // When mGnssDevice finishes device-shutdown, it will quit(), starting the rest of the shutdown sequence.
    mGnssDevice->slotShutDown();
}

int main(int argc, char **argv)
{
    if(getuid() != 0) qFatal("I must be run as root so I can synchronize system time to GNSS time. Bye.");

    setupUnixSignalHandlers();

    KopterControl KopterControl(argc, argv);

    return KopterControl.exec();
}
