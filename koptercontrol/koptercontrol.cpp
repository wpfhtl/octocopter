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
    // set up signal handling
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signalFd)) qFatal("Couldn't create INT socketpair");

    snSignalPipe = new QSocketNotifier(signalFd[1], QSocketNotifier::Read, this);
    connect(snSignalPipe, SIGNAL(activated(int)), SLOT(slotHandleSignal()));

    QString networkInterface = "wlan0";
    QString deviceCamera = "/dev/video0";
    QString deviceSerialKopter = "/dev/ttyUSB0";
    QString deviceSerialGpsCom = "/dev/ttyUSB1";
    QString deviceSerialGpsUsb = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if00"; //"/dev/ttyACM0";
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
        deviceSerialGpsUsb = commandLine.at(commandLine.lastIndexOf("-sgu") + 1);
    }

    if(commandLine.lastIndexOf("-sgc") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgc") + 1)
    {
        deviceSerialGpsCom = commandLine.at(commandLine.lastIndexOf("-sgc") + 1);
    }

    if(commandLine.lastIndexOf("-netiface") != -1 && commandLine.size() > commandLine.lastIndexOf("-netiface") + 1)
    {
        networkInterface = commandLine.at(commandLine.lastIndexOf("-netiface") + 1).toLower();
    }

    qDebug() << "KopterControl::KopterControl(): using serial ports: kopter" << deviceSerialKopter << "gps com" << deviceSerialGpsCom << "gps usb" << deviceSerialGpsUsb;
    qDebug() << "KopterControl::KopterControl(): using laserscanner at" << deviceSerialLaserScanner;
    qDebug() << "KopterControl::KopterControl(): reading RSSI at interface" << networkInterface;

    mFlightController = new FlightController();
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
                    )
                );
    mSensorFuser = new SensorFuser(mLaserScanner, static_cast<SensorFuser::Behavior>(SensorFuser::WriteRawLogs/* | SensorFuser::FuseData*/));
    mBaseConnection = new BaseConnection(networkInterface);
    mKopter = new Kopter(deviceSerialKopter, this);
    mGpsDevice = new GpsDevice(deviceSerialGpsUsb, deviceSerialGpsCom, this);

//    mCamera = new Camera(deviceCamera, QSize(320, 240), Pose(), 15);
//    mVisualOdometry = new VisualOdometry(mCamera);

//    connect(mCamera, SIGNAL(imageReadyJpeg(QString,QSize,Pose,QQuaternion,const QByteArray*)), mBaseConnection, SLOT(slotNewCameraImage(QString,QSize,Pose,const QByteArray*)));
//    connect(mCamera, SIGNAL(imageReadyYCbCr(QString,QSize,Pose,QByteArray)), mVisualOdometry, SLOT(slotProcessImage(QString,QSize,Pose,QByteArray)));
    connect(mLaserScanner, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mKopter, SIGNAL(kopterStatus(quint32, qint16, float)), mBaseConnection, SLOT(slotNewVehicleStatus(quint32, qint16, float)));
    connect(mLaserScanner, SIGNAL(bottomBeamLength(const float&)), mFlightController, SLOT(slotSetBottomBeamLength(const float&)));
    connect(mBaseConnection, SIGNAL(enableScanning(const bool&)), mLaserScanner, SLOT(slotEnableScanning(const bool&)));
    connect(mBaseConnection, SIGNAL(rtkDataReady(const QByteArray&)), mGpsDevice, SLOT(slotSetRtkData(const QByteArray&)));

    //    WARNING! THIS ENABLES MOTION!
    //    connect(mFlightController, SIGNAL(motion(quint8,qint8,qint8,qint8,qint8)), mKopter, SLOT(slotSetMotion(quint8,qint8,qint8,qint8,qint8)));

    connect(mGpsDevice->getSbfParser(), SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mGpsDevice, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));

    connect(mGpsDevice->getSbfParser(), SIGNAL(gpsTimeOfWeekEstablished(quint32)), mLaserScanner, SLOT(slotSetScannerTimeStamp(quint32)));

    connect(mGpsDevice->getSbfParser(),SIGNAL(status(GpsStatusInformation::GpsStatus)), mBaseConnection, SLOT(slotNewGpsStatus(GpsStatusInformation::GpsStatus)));

    // Feed sensor data into SensorFuser
    connect(mGpsDevice->getSbfParser(), SIGNAL(scanFinished(quint32)), mSensorFuser, SLOT(slotScanFinished(quint32)));
    connect(mGpsDevice->getSbfParser(), SIGNAL(newVehiclePose(Pose)), mFlightController, SLOT(slotNewVehiclePose(Pose)));
    connect(mGpsDevice->getSbfParser(), SIGNAL(newVehiclePoseLowFreq(Pose)), mBaseConnection, SLOT(slotNewVehiclePose(Pose)));
    connect(mGpsDevice->getSbfParser(), SIGNAL(newVehiclePoseLowFreq(Pose)), mLaserScanner, SLOT(slotEnableScanning()));
    connect(mGpsDevice->getSbfParser(), SIGNAL(newVehiclePosePrecise(Pose)), mSensorFuser, SLOT(slotNewVehiclePose(Pose)));
    connect(mLaserScanner, SIGNAL(newScanData(quint32, std::vector<long>*const)), mSensorFuser, SLOT(slotNewScanData(quint32,std::vector<long>*const)));
    connect(mSensorFuser, SIGNAL(newScannedPoints(QVector<QVector3D>,QVector3D)), mBaseConnection, SLOT(slotNewScannedPoints(QVector<QVector3D>,QVector3D)));

    mTimerComputeMotion = new QTimer(this);
    mTimerComputeMotion->setInterval(50);
    mTimerComputeMotion->start();
    connect(mTimerComputeMotion, SIGNAL(timeout()), mFlightController, SLOT(slotComputeMotionCommands()));

    mKopter->slotSubscribeDebugValues(500);

    //        connect(mKopter, SIGNAL(externControlReplyReceived()), SLOT(slotDoSomething()));
    //        mKopter->slotSetMotion(fabs(sin(0.00))*40, 0, 0, 0, 0);
}

KopterControl::~KopterControl()
{
    qDebug() << "KopterControl::~KopterControl(): shutting down, deleting objects.";
    delete mGpsDevice;
    delete mLaserScanner;
    delete mSensorFuser;
    delete snSignalPipe;

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

void KopterControl::slotDoSomething()
{
    //    QList<unsigned char> speeds;
    //    speeds << 0 << 3 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0;
    //    mKopter->slotSendMotorTest(speeds);

    static float wert = 0.0;
    wert += 0.02;

    //    mKopter->slotSetMotion(100, 0, 20, 0, 10);
    //qDebug() << "setting thrust to" << fabs(sin(wert)*40);
    qDebug() << "extern control reply received.";
    //    mKopter->slotSetMotion(fabs(sin(wert))*40, 0, 0, 0, 0);
}

void KopterControl::signalHandler(int signal)
{
    char a = 1;
    qDebug() << "KopterControl::signalHandler(): received signal" << signal;
    ::write(signalFd[0], &a, sizeof(a));
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
    mGpsDevice->slotShutDown();

//    quit();
}

int main(int argc, char **argv)
{
    setupUnixSignalHandlers();

    KopterControl KopterControl(argc, argv);

    return KopterControl.exec();
}
