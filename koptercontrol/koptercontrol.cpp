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

    mBaseConnection = new BaseConnection(networkInterface);
    mKopter = new Kopter(deviceSerialKopter, this);
    mGpsDevice = new GpsDevice(deviceSerialGpsUsb, deviceSerialGpsCom, this);
    mLaserScanner = new LaserScanner(
                deviceSerialLaserScanner,
                Pose(
                    QVector3D(      // Offset from Antenna to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                        +0.09,      // From antenna positive is right to laser
                        -0.38,      // From Antenna negative is down to laser
                        -0.11),     // From Antenna negative forward to laser
                    0.0,            // No yawing
                    -90.0,            // 90 deg pitched down
                    0.0           // No rolling
                    )
                );

    mFlightController = new FlightController();
    mCamera = new Camera(deviceCamera, QSize(320, 240), QVector3D(), QQuaternion(), 30);

    mVisualOdometry = new VisualOdometry(mCamera);

    connect(mLaserScanner, SIGNAL(message(LogImportance,QString,QString)), mBaseConnection, SLOT(slotNewLogMessage(LogImportance,QString,QString)));
    connect(mLaserScanner, SIGNAL(newScannedPoints(QVector3D,QVector<QVector3D>)), mBaseConnection, SLOT(slotNewScannedPoints(QVector3D,QVector<QVector3D>)));
    connect(mCamera, SIGNAL(imageReadyJpeg(QString,QSize,QVector3D,QQuaternion,const QByteArray*)), mBaseConnection, SLOT(slotNewCameraImage(QString,QSize,QVector3D,QQuaternion,const QByteArray*)));
    connect(mCamera, SIGNAL(imageReadyYCbCr(QString,QSize,QVector3D,QQuaternion,QByteArray)), mVisualOdometry, SLOT(slotProcessImage(QString,QSize,QVector3D,QQuaternion,QByteArray)));
    connect(mKopter, SIGNAL(kopterStatus(quint32, qint16, float)), mBaseConnection, SLOT(slotNewVehicleStatus(quint32, qint16, float)));
    connect(mLaserScanner, SIGNAL(bottomBeamLength(const float&)), mFlightController, SLOT(slotSetBottomBeamLength(const float&)));
    connect(mBaseConnection, SIGNAL(enableScanning(const bool&)), mLaserScanner, SLOT(slotEnableScanning(const bool&)));
    connect(mBaseConnection, SIGNAL(rtkDataReady(const QByteArray&)), mGpsDevice, SLOT(slotSetRtkData(const QByteArray&)));

    //    WARNING! THIS ENABLES MOTION!
    //    connect(mFlightController, SIGNAL(motion(quint8,qint8,qint8,qint8,qint8)), mKopter, SLOT(slotSetMotion(quint8,qint8,qint8,qint8,qint8)));

    connect(mGpsDevice, SIGNAL(newVehiclePose(Pose)), mFlightController, SLOT(slotSetVehiclePose(Pose)));
    connect(mGpsDevice, SIGNAL(newVehiclePose(Pose)), mLaserScanner, SLOT(slotNewVehiclePose(Pose)));
    connect(mGpsDevice, SIGNAL(newVehiclePoseLowFreq(Pose)), mBaseConnection, SLOT(slotPoseChanged(Pose)));
    connect(mGpsDevice, SIGNAL(scanFinished(quint32)), mLaserScanner, SLOT(slotScanFinished(quint32)));
    connect(
                mGpsDevice,
                SIGNAL(gpsStatus(const quint8&, const quint8&, const quint16&, const quint8&, const quint8&, const quint8&, const quint8&, const QString&)),
                mBaseConnection,
                SLOT(slotNewGpsStatus(const quint8&, const quint8&, const quint16&, const quint8&, const quint8&, const quint8&, const quint8&, const QString&))
                );

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
    qDebug() << "KopterControl::~KopterControl(): deleting objects, shutting down.";
    delete mLaserScanner;
//    delete mKopter;
    delete snSignalPipe;
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

    qDebug("Caught unix-signal, shutting down...");

    snSignalPipe->setEnabled(true);

    // shutdown orderly
    mGpsDevice->slotShutDown();
    quit();
}

int main(int argc, char **argv)
{
    setupUnixSignalHandlers();

    KopterControl KopterControl(argc, argv);

    return KopterControl.exec();
}
