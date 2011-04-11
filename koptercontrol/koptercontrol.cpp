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

    QString portSerialKopter = "/dev/ttyUSB1";
    QString portSerialGpsCom = "/dev/ttyUSB0";
    QString portSerialGpsUsb = "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device-if00"; //"/dev/ttyACM0";
    QString portSerialLaserScanner = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"; // "/dev/ttyACM1";

    QString rtkBaseHostName = "134.100.13.202";
    int rtkBasePort = 1234;

        QStringList commandLine = arguments();

        if(commandLine.lastIndexOf("-sk") != -1 && commandLine.size() > commandLine.lastIndexOf("-sk") + 1)
        {
            portSerialKopter = commandLine.at(commandLine.lastIndexOf("-sk") + 1);
        }

        if(commandLine.lastIndexOf("-sl") != -1 && commandLine.size() > commandLine.lastIndexOf("-sl") + 1)
        {
            portSerialLaserScanner = commandLine.at(commandLine.lastIndexOf("-sl") + 1);
        }

        if(commandLine.lastIndexOf("-sgu") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgu") + 1)
        {
            portSerialGpsUsb = commandLine.at(commandLine.lastIndexOf("-sgu") + 1);
        }

        if(commandLine.lastIndexOf("-sgc") != -1 && commandLine.size() > commandLine.lastIndexOf("-sgc") + 1)
        {
            portSerialGpsCom = commandLine.at(commandLine.lastIndexOf("-sgc") + 1);
        }

        if(commandLine.lastIndexOf("-rtkhostname") != -1 && commandLine.size() > commandLine.lastIndexOf("-rtkhostname") + 1)
        {
            rtkBaseHostName = commandLine.at(commandLine.lastIndexOf("-rtkhostname") + 1);
        }

        if(commandLine.lastIndexOf("-rtkhostport") != -1 && commandLine.size() > commandLine.lastIndexOf("-rtkhostport") + 1)
        {
            rtkBasePort = commandLine.at(commandLine.lastIndexOf("-rtkhostport") + 1).toInt();
        }

        qDebug() << "KopterControl::KopterControl(): using serial ports: kopter" << portSerialKopter << "gps com" << portSerialGpsCom << "gps usb" << portSerialGpsUsb;
        qDebug() << "KopterControl::KopterControl(): using rtk base at" << rtkBaseHostName << rtkBasePort;
        qDebug() << "KopterControl::KopterControl(): using laserscanner at" << portSerialLaserScanner;

        mKopter = new Kopter(portSerialKopter, this);
        mGpsDevice = new GpsDevice(portSerialGpsUsb, portSerialGpsCom, this);
        mRtkFetcher = new RtkFetcher(rtkBaseHostName, rtkBasePort, this);
        mLaserScanner = new LaserScanner(portSerialLaserScanner, Pose());

        QTimer *ben = new QTimer(this);
        ben->setInterval(50);
//        ben->start();
        connect(ben, SIGNAL(timeout()), SLOT(slotDoSomething()));

        mKopter->slotSubscribeDebugValues(100);

        connect(mKopter, SIGNAL(externControlReplyReceived()), SLOT(slotDoSomething()));
        mKopter->slotSetMotion(fabs(sin(0.00))*40, 0, 0, 0, 0);
}

KopterControl::~KopterControl()
{
    qDebug() << "KopterControl::~KopterControl(): deleting objects, shutting down.";
    delete mKopter;
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
    qDebug() << "setting thrust to" << fabs(sin(wert)*40);
    mKopter->slotSetMotion(fabs(sin(wert))*40, 0, 0, 0, 0);
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
     quit();
}

int main(int argc, char **argv)
{
    setupUnixSignalHandlers();

    KopterControl KopterControl(argc, argv);

    return KopterControl.exec();
}
