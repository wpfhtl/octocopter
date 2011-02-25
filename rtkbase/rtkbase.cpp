#include "rtkbase.h"

int RtkBase::signalFd[] = {0,0};

void setup_unix_signal_handlers()
{
    // Set up all signals to call RtkBase::signalHandler()
    struct sigaction intr, hup, term;

    intr.sa_handler = RtkBase::signalHandler;
    sigemptyset(&intr.sa_mask);
    intr.sa_flags = 0;
    intr.sa_flags |= SA_RESTART;

    if(sigaction(SIGINT, &intr, 0) != 0) qFatal("Couldn't set up signal handler for SIGINT");

    hup.sa_handler = RtkBase::signalHandler;
    sigemptyset(&hup.sa_mask);
    hup.sa_flags = 0;
    hup.sa_flags |= SA_RESTART;

    if(sigaction(SIGHUP, &hup, 0) != 0) qFatal("Couldn't set up signal handler for SIGHUP");

    term.sa_handler = RtkBase::signalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;

    if(sigaction(SIGTERM, &term, 0) != 0) qFatal("Couldn't set up signal handler for SIGTERM");
}

RtkBase::RtkBase(int argc, char **argv) : QCoreApplication(argc, argv)
{
    // set up signal handling
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signalFd)) qFatal("Couldn't create INT socketpair");

    snSignalPipe = new QSocketNotifier(signalFd[1], QSocketNotifier::Read, this);
    connect(snSignalPipe, SIGNAL(activated(int)), SLOT(slotHandleSignal()));

	uint portNumberNetwork = 8888;
	QString portSerial = "/dev/ttyACM0";

	QStringList commandLine = arguments();

	if(commandLine.lastIndexOf("-p") != -1 && commandLine.size() > commandLine.lastIndexOf("-p") + 1)
	{
	    bool ok;
	    uint networkPortNew = commandLine.at(commandLine.lastIndexOf("-p")+1).toUInt(&ok);
	    if(ok) portNumberNetwork = networkPortNew;
	}

	if(commandLine.lastIndexOf("-s") != -1 && commandLine.size() > commandLine.lastIndexOf("-s") + 1)
	{
	    portSerial = commandLine.at(commandLine.lastIndexOf("-s") + 1);
	}

	qDebug() << "using serial port" << portSerial << "and network port" << portNumberNetwork;

	mServer = new Server(this, portNumberNetwork);
	mGpsDevice = new GpsDevice(portSerial, this);

	connect(mGpsDevice, SIGNAL(correctionDataReady(QByteArray)), mServer, SLOT(slotSendCorrectionData(QByteArray)));
}

RtkBase::~RtkBase()
{
    qDebug() << "RtkBase::~RtkBase(): deleting objects, shutting down.";
    delete mServer;
    delete mGpsDevice;
    delete snSignalPipe;
}

void RtkBase::signalHandler(int signal)
{
    char a = 1;
    qDebug() << "RtkBase::signalHandler(): received signal" << signal;
    ::write(signalFd[0], &a, sizeof(a));
}

void RtkBase::slotHandleSignal()
{
     snSignalPipe->setEnabled(false);
     char tmp;
     ::read(signalFd[1], &tmp, sizeof(tmp));

     qDebug("Unix-signal arrived, shutting down...");

     snSignalPipe->setEnabled(true);

     // shutdown orderly
     quit();
}

int main(int argc, char **argv)
{
    setup_unix_signal_handlers();
    RtkBase rtkBase(argc, argv);

    return rtkBase.exec();
}
