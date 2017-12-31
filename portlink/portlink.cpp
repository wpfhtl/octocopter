#include "portlink.h"
#include <unistd.h>

int PortLink::signalFd[] = {0,0};

void setupUnixSignalHandlers()
{
    // Set up all signals to call PortLink::signalHandler()
    struct sigaction intr, hup, term;

    intr.sa_handler = PortLink::signalHandler;
    sigemptyset(&intr.sa_mask);
    intr.sa_flags = 0;
    intr.sa_flags |= SA_RESTART;

    if(sigaction(SIGINT, &intr, 0) != 0) qFatal("Couldn't set up signal handler for SIGINT");

    hup.sa_handler = PortLink::signalHandler;
    sigemptyset(&hup.sa_mask);
    hup.sa_flags = 0;
    hup.sa_flags |= SA_RESTART;

    if(sigaction(SIGHUP, &hup, 0) != 0) qFatal("Couldn't set up signal handler for SIGHUP");

    term.sa_handler = PortLink::signalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;

    if(sigaction(SIGTERM, &term, 0) != 0) qFatal("Couldn't set up signal handler for SIGTERM");
}

PortLink::PortLink(int argc, char **argv) : QCoreApplication(argc, argv)
{
    // set up signal handling
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signalFd)) qFatal("Couldn't create INT socketpair");

    snSignalPipe = new QSocketNotifier(signalFd[1], QSocketNotifier::Read, this);
    connect(snSignalPipe, SIGNAL(activated(int)), SLOT(slotHandleSignal()));

    quint8 numberOfPorts = 0;
    QStringList commandLine = arguments();

    // First argument is our binary's name, discard it.
    commandLine.removeAt(0);

    qDebug() << "PortLink::PortLink(): startup, creating connections...";

    while(commandLine.size() >= 2)
    {
//        qDebug() << "cmdline is:" << commandLine.join(" ");

        const QString argument = commandLine.takeFirst();

        if(argument.compare("-s", Qt::CaseInsensitive) == 0)
        {
            Port* port;
            // There might be a port-settings-string like 115200,8,n,1 after the serial device file
            const QString deviceFile = commandLine.takeFirst();

            if(commandLine.size() && commandLine.first().left(1) != "-")
                port = new SerialPort(deviceFile, commandLine.takeFirst());
            else
                port = new SerialPort(deviceFile);

            mPortList.append(port);
            numberOfPorts++;
        }
        else if(argument.compare("-nc", Qt::CaseInsensitive) == 0)
        {
            Port* port = new NetworkClient(commandLine.takeFirst());
            mPortList.append(port);
            numberOfPorts++;
        }
        else if(argument.compare("-ns", Qt::CaseInsensitive) == 0)
        {
            Port* port = new NetworkServer(commandLine.takeFirst());
            mPortList.append(port);
            numberOfPorts += 2; // We increment by two to disable the error based on this value below. One server can have many clients...
        }
        else
        {
            qWarning("PortLink::PortLink(): I don't understand argument %s, ignoring.", qPrintable(argument));
        }
    }

    while(commandLine.size()) qWarning("PortLink::PortLink(): Ignoring leftover argument %s.", qPrintable(commandLine.takeFirst()));

    if(numberOfPorts < 2)
    {
        qDebug() << "PortLink::PortLink(): less than two ports in use, kinda useless for forwarding data. Exiting.";
        abort();
    }

    for(int i = 0; i < mPortList.size(); i++)
    {
        Port* port = mPortList.at(i);

        // connect this ports data() signal to all other ports write() slots.
        for(int j = 0; j < mPortList.size(); j++)
        {
            if(i != j)
            {
                connect(port, SIGNAL(data(QByteArray)), mPortList.at(j), SLOT(write(QByteArray)));
            }
        }
    }

    qDebug() << "PortLink::PortLink(): data routing set up, entering event loop.";
}

PortLink::~PortLink()
{
    qDebug() << "PortLink::~PortLink(): deleting objects, shutting down.";
    while(mPortList.size())
    {
        delete mPortList.takeFirst();
    }

    delete snSignalPipe;
}

void PortLink::signalHandler(int signal)
{
    char a = 1;
    qDebug() << "PortLink::signalHandler(): received signal" << signal;
    ::write(signalFd[0], &a, sizeof(a));
}

void PortLink::slotHandleSignal()
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

    PortLink pl(argc, argv);

    return pl.exec();
}
