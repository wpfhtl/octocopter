#include "filewriter.h"

int FileWriter::signalFd[] = {0,0};

void setupUnixSignalHandlers()
{
    // Set up all signals to call FileWriter::signalHandler()
    struct sigaction intr, hup, term;

    intr.sa_handler = FileWriter::signalHandler;
    sigemptyset(&intr.sa_mask);
    intr.sa_flags = 0;
    intr.sa_flags |= SA_RESTART;

    if(sigaction(SIGINT, &intr, 0) != 0) qFatal("Couldn't set up signal handler for SIGINT");

    hup.sa_handler = FileWriter::signalHandler;
    sigemptyset(&hup.sa_mask);
    hup.sa_flags = 0;
    hup.sa_flags |= SA_RESTART;

    if(sigaction(SIGHUP, &hup, 0) != 0) qFatal("Couldn't set up signal handler for SIGHUP");

    term.sa_handler = FileWriter::signalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;

    if(sigaction(SIGTERM, &term, 0) != 0) qFatal("Couldn't set up signal handler for SIGTERM");
}

FileWriter::FileWriter(int argc, char **argv) : QCoreApplication(argc, argv)
{
    QStringList commandLine = arguments();

    // First argument is our binary's name, discard it.
    commandLine.removeAt(0);

    // To write multiple files, specify how many bytes per interval shall be written
    // ./filewriter bytes1 interval1 [bytes2 interval2]

    // Set up signal handling
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signalFd)) qFatal("Couldn't create INT socketpair");
    snSignalPipe = new QSocketNotifier(signalFd[1], QSocketNotifier::Read, this);
    connect(snSignalPipe, SIGNAL(activated(int)), SLOT(slotHandleSignal()));

    if(commandLine.size() < 2) printUsage();

    qDebug() << "FileWriter::FileWriter(): startup, creating file writers...";

    while(commandLine.size() >= 2)
    {
//        qDebug() << "cmdline is:" << commandLine.join(" ");

        bool ok;
        const quint32 numBytes = commandLine.takeFirst().toInt(&ok);
        if(!ok) printUsage();

        const quint32 interval = commandLine.takeFirst().toInt(&ok);
        if(!ok) printUsage();

        mFileVector.append(new FileToWrite(interval, numBytes));
    }

    while(commandLine.size()) qWarning("FileWriter::FileWriter(): Ignoring leftover argument %s.", qPrintable(commandLine.takeFirst()));

    qDebug() << "FileWriter::FileWriter():" << mFileVector.size() << "files set up, starting write";
}

FileWriter::~FileWriter()
{
    qDebug() << "FileWriter::~FileWriter(): deleting objects, shutting down.";
    while(mFileVector.size())
    {
        delete mFileVector.at(mFileVector.size()-1);
        mFileVector.remove(mFileVector.size()-1);
    }
}


void FileWriter::signalHandler(int signal)
{
    static int abortCounter = 0;
    abortCounter++;
    char a = 1;
    qDebug() << "FileWriter::signalHandler(): received signal" << signal;
    ::write(signalFd[0], &a, sizeof(a));

    if(abortCounter == 2)
    {
        qDebug() << "FileWriter::signalHandler(): received signal" << signal << "for" << abortCounter << "times, quit()ing.";
        QCoreApplication::quit();
    }
    else if(abortCounter > 2)
    {
        qDebug() << "FileWriter::signalHandler(): received signal" << signal << "for" << abortCounter << "times, comitting suicide now.";
        exit(1);
    }
}

void FileWriter::slotHandleSignal()
{
    snSignalPipe->setEnabled(false);
    char tmp;
    ::read(signalFd[1], &tmp, sizeof(tmp));

    qDebug() << "FileWriter::slotHandleSignal(): caught unix-signal, quitting...";

    snSignalPipe->setEnabled(true);
    QCoreApplication::quit();
}

void FileWriter::printUsage()
{
    qDebug() << "usage: ./filewriter bytes1 interval1 [bytes2 interval2]";
    exit(-1);
}

int main(int argc, char **argv)
{
    setupUnixSignalHandlers();

    FileWriter fw(argc, argv);

    return fw.exec();
}
