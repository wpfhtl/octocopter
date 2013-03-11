#include "filewriter.h"

FileWriter::FileWriter(int argc, char **argv) : QCoreApplication(argc, argv)
{
    QStringList commandLine = arguments();

    // First argument is our binary's name, discard it.
    commandLine.removeAt(0);

    // To write multiple files, specify how many bytes per interval shall be written
    // ./filewriter bytes1 interval1 [bytes2 interval2]
    
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

void FileWriter::printUsage()
{
    qDebug() << "usage: ./filewriter bytes1 interval1 [bytes2 interval2]";
    QCoreApplication::quit();
}

int main(int argc, char **argv)
{
    FileWriter fw(argc, argv);

    return fw.exec();
}
