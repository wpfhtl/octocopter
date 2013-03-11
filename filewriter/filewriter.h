#ifndef FILEWRITER_H
#define FILEWRITER_H

#include <sys/socket.h>
#include <signal.h>

#include "filetowrite.h"

#include <QCoreApplication>
#include <QSocketNotifier>
#include <QVector>
#include <QDebug>
#include <QStringList>

class FileWriter : public QCoreApplication
{
    Q_OBJECT

    QVector<FileToWrite*> mFileVector;
    void printUsage();

    QSocketNotifier *snSignalPipe;

    // We use this pipe for all signals.
    static int signalFd[2];

public slots:
    void slotHandleSignal();

public:
    FileWriter(int argc, char **argv);
    ~FileWriter(void);

    // Unix signal handler
    static void signalHandler(int unused);
};

#endif
