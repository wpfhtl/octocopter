#ifndef PortLink_H
#define PortLink_H

#include <sys/socket.h>
#include <signal.h>

#include <QCoreApplication>
#include <QList>
#include <QTimer>
#include <QStringList>

#include "port.h"
#include "serialport.h"
#include "networkserver.h"
#include "networkclient.h"

class PortLink : public QCoreApplication
{
    Q_OBJECT

public:
    PortLink(int argc, char **argv);
    ~PortLink(void);

    // Unix signal handler
    static void signalHandler(int unused);

public slots:
    void slotHandleSignal();

private:
    QList<Port*> mPortList;

    // We use this pipe for all signals.
    static int signalFd[2];
    QSocketNotifier *snSignalPipe;

private slots:

};

#endif
