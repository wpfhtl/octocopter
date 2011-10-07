#ifndef KOPTERCONTROL_H
#define KOPTERCONTROL_H

#include <sys/socket.h>
#include <signal.h>

#include <QCoreApplication>
#include <QList>
#include <QTimer>
#include <QStringList>

#include "kopter.h"
#include "camera.h"
#include "gpsdevice.h"
#include "laserscanner.h"
#include "baseconnection.h"
#include "flightcontroller.h"

class KopterControl : public QCoreApplication
{
    Q_OBJECT

public:
    KopterControl(int argc, char **argv);
    ~KopterControl(void);

    // Unix signal handler
    static void signalHandler(int unused);

public slots:
    void slotHandleSignal();

private:
    // We use this pipe for all signals.
    static int signalFd[2];
    Kopter *mKopter;
    Camera* mCamera;
    FlightController *mFlightController;
    GpsDevice *mGpsDevice;
    LaserScanner *mLaserScanner;
    BaseConnection *mBaseConnection;

    QSocketNotifier *snSignalPipe;

    QTimer *mTimerComputeMotion;

private slots:
    void slotDoSomething();
};

#endif
