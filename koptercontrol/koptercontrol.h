#ifndef KOPTERCONTROL_H
#define KOPTERCONTROL_H

#include <sys/socket.h>
#include <signal.h>
#include <unistd.h> // getuid()
#include <sys/types.h> // getuid()

#include <QCoreApplication>
#include <QList>
#include <QTimer>
#include <QDir>
#include <QStringList>

#include "kopter.h"
#include "camera.h"
#include "gpsdevice.h"
#include "sensorfuser.h"
#include "laserscanner.h"
//#include "visualodometry.h"
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
    SensorFuser* mSensorFuser;
    FlightController *mFlightController;
    GpsDevice *mGpsDevice;
    LaserScanner *mLaserScanner;
    BaseConnection *mBaseConnection;
//    VisualOdometry* mVisualOdometry;

    QSocketNotifier *snSignalPipe;

    QTimer *mTimerComputeMotion;

private slots:
    void slotDoSomething();
};

#endif
