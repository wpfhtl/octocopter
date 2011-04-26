#ifndef KOPTERCONTROL_H
#define KOPTERCONTROL_H

#include <sys/socket.h>
#include <signal.h>

#include <QCoreApplication>
#include <QList>
#include <QTimer>
#include <QStringList>

#include "kopter.h"
#include "gpsdevice.h"
#include "rtkfetcher.h"
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
        FlightController *mFlightController;
        GpsDevice *mGpsDevice;
        RtkFetcher *mRtkFetcher;
        LaserScanner *mLaserScanner;
        BaseConnection *mBaseConnection;

        QSocketNotifier *snSignalPipe;

       private slots:
        void slotDoSomething();
void slotNewPose(const Pose&, quint32);
};

#endif
