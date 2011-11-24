#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <urg/UrgCtrl.h>
#include <stdlib.h>

#include <sys/time.h>

#include <QtCore>
#include <QString>

using namespace std;
using namespace qrk;

class LaserScanner : public QObject
{
    Q_OBJECT

private:
    QString mDeviceFileName;

    UrgCtrl mScanner;

    vector<long> mScanDistances;
    vector<long> mScanIntensities;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerScan;

    QDateTime mStartUpTime;

    QString getTime();
    void setTime(const quint32& time);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    LaserScanner(const QString &deviceFileName);

    ~LaserScanner();

    void run(void);
};

#endif
