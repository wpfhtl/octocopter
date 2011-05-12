#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <urg/UrgCtrl.h>
#include "pose.h"
#include <stdlib.h>
#include <lidarpoint.h>

#include <QtCore>
#include <QtNetwork>
#include <QQuaternion>
#include <QVector3D>

using namespace std;
using namespace qrk;

class LaserScanner : public QObject//QThread
{
    Q_OBJECT

private:
    QString mDeviceFileName;

    bool mIsEnabled;

    UrgCtrl mScanner;
    int mMilliSecondsPerScan;

    vector<long> *mScanDistancesPrevious, *mScanDistancesCurrent, *mScanDistancesNext;

    QVector<LidarPoint> mPoints;

    // This scanner's Pose relative to the vehicle
    Pose mRelativePose;

    QMutex mMutex;

    // The last 4 scanner-poses in world-coordinates
    Pose *mScannerPoseFirst, *mScannerPoseBefore, *mScannerPoseAfter, *mScannerPoseLast;


private slots:

public:
    // Laser rotation is always CCW, angleStart < angleStop
    LaserScanner(const QString &deviceFileName, const Pose &pose = Pose());

    ~LaserScanner();

    bool isScanning(void) const;

    float getHeightAboveGround() const;

//    Pose getRelativePose(void) const;

//    void run(void);

public slots:
    void slotSetRelativePose(const Pose &pose);

    void slotEnableScanning(const bool&);

    // When a laserscan is finished, the lidar changes the electircal level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened.
    void slotScanFinished(const quint32& timestamp);

    // The @pose is created by GpsDevice and then ownership is passed to LaserScanner.
    // Pose also contains a timestamp which is set to the receiver-time of when that pose
    // was recorded.
    void slotNewVehiclePose(const Pose& pose);

signals:
    void bottomBeamLength(const float&);

    void newLidarPoints(const QVector<LidarPoint>&);
};

#endif
