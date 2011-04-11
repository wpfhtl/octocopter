#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <urg/UrgCtrl.h>
#include "pose.h"
#include <stdlib.h>

#include <QtCore>
#include <QtNetwork>
#include <QQuaternion>
#include <QVector3D>

using namespace std;
using namespace qrk;

class LaserScanner : public QThread
{
    Q_OBJECT

private:
    QString mDeviceFileName;

    UrgCtrl mScanner;

    vector<long> *mScanDistancesPrevious, *mScanDistancesCurrent, *mScanDistancesNext;

    QVector<QVector3D> mPoints;

    // This scanner's Pose relative to the vehicle
    Pose mRelativePose;

    QMutex mMutex;

    // The last 4 scanner-poses in world-coordinates
    Pose *mScannerPoseFirst, *mScannerPoseBefore, *mScannerPoseAfter, *mScannerPoseLast;


private slots:
    void slotDoScan(void);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    LaserScanner(const QString &deviceFileName, const Pose &pose = Pose());

    ~LaserScanner();

    bool isScanning(void) const;

    void setPose(const Pose &pose);
    Pose getPose(void) const;

    void run(void);

public slots:
//    void slotSetScannerPose(const QVector3D &position, const QQuaternion &orientation);

    // When a laserscan is finished, the lidar changes the electircal level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // its current pose. This pose is emitted by GpsDevice. That signal is connected
    // to this slot, so we can combine this pose with our measured data into a 3d cloud.
    //
    // The @pose is created by GpsDevice and then ownership is passed to LaserScanner.
    void slotScanFinished(Pose* pose);
};

#endif
