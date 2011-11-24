#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <urg/UrgCtrl.h>
//#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>

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

    vector<long> mScanDistances;
    vector<long> mScanIntensities;

    QVector3D mPosition;
    QQuaternion mOrientation;

    QMutex mMutex;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerScan;

    // the scanner's range in meters. This abstract the function between scan-target's size and range
    float mRange, mRangeSquared;

    // how fast does the laser rotate in rounds per minute? Storing a float makes following calculations easier.
    float mSpeed;

    // 0 deg is the rear, 180deg is the front. E.g. hokuyo utm30lx goes from 45 to 315 deg.
    int mAngleStart, mAngleStop;

    // how many degrees between two rays?
    float mAngleStep;

    // current angle/status of the current scan. Valid between mAngleStart and mAngleStop
    float mCurrentScanAngle;

    std::string mMaterialName;

    double mTimeFactor;


private slots:
    void slotDoScan(void);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    LaserScanner(const QString &deviceFileName);

    ~LaserScanner();

    bool isScanning(void) const;


    // Getters for the properties
    float range(void) const;
    int speed(void) const;
    int angleStart(void) const;
    int angleStop(void) const;
    float angleStep(void) const;
    QVector3D getPosition(void);
    QQuaternion getOrientation(void);

    // Setters for the properties
    void setRange(float range);
    void setSpeed(int speed);
    void setAngleStart(int angleStart);
    void setAngleStop(int angleStop);
    void setAngleStep(float angleStep);
    void setPosition(const QVector3D &position);
    void setOrientation(const QQuaternion &orientation);

    void run(void);

public slots:
    void slotStart(void);
    void slotPause(void);
    void slotSetScannerPose(const QVector3D &position, const QQuaternion &orientation);
};

#endif
