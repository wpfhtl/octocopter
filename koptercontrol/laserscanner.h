#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
#include <pose.h>
#include "logfile.h"
#include "hokuyo.h"

// We don't use lidarpoints on the rover, because storing the ray back to laserscanner from
// each scanned point is a large overhead with little gain. Instead, we store a list of
// QVector3Ds (points in world frame) and also emit the mean position together with that list.

class LaserScanner : public QObject
{
    Q_OBJECT

private:
    static quint32 instanceCounter;
    QThread* mThreadReadScanner;
    QString mDeviceFileName;
    Hokuyo* mHokuyo;
    QMatrix4x4 mRelativeScannerPose; // The scanner's pose relative to the vehicle frame
    qint64 mOffsetTimeScannerToTow;
    qint32 mLastScannerTimeStamp;
    LogFile* mLogFile;

    std::vector<long> mScannedDistances;

    // slotSetRelativeScannerPose() cannot write the pose directly into the LogFile as an RPOSE entry,
    // because it is called on startup, when the TOW is still unknown and the system time isn't synced
    // yet. Thus, it caches the relative scanner pose (which needs to be done anyway for Hokyuo to emit
    // meaningful ScanRaws). Then, when the time is known (in slotSetScannerTimeStamp), we write it to
    // the logile using this method. Whew.
    void writeRelativeScannerPoseToLogFile();

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    LaserScanner(const QString &deviceFileName, const QString& logFilePrefix, bool isConnectedToEventPin = false);

    ~LaserScanner();

    Hokuyo* getHokuyo() const {return mHokuyo;}

public slots:
    void slotEnableScanning(const bool = true);
    void slotSetRelativeScannerPose(const Pose& p);

    // To set the laserscanner's timestamp to the gps time. Hopefully.
    void slotSetScannerTimeStamp();

    void slotThreadStarted();
    void slotThreadFinished();

signals:
    // the distance from the vehicle's center to the ground in meters
    void distanceAtFront(const float&);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);
};

#endif
