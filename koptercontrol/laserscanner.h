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
//    LogFile* mLogFile;
    static quint32 instanceCounter;
    QThread* mThreadReadScanner;
    QString mDeviceFileName;
    Hokuyo* mHokuyo;
    Pose mRelativeScannerPose; // The scanner's pose relative to the vehicle frame
    qint64 mOffsetTimeScannerToTow;
    qint32 mLastScannerTimeStamp;
    LogFile* mLogFile;

    std::vector<long> mScannedDistances;

private slots:
    void slotNewScanData(RawScan *rawScan);

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    LaserScanner(const QString &deviceFileName, const QString& logFilePrefix);

    ~LaserScanner();

    const bool isScanning() const;
    const Pose& getRelativePose() const;

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

    // Emits new scan data, allocated on heap. Ownership (of the distances only!) is passed to receiver(s).
    void scanData(RawScan*);
};

#endif
