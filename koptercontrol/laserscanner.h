#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
#include <QMatrix4x4>
#include <urg/UrgCtrl.h>
#include <pose.h>

// We don't use lidarpoints on the rover, because storing the ray back to laserscanner from
// each scanned point is a large overhead with little gain. Instead, we store a list of
// QVector3Ds (points in world frame) and also emit the mean position together with that list.

class LaserScanner : public QObject
{
    Q_OBJECT

private:
    QString mDeviceFileName;
    qrk::UrgCtrl mScanner;
    const Pose mRelativeScannerPose; // The scanner's pose relative to the vehicle frame
    bool mIsEnabled;
    qint64 mOffsetTimeScannerToTow;
    qint32 mLastScannerTimeStamp;
    QTimer* mTimerScan; // this calls capture() when needed;

private slots:
    void slotCaptureScanData();

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    LaserScanner(const QString &deviceFileName, const Pose &relativeScannerPose);

    ~LaserScanner();

    const bool isScanning() const;
    const Pose& getRelativePose() const;

    // This depends on the laserscanner's pose relative to the vehicle, but is hardcoded ATM.
    const float getHeightAboveGround() const;

    inline static quint8 getScanDuration() {return 25;}

/*
    // This is the static version of index2rad, valid only for UTM 30 LX.
    static inline float index2rad(int index)
    {
        // Algorithm is: return (2.0 * M_PI) * (index - 540) / 1440;
        // Where 540 is ray-index of front rray AFRT in SCIP protocol) and 1440 is number of rays in 360° (ARES in SCIP protocol)

        // The correct result for index 720 seems to be   0.785398163397448296348

        // When returning double, result for index 720 is 0.785398163397448279   (error 0.000000000000000017348 = 2E-17)
        //return 0.0043633231299858238686 * (index - 540);

        // When returning float , result for index 720 is 0.78539818525314331055 (error 0.000000021855695014202 = 2E-8)
        return 0.0043633231299858238686f * (index - 540);
    }
*/

public slots:
    void slotEnableScanning(const bool& = true);

    // To set the laserscanner's timestamp to the gps time. Hopefully.
    void slotSetScannerTimeStamp(const quint32& timestamp);

signals:
    void bottomBeamLength(const float&);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);

    // Emits new scan data, allocated on heap. Ownership is passed to receiver(s).
    void newScanData(quint32 timestampScanner, std::vector<long> * const distances);
};

#endif
