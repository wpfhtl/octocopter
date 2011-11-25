#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
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

    inline quint8 getScanDuration() {return 25;} const

    // This is defined in header to make it inlineable
    inline QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16& scannerIndex, const float& distance) const
    {
        // Determine vector from LaserScanner to scanned point, using the normal OpenGl coordinate system as seen from scanner,
        // +x is right, -x is left, y is always 0, +z is back, -z is front,
        const QVector3D vectorScannerToPoint(
                    sin(-mScanner.index2rad(scannerIndex)) * distance,  // X in meters
                    0.0,                                                // Y always 0
                    cos(-mScanner.index2rad(scannerIndex)) * distance); // Z in meters, zero when pointing forward.

        // Create a scanpoint
        const QVector3D scannedPoint(scannerPose.position + scannerPose.getOrientation().rotatedVector(vectorScannerToPoint));

    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): interpolated scanner pose is" << scannerPose;
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): distance is" << distance << "and index" << scannerIndex << "=>" << mScanner.index2deg(scannerIndex) << "degrees";
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): scanner to point in scanner frame is" << vectorScannerToPoint;
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): point position   in world   frame is" << scannedPoint;

        return scannedPoint;
    }

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
