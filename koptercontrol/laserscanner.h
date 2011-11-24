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

    QFile* mLogFileScanData;     // for unprocessed laserscanner values

    bool mIsEnabled;
    quint64 mNumberOfScannedPoints;

    qint64 mOffsetTimeScannerToTow;

    QTimer* mTimerScan; // this calls capture() when needed;

    qrk::UrgCtrl mScanner;

    QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16 scannerIndex, const float& distance) const;

private slots:
    void slotSimulateScanning();

    void slotCaptureScanData();

public:
    LaserScanner(const QString &deviceFileName);

    ~LaserScanner();

    const bool isScanning() const;
    const Pose& getRelativePose() const;

    // This depends on the laserscanner's pose relative to the vehicle, but is hardcoded ATM.
    const float getHeightAboveGround() const;

    // This is defined in header to make it inlineable
    inline QVector3D LaserScanner::getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16 scannerIndex, const float& distance) const
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
    void slotEnableScanning(const bool&);
    
    // To set the laserscanner's timestamp to the gps time. Hopefully.
    void slotSetScannerTimeStamp(const quint32& timestamp);

signals:
    void bottomBeamLength(const float&);
    void message(const LogImportance, const QString& source, const QString &text);

    void newScanData(quint32, std::vector<long>);


};

#endif
