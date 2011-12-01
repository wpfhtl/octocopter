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

    inline quint8 getScanDuration() {return 25;} const

    // This is defined in header to make it inlineable
    inline static QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16& scannerIndex, const float& distance)
    {
#ifdef false
        // Version using QMatrix4x4
        const QVector3D vectorScannerToPoint(
                    sin(-0.0043633231299858238686f * (scannerIndex - 540)) * distance,  // X in meters
                    0.0,                                                                // Y always 0
                    cos(-0.0043633231299858238686f * (scannerIndex - 540)) * distance); // Z in meters

        QMatrix4x4 scannerOrientation;
        scannerOrientation.rotate(scannerPose.getYawDegrees(), QVector3D(0,1,0));
        scannerOrientation.rotate(scannerPose.getPitchDegrees(), QVector3D(1,0,0));
        scannerOrientation.rotate(scannerPose.getRollDegrees(), QVector3D(0,0,1));

        return scannerPose.position + scannerOrientation * vectorScannerToPoint;
#else
        // This is short, but uses getOrientation(), which is expensive.
        return QVector3D(
                    scannerPose.position
                    + scannerPose.getOrientation().rotatedVector(
                        QVector3D(
                            sin(-0.0043633231299858238686f * (scannerIndex - 540)) * distance,  // X in meters
                            0.0,                                                                // Y always 0
                            cos(-0.0043633231299858238686f * (scannerIndex - 540)) * distance   // Z in meters
                            )
                        )
                    );
#endif

        /* Elaborate version, slightly slower?!

        // Determine vector from LaserScanner to scanned point, using the normal OpenGl coordinate system as seen from scanner,
        // +x is right, -x is left, y is always 0, +z is back, -z is front,
        // We could use the UrgCtrl::index2rad(), but this seems a bit more optimized
        const QVector3D vectorScannerToPoint(
                    sin(-index2rad(scannerIndex)) * distance,  // X in meters
                    0.0,                                                // Y always 0
                    cos(-index2rad(scannerIndex)) * distance); // Z in meters

        // Or, even more optimized, as we cannot rely on gcc actually inlining what it marked as inline


        // Create a scanpoint
        const QVector3D scannedPoint(scannerPose.position + scannerPose.getOrientation().rotatedVector(vectorScannerToPoint));

    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): interpolated scanner pose is" << scannerPose;
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): distance is" << distance << "and index" << scannerIndex << "=>" << mScanner.index2deg(scannerIndex) << "degrees";
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): scanner to point in scanner frame is" << vectorScannerToPoint;
    //    qDebug() << "LaserScanner::getWorldPositionOfScannedPoint(): point position   in world   frame is" << scannedPoint;

        return scannedPoint;
        */
    }
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
