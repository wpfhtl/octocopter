#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
#include <urg/UrgCtrl.h>
#include <pose.h>

// We don't use lidarpoints on the rover, because storing the ray back to laserscanner from
// each scanned point is a large overhead with little gain. Instead, we store a list of
// QVector3Ds (points in world frame) and also emit the mean position together with that list.
// #include <lidarpoint.h>

class LaserScanner : public QObject
{
    Q_OBJECT

private:
    QString mDeviceFileName;

    QFile* mLogFileDataRaw;     // poses and unprocessed laserscanner values
    QFile* mLogFileDataGlobal;  // poses and scanned points in global frame (processed)

    bool mIsEnabled;
    quint64 mNumberOfScannedPoints;

    qrk::UrgCtrl mScanner;

    std::vector<long> *mScanDistancesPrevious, *scanDistances, *mScanDistancesNext;

    // This scanner's pose relative to the vehicle frame
    Pose mRelativePose;

    QMutex mMutex;

    quint32 mLastTimeOfPoseOrScan;

    // The last 4 scanner-poses in world-coordinates (vehicleFrame + relativeScannerPose),
    // used for cubic interpolation.
    Pose *mScannerPoseFirst, *mScannerPoseBefore, *mScannerPoseAfter, *mScannerPoseLast;
    // OR, ALTERNATIVELY, an associative map of the last N timestamp => scannerposes
    QList<Pose> mSavedPoses;

    QMap<quint32, std::vector<long> > mSavedScans;

    QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16 scannerIndex, const float& distance) const;

    void transformScanData();

private slots:
    void slotSimulateScanning();
    void slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points);

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    LaserScanner(const QString &deviceFileName, const Pose &pose);

    ~LaserScanner();

    bool isScanning(void) const;

    // This depends on the laserscanner's pose relative to the vehicle, but is hardcoded ATM.
    float getHeightAboveGround() const;

public slots:
    void slotEnableScanning(const bool&);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened.
    void slotScanFinished(const quint32& timestamp);

    // The pose also contains a timestamp (receiver-time) of when that pose was recorded.
    void slotNewVehiclePose(const Pose& pose);

signals:
    void bottomBeamLength(const float&);

    void message(const LogImportance, const QString& source, const QString &text);

    void newScannedPoints(const QVector3D& scanPosition, const QVector<QVector3D>&);
};

#endif
