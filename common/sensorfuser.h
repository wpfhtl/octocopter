#ifndef SENSORFUSER_H
#define SENSORFUSER_H

#include <QtCore>
#include "pose.h"
#include "rawscan.h"

class SensorFuser : public QObject
{
    Q_OBJECT

public:
    SensorFuser(const quint8& stridePoint = 1, const quint8& strideScan = 1);
    ~SensorFuser();

    enum class InterpolationMethod
    {
        NearestNeighbor,
        Linear,
        Cubic
    };

    void setInterpolationMethod(const InterpolationMethod im) {mBestInterpolationMethodToUse = im;}

    void setMaximumFusableRayLength(const float& rayLength) {mMaximumFusableRayLength = rayLength;}

    quint8 getStridePoint(void) const {return mStridePoint;}
    void setStridePoint(quint8 stridePoint) {mStridePoint = stridePoint;}

    quint8 getStrideScan(void) const {return mStrideScan;}
    void setStrideScan(quint8 strideScan) {mStrideScan = strideScan;}

private:
    struct MaximumFusionTimeOffset
    {
        // These values assume 20 Hz (50ms interval) pose rate. Higher frequencies should work flawlessly!
        #define POSE_INTERVAL 100
        static const quint16 NearestNeighbor = POSE_INTERVAL / 2 + 1; // 21
        static const quint16 Linear = ((POSE_INTERVAL / 2) * 3) + 1; // 61
        static const quint16 Cubic = (POSE_INTERVAL * 2.5) + 1; // 91
    };


    // for debugging/logging
    QFile* mPoseDynamicsLogFile;
    QTextStream* mPoseDynamicsStream;

    // appends the resulting point to mRegisteredPoints
    void fuseRayWithLastInterpolatedPose(const qint16 index, const float& distance);

    InterpolationMethod mBestInterpolationMethodToUse;

    quint8 mStridePoint, mStrideScan;

    quint32 mNumberOfScansWithMissingGnssTimestamps;

    Pose mLastInterpolatedPose;

    QMap<InterpolationMethod,quint16> mStatsScansFused;
    quint32 mStatsScansDiscarded;

    bool mFlushRemainingData;

    // Maximum clock offset between scanner and gnss device timestamp in miliseconds. We will always find the best-fitting scan,
    // and that SHOULD always be the correct match. This is more of a safeguard to prevent matching scans with a gnss timestamp
    // seconds away from each other.
    qint32 mMaximumTimeOffsetBetweenScannerAndGnss;

    float mMaximumFusableRayLength; // rays longer than this will be ignored (because they introduce too much error due to orientation errors)

    qint32 mNewestDataTime; // used to clear data older than X ms
    qint32 mLastScanMiddleGnssTow; // the last time the GNSS told us about the TOW of a scanSweepMiddle.

    // All incoming poses will be registered in this vector
    QList<Pose*> mPoses;

    // All incoming gnss timestamps will be registered in this vector and then augmented into mScanInformation
    QList<qint32> mGnssTimeStamps;

    // All incoming scans will be registered in this vector of structs
    QList<RawScan*> mRawScans;

    // These two save the last registration results, which are emitted and processed by others
    float* mRegisteredPoints;
    quint32 mNumberOfPointsFusedInThisScan;
    QVector3D mLastScannerPosition;

    // This method moves gnss timestamps to their place in mScanInformation
    qint8 matchTimestamps();

    // This method uses mSavedScans and mSavedPoses to create and emit world-cooridnate scanpoints.
    void fuseScans();

    // Cleans old scans, poses and gnss timestamps
    void cleanUnusableData();

    void emitLastInterpolatedPose();

public slots:
    // Used to send a new vehicle pose to SensorFuser. You must guarantee that the poses are
    // supplied in chronological order!
    // Ownership stays with caller, as the const implies
    void slotNewVehiclePose(const Pose *const pose);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gnss-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened. We expect the time of the MIDDLE
    // ray! You must guarantee that the timestamps are supplied in chronological order
    void slotScanFinished(const quint32& timestampScanGnss);

    // Used to feed data from the laserscanner. You must guarantee that the scans are supplied
    // in chronological order!
    void slotNewRawScan(RawScan* scan);

    // Clears all poses, gnss timestamps and scans. This is used by LogPlayer when seeking backwards.
    // If it didn't clean our data, there'd be no guarantee data comes in in chronological order.
    void slotClearData(const qint32 maximumDataAge = -1);

    // Called when all data has been submitted, so that all leftover scans are fused, using worse
    // interpolation if poses are insufficient.
    void slotFlushData();

signals:
    // Emits a pointer to a vector of registered points. The data is always owned by SensorFuser!
//    void newScannedPoints(const QVector<QVector3D>* const, const QVector3D* const scanPosition);
    void scanFused(const float* const points, const quint32& numberOfPoints, const QVector3D* const scannerPosition);

    // For debugging/visualizing the interpolated poses
    void vehiclePose(const Pose* const);
};

#endif // SENSORFUSER_H
