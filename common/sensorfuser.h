#ifndef SENSORFUSER_H
#define SENSORFUSER_H

#include <QtCore>
#include "pose.h"

#define quint32_max 4294967295
#define qint32_max 2147483647

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

    void setInterpolationmethod(const InterpolationMethod im) {mBestInterpolationMethodToUse = im;}

    void setMaximumFusableRayLength(const float& rayLength) {mMaximumFusableRayLength = rayLength;}

    void setLaserScannerRelativePose(const Pose& pose)
    {
        mLaserScannerRelativePose = pose;

        // We can assign perfect precision, as it will be
        // ANDed with the vehicle pose's precision lateron.
        mLaserScannerRelativePose.precision = 255;
        mLaserScannerRelativePose.covariances = 0.0f;
    }

    quint8 getStridePoint(void) const {return mStridePoint;}
    void setStridePoint(quint8 stridePoint) {mStridePoint = stridePoint;}

    quint8 getStrideScan(void) const {return mStrideScan;}
    void setStrideScan(quint8 strideScan) {mStrideScan = strideScan;}

    struct ScanInformation
    {
        std::vector<long>* ranges;
        qint32 timeStampScanMiddleGnss;
        qint32 timeStampScanMiddleScanner;

        ScanInformation()
        {
            ranges = 0;
            timeStampScanMiddleGnss = 0;
            timeStampScanMiddleScanner = 0;
        }
    };

private:
    struct MaximumFusionTimeOffset
    {
        // These values assume 50 Hz pose rate. That might be problematic for older logdata!
//        static const quint8 NearestNeighbor = 10;
//        static const quint8 Linear = 20;
//        static const quint8 Cubic = 41;

        // These values assume 25 Hz pose rate. Higher frequencies should work flawlessly!
        static const quint8 NearestNeighbor = 21;
        static const quint8 Linear = 61;
        static const quint8 Cubic = 91;
    };

    // appends the resulting point to mRegisteredPoints
    void fuseRay(const Pose &pose, const qint16 index, const float& distance);

    InterpolationMethod mBestInterpolationMethodToUse;

    quint8 mStridePoint, mStrideScan;

    //
    Pose mLaserScannerRelativePose;

    QMap<InterpolationMethod,quint16> mStatsScansFused;
    quint16 mStatsScansDiscarded;

    // How much time difference from a scan to a pose (or vice versa) in past or future for the data to be usable for interpolation?
    quint8 mMaximumTimeOffsetBetweenFusedPoseAndScanMsec;

    // Maximum clock offset between scanner and gnss device timestamp in miliseconds. Smaller means less data, more means worse data.
    qint32 mMaximumTimeOffsetBetweenScannerAndGnss;

    float mMaximumFusableRayLength; // rays longer than this will be ignored (because they introduce too much error due to orientation errors)

    qint32 mNewestDataTime; // used to clear data older than X ms
    qint32 mLastScanMiddleGnssTow; // the last time the GNSS told us about the TOW of a scanSweepMiddle.

    // All incoming poses will be registered in this vector
    QVector<Pose> mPoses;

    // All incoming gnss timestamps will be registered in this vector and then augmented into mScanInformation
    QVector<qint32> mGnssTimeStamps;

    // All incoming scans will be registered in this vector of structs, together with their timeStampScanner
    QVector<ScanInformation> mScanInformation;

    // These two save the last registration results, which are emitted and processed by others
    QVector<QVector3D> mRegisteredPoints;
    QVector3D mLastScannerPosition;

    // This method moves gnss timestamps to their place in mScanInformation
    qint8 matchTimestamps();

    // This method uses mSavedScans and mSavedPoses to create and emit world-cooridnate scanpoints.
    void fuseScans();
    void transformScanDataNearestNeighbor();

    // Cleans old data (mSaved, scanGps and mSavedPoses
    void cleanUnusableData();

    // Used for debugging only.
    inline const QStringList getTimeStamps(const QMap<qint32, std::vector<long>* >& list) const
    {
        QStringList timeStamps;
        QMap<qint32, std::vector<long>* >::const_iterator itr = list.constBegin();
        while(itr != list.constEnd())
        {
            // append an "e" for empty after entries with a 0-pointer as the value
            if(itr.value() == 0)
                timeStamps << QString::number((uint)itr.key()).append('e');
            else
                timeStamps << QString::number((uint)itr.key());

            ++itr;
        }
        return timeStamps;
    }

    // Used for debugging only.
    inline const QStringList getTimeStamps(const QList<Pose>& list) const
    {
        QStringList timeStamps;

        for(int i = 0; i < list.size(); ++i) timeStamps << QString::number((uint)list.at(i).timestamp);

        return timeStamps;
    }

public slots:
    // The pose also contains a timestamp (receiver-time) of when that pose was recorded.
    // Ownership stays with caller, as the const implies
    void slotNewVehiclePose(const Pose *const pose);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gnss-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened. We expect the time of the MIDDLE
    // ray!
    void slotScanFinished(const quint32& timestampScanGnss);

    // Used to feed data from the laserscanner
    void slotNewScanData(const qint32& timestampScanner, std::vector<long> * const distances);

signals:
    // Emits a pointer to a vector of registered points. The data is always owned by SensorFuser!
    void newScannedPoints(const QVector<QVector3D>* const, const QVector3D* const scanPosition);
};

#endif // SENSORFUSER_H
