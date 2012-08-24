#ifndef SENSORFUSER_H
#define SENSORFUSER_H

#include <QtCore>
#include "pose.h"

class SensorFuser : public QObject
{
    Q_OBJECT

public:
    SensorFuser(const quint8& stridePoint = 1, const quint8& strideScan = 1);
    ~SensorFuser();

    void setMaximumFusableRayLength(const float& rayLength) {mMaximumFusableRayLength = rayLength;}

    void setLaserScannerRelativePose(const Pose& pose);

    quint8 getStridePoint(void) const {return mStridePoint;}
    void setStridePoint(quint8 stridePoint) {mStridePoint = stridePoint;}

    quint8 getStrideScan(void) const {return mStrideScan;}
    void setStrideScan(quint8 strideScan) {mStrideScan = strideScan;}



private:
    quint8 mStridePoint, mStrideScan;
    quint32 mPointCloudSize;

    Pose mLaserScannerRelativePose;

    quint16 mStatsFusedScans;
    quint16 mStatsDiscardedScans;

    // How much time difference from a scan to a pose (or vice versa) in past or future for the data to be usable for interpolation?
    quint8 mMaximumTimeBetweenFusedPoseAndScanMsec;
    qint32 mMaximumTimeBetweenMatchingScans;

    float mMaximumFusableRayLength; // rays longer than this will be ignored (because they introduce too much error due to orientation errors)

    qint32 mNewestDataTime; // used to clear data older than X ms
    qint32 mLastScanMiddleTow; // the last time the GNSS told us about the TOW of a scanSweepMiddle.

    Pose mLastInterpolatedPose; // Interpolated poses can be used for many consecutive rays sharing the same millisecond...
    qint32 mLastRayTime; // .. and this var is used to store that msec of the last computed ray.

    QList<Pose> mPoses;

    QVector<QVector3D> mRegisteredPoints;

    /*
      These two containers store a timestamp of a scan and a pointer to its scandata.
      For each scan, we get two timestamps:
        a) one directly from the laserscanner, which is a few msecs off
        b) one from the gps board, which is precise relative to gps-generated poses
      Unfortunately, the scandata (distance values) themselves come with a), so we
      need to match the values of a) from mSavedScansTimestampScanner to the values
      of b) in mSavedScansTimestampGps. This is what matchTimestamps() does.

      Instead of quint32, we use qint32 because we want to substract them from each
      other to see who came first.
      */
    QMap<qint32, std::vector<long>* > mScansTimestampScanner;
    QMap<qint32, std::vector<long>* > mScansTimestampGps;

    // This method matches gps timestamps to laserscanner timestamps
    qint8 matchTimestamps();

    // This method uses mSavedScans and mSavedPoses to create and emit world-cooridnate scanpoints.
    void transformScanDataCubic();
    void transformScanDataNearestNeighbor();

    // Cleans old data (mSaved, scanGps and mSavedPoses
    void cleanUnusableData();

    QVector3D getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16& scannerIndex, const float& distance);

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
    void slotNewVehiclePose(const Pose& pose);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened.
    void slotScanFinished(const quint32& timestamp);

    // Used to feed data from the laserscanner
    void slotNewScanData(const qint32& timestampScanner, std::vector<long> * const distances);

signals:
    void newScannedPoints(const QVector<QVector3D>*, const QVector3D& scanPosition);
};

#endif // SENSORFUSER_H
