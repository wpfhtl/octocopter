#ifndef SENSORFUSER_H
#define SENSORFUSER_H

#include <QtCore>
#include "pose.h"
#include "laserscanner.h"

class SensorFuser : public QObject
{
    Q_OBJECT
private:
    quint32 mPointCloudSize;

    QFile* mLogFileGlobalPoints;  // ply file format
    QFile* mLogFileRawData;  // poses, scans and timestamps

    LaserScanner * const mLaserScanner; // used to retrieve relative pose and use LaserScanner::getWorldPositionOfScannedPoint(...)

    // How much time difference from a scan to a pose (or vice versa) in past or future for the data to be usable for interpolation?
    quint8 mMaximumTimeBetweenFusedPoseAndScanMsec;

//    QVector<quint32> mScanTimestampsFromGps;
    QList<Pose> mPoses;

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
    void transformScanData();

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

public:
    SensorFuser(LaserScanner* const laserScanner);
    ~SensorFuser();

signals:
    void newScannedPoints(const QVector3D& scanPosition, const QVector<QVector3D>&);

private slots:
    void slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points);

public slots:
    // The pose also contains a timestamp (receiver-time) of when that pose was recorded.
    void slotNewVehiclePose(const Pose& pose);

    // When a laserscan is finished, the lidar changes the electrical level on the
    // event-pin of the gps-receiver-board, which then notifies the PC together with
    // the receiver-time (TOW) when this event happened.
    void slotScanFinished(const quint32& timestamp);

    // Used to feed data from the laserscanner
    void slotNewScanData(const quint32& timestampScanner, std::vector<long> * const distances);
};

#endif // SENSORFUSER_H
