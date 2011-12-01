#include "sensorfuser.h"

SensorFuser::SensorFuser(LaserScanner* const laserScanner, const bool& writeLogs) : QObject(), mLaserScanner(laserScanner)
{
    mPointCloudSize = 0;
    mNewestDataTime = 0;
    mStatsFusedScans = 0;
    mStatsDiscardedScans = 0;
    mLastRayTime = -1000; // make sure first comparision fails
    mWriteLogs = writeLogs;
    mMaximumTimeBetweenFusedPoseAndScanMsec = 101; // 2*poseInterval+1
    mMaximumTimeBetweenMatchingScans = 12; // msecs maximum clock offset between scanner and gps device. Smaller means less data, moremeans worse data. Yes, we're screwed.

    if(mWriteLogs)
    {
        mLogFileRawData = new QFile(QString("scannerdata-raw-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
        if(!mLogFileRawData->open(QIODevice::WriteOnly | QIODevice::Text))
            qFatal("SensorFuser::SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileRawData->fileName()));

        mLogFileGlobalPoints = new QFile(QString("scannerdata-fused-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
        if(!mLogFileGlobalPoints->open(QIODevice::ReadWrite | QIODevice::Text))
            qFatal("SensorFuser::SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileGlobalPoints->fileName()));
    }
}

SensorFuser::~SensorFuser()
{
    if(mWriteLogs)
    {
        // We won't gather any new data, close raw logfile
        mLogFileRawData->close();

        // deleteLater() doesn't help, because the event-loop won't run again.
        delete mLogFileRawData;

        // Seems file is zero-sized and thus not processed?!
        mLogFileGlobalPoints->flush();

        // If we have global data, write a new file in PLY format
        if(mLogFileGlobalPoints->size())
        {
            QFile logFileDataPly(QString("pointcloud-%1-%2.ply").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
            if(!logFileDataPly.open(QIODevice::WriteOnly | QIODevice::Text))
                qFatal("SensorFuser::~SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(logFileDataPly.fileName()));

            // Create ply header, "element vertex" requires count argument
            logFileDataPly.write(QString("ply\nformat ascii 1.0\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n").arg(mPointCloudSize).toAscii());

            // Read the processed scanned points, then write ply file
            // Seek to the file's beginning before reading it
            //qDebug() << "SensorFuser::~SensorFuser(): writing pointcloud file of about" << ((mPointCloudSize*24.5f)/1000000.0f) << "mb, this might take some time...";
            mLogFileGlobalPoints->reset();
            while (!mLogFileGlobalPoints->atEnd())
            {
                const QByteArray line = mLogFileGlobalPoints->readLine();
                if(!line.contains("extevent") && !line.contains("scannerdata") && !line.contains("pointcloud"))
                    logFileDataPly.write(line);
            }

            logFileDataPly.close();
            //qDebug() << "SensorFuser::~SensorFuser(): done writing ply file.";
        }

        // Close global-point logfile
        mLogFileGlobalPoints->close();
        delete mLogFileGlobalPoints;
    }

    qDebug() << "SensorFuser::~SensorFuser(): total scans:"  << mStatsFusedScans + mStatsDiscardedScans << "fused:" << mStatsFusedScans << "discarded:"<< mStatsDiscardedScans;
}

void SensorFuser::cleanUnusableData()
{
//    //qDebug() << "SensorFuser::cleanUnusableData(): before scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
//    //qDebug() << "SensorFuser::cleanUnusableData(): before gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");
//    //qDebug() << "SensorFuser::cleanUnusableData(): before pose stamps:" << getTimeStamps(mPoses).join(",");

    // We can delete all scandata if we have no poses. We always need a pose wih a timestamp at least 50+13msec
    // smaller than scandata for interpolation. So if we have scandata and NO poses, all that data is useless -
    // except if poses arrived more than 63msec after they were measured, then latency would invalidate this thought
    if(mPoses.size() == 0 && (mScansTimestampScanner.size() || mScansTimestampGps.size()))
    {
        qDebug() << "SensorFuser::cleanUnusableData(): no poses present, discarding" << mScansTimestampScanner.size() << "scanner scans and" << mScansTimestampGps.size() << "gps scans";

        QMap<qint32, std::vector<long>*>::const_iterator itScanScanner = mScansTimestampScanner.constBegin();
        while(itScanScanner != mScansTimestampScanner.constEnd())
        {
            if(itScanScanner.value())
            {
                delete itScanScanner.value();
                mStatsDiscardedScans++;
            }
            ++itScanScanner;
        }
        mScansTimestampScanner.clear();

        QMap<qint32, std::vector<long>*>::const_iterator itScanGps = mScansTimestampGps.constBegin();
        while(itScanGps != mScansTimestampGps.constEnd())
        {
            if(itScanGps.value())
            {
                delete itScanGps.value();
                mStatsDiscardedScans++;
            }
            ++itScanGps;
        }
        mScansTimestampGps.clear();
    }

    // Delete gps-scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mPoses.size() && mScansTimestampGps.size() && mPoses.at(0).timestamp - mMaximumTimeBetweenFusedPoseAndScanMsec - 13 > mScansTimestampGps.begin().key())
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first scanGps from" << mScansTimestampGps.constBegin().key() << "because the first of" << mPoses.size() << "poses was much later at" << mPoses.at(0).timestamp;
        if(mScansTimestampGps.begin().value())
        {
            delete(mScansTimestampGps.begin().value()); // delete the data it is pointing to. Might be 0, but then delete() shouldn't harm.
            mStatsDiscardedScans++;
        }
        mScansTimestampGps.erase(mScansTimestampGps.begin());
    }
    /*
    // Delete poses that are far older than the first/oldest scan. These cannot be used anymore because we can only get newer scans from now on.
    We need to compare to BOTH gps and scanner stamps!
    while(mScansTimestampGps.size() && mPoses.at(0).timestamp + mMaximumTimeBetweenFusedPoseAndScanMsec < mScansTimestampGps.begin().key())
    {
        //qDebug() << "SensorFuser::cleanUnusableData(): removing first pose from" << mPoses.at(0).timestamp << "because the first of" << mScansTimestampGps.size() << "scans was much later at" << mScansTimestampGps.begin().key();
        mPoses.removeFirst();
    }

    // Delete scanner-scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mScansTimestampScanner.size() && mPoses.at(0).timestamp - mMaximumTimeBetweenFusedPoseAndScanMsec > mScansTimestampScanner.begin().key())
    {
        //qDebug() << "SensorFuser::cleanUnusableData(): removing first scanScanner from" << mScansTimestampScanner.begin().key() << "because the first of" << mPoses.size() << "poses was much later at" << mPoses.at(0).timestamp;
        mScansTimestampScanner.erase(mScansTimestampScanner.begin());
    }
    */

    // We can delete all EMPTY=UNMATCHED gnss scans with timestamps more than mMaximumTimeBetweenMatchingScans msecs older than
    // the first scanner-timestamp: as we'll only get newer scanner timestamps, these gps timestamps cannot ever be matched and become useful
    while(
          mScansTimestampScanner.size()
          && mScansTimestampGps.size()
          && mScansTimestampGps.constBegin().key() < (mScansTimestampScanner.constBegin().key() - mMaximumTimeBetweenMatchingScans)
          && mScansTimestampGps.constBegin().value() == 0
          )
    {
        //qDebug() << "SensorFuser::cleanUnusableData(): will never find scandata for empty gps scan" << mScansTimestampGps.constBegin().key() << "because first scanner scan is much later at" << mScansTimestampScanner.constBegin().key();

        const int numRemoved = mScansTimestampGps.remove(mScansTimestampGps.constBegin().key());

        if(numRemoved != 1) qDebug() << "SensorFuser::cleanUnusableData(): couldn't remove gps scan from" << mScansTimestampGps.constBegin().key() << ", removed" << numRemoved;
    }

    // This aproach is more brute-force: delete all data X milliseconds older than the newest data
    const qint32 maxAge = 400;
    const qint32 minimumDataTimeToSurvive = mNewestDataTime - maxAge;

    while(mPoses.size() && mPoses.first().timestamp < minimumDataTimeToSurvive)
    {
        //qDebug() << "SensorFuser::cleanUnusableData(): removing first pose from" << mPoses.first().timestamp << "because it is more than" << maxAge << "msec older than the newest data from" << mNewestDataTime;
        mPoses.removeFirst();
    }

    while(mScansTimestampScanner.size() && mScansTimestampScanner.constBegin().key() < minimumDataTimeToSurvive)
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first scanScanner from" << mScansTimestampScanner.constBegin().key() << "because it is more than" << maxAge << "msec older than the newest data from" << mNewestDataTime;

        if(mScansTimestampScanner.value(mScansTimestampScanner.constBegin().key()))
        {
            delete mScansTimestampScanner.value(mScansTimestampScanner.constBegin().key());
            mStatsDiscardedScans++;
        }

        mScansTimestampScanner.remove(mScansTimestampScanner.constBegin().key());
    }

    while(mScansTimestampGps.size() && mScansTimestampGps.constBegin().key() < minimumDataTimeToSurvive)
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first scanGps from" << mScansTimestampGps.constBegin().key() << "because it is more than" << maxAge << "msec older than the newest data from" << mNewestDataTime;
        if(mScansTimestampGps.value(mScansTimestampGps.constBegin().key()))
        {
            delete mScansTimestampGps.value(mScansTimestampGps.constBegin().key());
            mStatsDiscardedScans++;
        }
        mScansTimestampGps.remove(mScansTimestampGps.constBegin().key());
    }

    //qDebug() << "SensorFuser::cleanUnusableData(): after scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
    //qDebug() << "SensorFuser::cleanUnusableData(): after gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");
    //qDebug() << "SensorFuser::cleanUnusableData(): after pose stamps:" << getTimeStamps(mPoses).join(",");
}

void SensorFuser::transformScanData()
{
    // We have scan data from previous scans in mSavedScansTimestampGps and poses in mSavedPoses, lets work out the world coordinates.
    //qDebug() << "SensorFuser::transformScanData(): gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");
    //qDebug() << "SensorFuser::transformScanData(): pose stamps:" << getTimeStamps(mPoses).join(",");

    // Now lets look at every scan...
    QMutableMapIterator<qint32, std::vector<long>* > iteratorSavedScans(mScansTimestampGps);
//    QMap<qint32, std::vector<long>* >::iterator iteratorSavedScans = mScansTimestampGps.begin();
    while(iteratorSavedScans.hasNext())
    {
        // Advance the iterator to the next item.
        iteratorSavedScans.next();

        // mSavedScansTimestampGps can still contain scans with empty values (pointer is 0) because they
        // weren't populated with matched scans-with-laserscanner-timestamps. Don't try to process those.
        if(iteratorSavedScans.value() == 0)
        {
            continue;
        }

        const qint32 timestampMiddleOfScan = iteratorSavedScans.key();
        //qDebug() << "SensorFuser::transformScanData(): trying to fuse scan from" << timestampMiddleOfScan;

        // Find Poses needed to fuse this scan. Scans are stored with the times their ray was in front, so for cubic
        // interpolation, we need two poses before and two poses after each *ray*. Hence, we need two poses before
        // scanStartRay (t-9.375ms) and two poses after scanEndRay (t+9.375ms). Depending on the temporal shift
        // between scans and poses, we'll need 4 or 5 poses for all of a scan's rays.
        const qint32 rayStart = timestampMiddleOfScan - 10;
        const qint32 rayEnd = timestampMiddleOfScan + 10;

        QList<Pose*> posesForThisScan;
        for(int j = 0; j < mPoses.size(); ++j)
        {
            /* unnecessarily complex:
            // Use poses that are 1) before rayStart and were 2) less than mMaximumTimeBetweenFusedPoseAndScanMsec before rayStart
            if(rayStart - mPoses.at(j).timestamp > 0 && rayStart - mPoses.at(j).timestamp < mMaximumTimeBetweenFusedPoseAndScanMsec)
                posesForThisScan.append(&mPoses[j]);
            // Use poses that are 1) after rayEnd and were 2) less than mMaximumTimeBetweenFusedPoseAndScanMsec after rayEnd
            else if(mPoses.at(j).timestamp - rayEnd > 0 && mPoses.at(j).timestamp - rayEnd < mMaximumTimeBetweenFusedPoseAndScanMsec)
                posesForThisScan.append(&mPoses[j]);
                */

//            //qDebug() << "SensorFuser::transformScanData(): timediff between gpsscan" << timestampMiddleOfScan << "and pose" << j << "is" << abs(timestampMiddleOfScan - mPoses.at(j).timestamp);
            if(abs(timestampMiddleOfScan - mPoses.at(j).timestamp) < mMaximumTimeBetweenFusedPoseAndScanMsec + 12)
            {
//                //qDebug() << "SensorFuser::transformScanData(): using pose from" << mPoses[j].timestamp << "for scan from" << timestampMiddleOfScan;
                posesForThisScan.append(&mPoses[j]);
            }

            // There's no scenario where we could use 6 poses for interpolation, so skip trying to look for more.
            if(posesForThisScan.size() >= 5) break;
        }

        // For debugging, show which poses could be found:
        QStringList poseTimes;
        for(int i=0;i<posesForThisScan.size();i++) poseTimes << QString::number((uint)posesForThisScan.at(i)->timestamp);
        //qDebug() << "SensorFuser::transformScanData(): for scan from" << timestampMiddleOfScan << "we have" << posesForThisScan.size() << "poses from" << poseTimes.join(",");

        // We found enough poses if we found at least 4 poses and:
        //  - The second-to-last pose if after rayEnd
        //  AND
        //  - The second pose is before rayStart
        if(posesForThisScan.size() >= 4 && posesForThisScan.at(1)->timestamp < rayStart && posesForThisScan.at(posesForThisScan.size()-2)->timestamp > rayEnd)
        {
            std::vector<long>* scanDistances = iteratorSavedScans.value();
            //qDebug() << "SensorFuser::transformScanData(): these" << posesForThisScan.size() << "poses are enough, fusing" << scanDistances->size() << "rays";

            mStatsFusedScans++;

            QVector<QVector3D> scannedPoints; // Do not reserve full length, will be less poins due to reflections on the vehicle being filtered
            scannedPoints.reserve(800);

            for(int index=0; index < scanDistances->size(); index++)
            {
                // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                if((*scanDistances)[index] < 500) continue;

                // Convert millimeters to meters.
                const float distance = (*scanDistances)[index] / 1000.0f;

                // Interpolate not using a parameter mu with 0.0<=mu<=1.0, but rather by passing the time of that ray
                // index 0000 is timestampMiddleOfScan - 9.375ms
                // index 0540 is timestampMiddleOfScan
                // index 1080 is timestampMiddleOfScan + 9.375ms
                // Each ray takes 0.01736 milliseconds of time.
                const qint32 timeOfCurrentRay = timestampMiddleOfScan + (qint32)(-9.375f + (((float)index) * 0.01736f));

                // For debugging, show which poses could be found:
//                QStringList poseTimes;
//                for(int i=0;i<posesForThisScan.size();i++) poseTimes << QString::number((uint)posesForThisScan.at(i)->timestamp);
//                //qDebug() << "SensorFuser::transformScanData(): scanmiddle at" << timestampMiddleOfScan << "ray-index is" << index << "raytime is" << timeOfCurrentRay<< "posetimes:" << poseTimes.join(",");

                // Many consecutive rays share the same millisecond, so we only need to interpolate a new pose if that ray's millisecond has changed.
                if(mLastRayTime != timeOfCurrentRay)
                {
                    // Now figure out which poses are needed for this ray
                    if(posesForThisScan.at(2)->timestamp < timeOfCurrentRay)
                    {
                        mLastInterpolatedPose = Pose::interpolateCubic(
                                posesForThisScan[1],
                                posesForThisScan[2],
                                posesForThisScan[3],
                                posesForThisScan[4],
                                timeOfCurrentRay
                                );
                    }
                    else
                    {
                        mLastInterpolatedPose = Pose::interpolateCubic(
                                posesForThisScan[0],
                                posesForThisScan[1],
                                posesForThisScan[2],
                                posesForThisScan[3],
                                timeOfCurrentRay
                                );
                    }
//                    //qDebug() << "SensorFuser::transformScanData(): interpolated pose to be used:" << mLastInterpolatedPose;
                }


                mLastRayTime = timeOfCurrentRay;

                scannedPoints.append(mLaserScanner->getWorldPositionOfScannedPoint(mLastInterpolatedPose, index, distance));

                mPointCloudSize++;
            }

            if(mWriteLogs) slotLogScannedPoints(posesForThisScan[1]->position, scannedPoints);

            emit newScannedPoints(posesForThisScan[1]->position, scannedPoints);

            // This scan has been processed. Delete it.
            delete iteratorSavedScans.value();
            iteratorSavedScans.remove();
        }
        else
        {
            // We could NOT find enough poses for this scan. This may only happen if this scan is so new that the next poses required for interpolation haven't yet arrived.
            // Make sure that the last pose is not much later than this scan. If it is, we must have missed a pose for this scan, meaning we can delete it.
            if(mPoses.last().timestamp - timestampMiddleOfScan > mMaximumTimeBetweenFusedPoseAndScanMsec + 10)
            {
                qDebug() << "SensorFuser::transformScanData(): deleting scan data with gps timestamp" << timestampMiddleOfScan << "because the latest pose is MUCH later at" << mPoses.last().timestamp << "- so no new poses helping this scan.";
                mStatsDiscardedScans++;

                delete iteratorSavedScans.value();
                iteratorSavedScans.remove();
            }
            else if( // scantime is in middle of scan, rays are collected 10ms before and after this moment
                    posesForThisScan.size() >= 2
                    && posesForThisScan.last()->timestamp > timestampMiddleOfScan + 10
                    && posesForThisScan.at(posesForThisScan.size()-2)->timestamp > timestampMiddleOfScan + 10)
            {
                // We can also delete scans if there's at least ONE pose AFTER them but not enough poses BEFORE them, because there will not be older poses coming in.
                qDebug() << "SensorFuser::transformScanData(): deleting scan data with gps timestamp" << timestampMiddleOfScan << " - 2 poses after scan are present, failure must be missing pre-poses. Unfixable.";
                qDebug() << "SensorFuser::transformScanData(): poses:" << getTimeStamps(mPoses);
                mStatsDiscardedScans++;
                delete iteratorSavedScans.value();
                iteratorSavedScans.remove();
            }
            else
            {
                //qDebug() << "SensorFuser::transformScanData(): not enough poses, will try later because latest pose is not THAT old, there might be newer poses helping us.";
            }
        }
    }

    //qDebug() << "SensorFuser::transformScanData(): after processing all data, there's" << mScansTimestampGps.size() << "scans and" << mPoses.size() << "poses left.";
}

qint8 SensorFuser::matchTimestamps()
{
    qint16 scansMatched = 0;
    qint16 scansUnmatched = 0;

    //qDebug() << "SensorFuser::matchTimestamps(): scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
    //qDebug() << "SensorFuser::matchTimestamps(): gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");

    // Iterate through the list of gps-timestamps and try to find fitting laserscanner-timestamps - as long as there are scanner scans
    QMap<qint32, std::vector<long>*>::const_iterator iteratorScansTimestampGps = mScansTimestampGps.begin();
    while(mScansTimestampScanner.size() && iteratorScansTimestampGps != mScansTimestampGps.end())
    {
        const qint32 timestampGps = iteratorScansTimestampGps.key();

        // Only treat unmatched entries
        if(iteratorScansTimestampGps.value() == 0)
        {
            // Find the timestamp from laserscanner closest to this one.
            //qDebug() << "SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp" << timestampGps;

            qint32 bestFittingScannerTime = 2147483647; // qint32 MAX
            qint32 smallestTimeDifference = 2147483647; // qint32 MAX, will always be an absolute value

            // Iterate through scanner list and find temporally closest entry
            QMap<qint32, std::vector<long>*>::const_iterator iteratorScansTimestampScanner = mScansTimestampScanner.begin();
            while(iteratorScansTimestampScanner != mScansTimestampScanner.end())
            {
                const qint32 timestampScanner = iteratorScansTimestampScanner.key();
                if(abs(timestampScanner - timestampGps) < smallestTimeDifference)
                {
                    //qDebug() << "SensorFuser::matchTimestamps(): improved match between gps" <<  timestampGps << "and scanner timestamp" << timestampScanner << "to" << abs(timestampScanner - timestampGps) << "msecs";
                    smallestTimeDifference = abs(timestampScanner - timestampGps);
                    bestFittingScannerTime = timestampScanner;
                }
                else
                {
                    // The time difference between our gps_scan and the scanner_scan has INcreased, so we're on the wrong path. STOP!
//                     //qDebug() << "SensorFuser::matchTimestamps(): comparison to next scanner timestamp shows worse results than last time, error now:" << abs(timestampScanner - timestampGps) << "msecs, breaking.";
                    break;
                }

                ++iteratorScansTimestampScanner;
            }

            // If we found a close-enough match, use it.
            if(smallestTimeDifference < mMaximumTimeBetweenMatchingScans)
            {
                //qDebug() << "SensorFuser::matchTimestamps(): using data from scanner timestamp" << bestFittingScannerTime << "to populate gps timestamp" << timestampGps << "time difference" << smallestTimeDifference;

                // fill gps scanlist
                mScansTimestampGps[timestampGps] = mScansTimestampScanner.value(bestFittingScannerTime);

                // remove used entry form scanner scanlist, but don't delete the scandata, it lives on in mSavedScansTimestampGps and is delete()d in transformScanData().
                mScansTimestampScanner.remove(bestFittingScannerTime);

                scansMatched++;
            }
            else
            {
                //qDebug() << "SensorFuser::matchTimestamps(): couldn't match scan from" << timestampGps << "with any gps timestamp, smallestTimeDifference was" << smallestTimeDifference;
                scansUnmatched++;
            }
        }
        ++iteratorScansTimestampGps;
    }

    // TODO: iterate mSavedScansTimestampScanner and find really old data to delete. This data shouldn't exist,
    // because every scan must have caused a SBF-packet creating an entry in the GPS list
    Q_ASSERT(mScansTimestampScanner.size() < 200);

    //qDebug() << "SensorFuser::matchTimestamps(): gps scans: matched" << scansMatched << "unmatched" << scansUnmatched << "total" << mScansTimestampGps.size();
    return scansMatched;
}

void SensorFuser::slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points)
{
    //qDebug() << "SensorFuser::logScannedPoints(): logging" << points.size() << "points.";
    QTextStream out(mLogFileGlobalPoints);
    out << "pointcloud: " << points.size() << " points scanned from world pos: " << vehiclePosition.x() << " " << vehiclePosition.y() << " " << vehiclePosition.z() << "\n";

    for(int i = 0; i < points.size(); ++i)
        out << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << endl;
}


void SensorFuser::slotNewVehiclePose(const Pose& pose)
{
    //qDebug() << t() << "SensorFuser::slotNewVehiclePose(): received a pose" << pose;

    if(mWriteLogs)
    {
        // Always write pose to logfile
        QTextStream out(mLogFileRawData);
        out << pose.toString() << endl;
    }

    if(!mLaserScanner->isScanning())
    {
        //qDebug() << t() << "SensorFuser::slotNewVehiclePose(): ignoring a received pose, laserscanner isn't scanning at" << pose.timestamp;
        return;
    }

    // Append pose to our list
    mPoses.append(Pose(pose + mLaserScanner->getRelativePose()));

    Q_ASSERT(mPoses.last().timestamp == pose.timestamp && "mangled pose timestamp is off.");

    mNewestDataTime = std::max(mNewestDataTime, pose.timestamp);

    // Make sure the timestamp from the incoming pose has survived the mangling.
    /*if(mSavedPoses.last().timestamp != pose.timestamp)
        //qDebug() << "SensorFuser::slotNewVehiclePose(): setting SensorFuser pose, incoming t" << pose.timestamp
                 << "mRelativePose t" << mRelativeScannerPose.timestamp
                 << "resulting t" << mSavedPoses.last().timestamp;*/

    cleanUnusableData();

    // Fuse pose and scans if it was possible to correct at least one lasertimestamp with a gps timestamp
    if(matchTimestamps())
        transformScanData();
}

void SensorFuser::slotScanFinished(const quint32 &timestampScanGps)
{
    //qDebug() << t() << "SensorFuser::slotScanFinished(): gps says scanner finished a scan at time" << timestampScanGps;

    if(mWriteLogs)
    {
        // Log this event to file
        QTextStream(mLogFileRawData) << "extevent: " << timestampScanGps << endl;
    }

    // Do not store data that we cannot fuse anyway, because the newest pose is very old (no gnss reception)
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanGps - 1000))
    {
//        //qDebug() << t() << "SensorFuser::slotScanFinished(): " << mPoses.size() << "/old poses, ignoring scanfinished gps signal for scantime" << timestampScanGps;
        return;
    }

    // will not work because qmap::end() ppoints to the imaginary item AFTER the last item in the map. key() will be 0.
    // Do not store data that we cannot fuse anyway, because the newest scanner data is very old (no data due to high system load)
//    if(!mScansTimestampScanner.size() || mScansTimestampScanner.end().key() < (timestampScanGps - 1000))
  //  {
    //    //qDebug() << t() << "SensorFuser::slotScanFinished(): " << mScansTimestampScanner.size() << "/old scandata, ignoring scanfinished gps signal for scantime" << timestampScanGps << "because last scna timestamp" << mScansTimestampScanner.end().key() << "is smaller than" << (timestampScanGps - 1000) << "=> older than 1 sec";
//        return;
    //}

    // Our gps board tells us that a scan is finished. The scan data itself might already be saved in mSavedScans - or it might not.
    mScansTimestampGps.insert(timestampScanGps, 0);
    mNewestDataTime = std::max((unsigned int)mNewestDataTime, timestampScanGps);
}

void SensorFuser::slotNewScanData(const quint32& timestampScanScanner, std::vector<long> * const distances)
{
    //qDebug() << t() << "SensorFuser::slotNewScanData(): received" << distances->size() << "distance values from scannertime" << timestampScanScanner;

    if(mWriteLogs)
    {
        // Always write log data for later replay: scannerdata:[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
        QTextStream out(mLogFileRawData);
        out << "scannerdata: " << timestampScanScanner;
        std::vector<long>::iterator itr;
        for(itr=distances->begin();itr != distances->end(); ++itr) out << " " << *itr;
        out << endl;
    }

    // Do not store data that we cannot fuse anyway, because the newest pose is very old (no gnss reception)
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanScanner - mMaximumTimeBetweenFusedPoseAndScanMsec - 13))
    {
        //qDebug() << t() << "SensorFuser::slotNewScanData(): " << mPoses.size() << "/old poses, ignoring scandata at time" << timestampScanScanner;
        // We cannot ignore the scandata, we must at least delete() it, because it was new()ed in LaserScanner and we are now the owner.
        delete distances;
        return;
    }

    mScansTimestampScanner.insert(timestampScanScanner, distances);
    mNewestDataTime = std::max((unsigned int)mNewestDataTime, timestampScanScanner);

    cleanUnusableData();
}

bool SensorFuser::processLogLine(const QString& line)
{
    if(line.contains("scannerdata"))
    {
        const QStringList tokens = line.split(' ');
        bool success = false;
        const qint32 timestamp = tokens.at(1).toInt(&success);
        if(!success) {qDebug() << "SensorFuser::processLogLine(): couldn't parse scannerdata-timestamp."; return false;}

        std::vector<long>* data = new std::vector<long>;
        for(int i=2;i<tokens.size();i++)
        {
            data->push_back(tokens.at(i).toInt(&success));
            if(!success) {qDebug() << "SensorFuser::processLogLine(): couldn't parse scannerdata-distance at index" << i; return false;}
        }

        slotNewScanData(timestamp, data);
    }
    else if(line.contains("extevent"))
    {
        bool success = false;
        slotScanFinished(line.split(' ').at(1).toInt(&success));
        if(!success) {qDebug() << "SensorFuser::processLogLine(): couldn't parse extevent"; return false;}
    }
    else if(line.contains("pose"))
    {
        slotNewVehiclePose(Pose(line));
    }
    else
    {
        qDebug() << "SensorFuser::processLogLine(): couldn't parse line" << line;
        return false;
    }

    return true;
}

bool SensorFuser::processLog(const QString& fileName)
{
    QTime startTime;
    startTime.start();

    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "SensorFuser::processLog(): cannot open file" << fileName << "for reading.";
        return false;
    }

    quint32 lineNumber = 0;
    QTextStream in(&file);
    while (!in.atEnd())
    {
        const QString line = in.readLine();
        if(!processLogLine(line))
        {
            qDebug() << "SensorFuser::processLog(): trouble parsing" << fileName << "line" << lineNumber << ":" << line;
            return false;
        }
        lineNumber++;
    }

    qDebug() << "SensorFuser::processLog(): processing time was" << startTime.elapsed() << "msecs.";
    return true;
}
