#include "sensorfuser.h"

SensorFuser::SensorFuser(LaserScanner* const laserScanner) : QObject(), mLaserScanner(laserScanner)
{
    mPointCloudSize = 0;
    mMaximumTimeBetweenFusedPoseAndScanMsec = 110;

    mLogFileRawData = new QFile(QString("scannerdata-raw-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileRawData->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("SensorFuser::SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileRawData->fileName()));

    mLogFileGlobalPoints = new QFile(QString("scannerdata-fused-%1-%2.ply").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileGlobalPoints->open(QIODevice::ReadWrite | QIODevice::Text))
        qFatal("SensorFuser::SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileGlobalPoints->fileName()));
}

SensorFuser::~SensorFuser()
{
    // We won't gather any new data, close raw logfile
    mLogFileRawData->close();
    mLogFileRawData->deleteLater();

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
        qDebug() << "SensorFuser::~SensorFuser(): writing pointcloud file of about" << ((mPointCloudSize*24.5f)/1000000.0f) << "mb, this might take some time...";
        mLogFileGlobalPoints->reset();
        while (!mLogFileGlobalPoints->atEnd())
        {
            const QByteArray line = mLogFileGlobalPoints->readLine();
            if(!line.contains("extevent") && !line.contains("scannerdata") && !line.contains("pointcloud"))
                logFileDataPly.write(line);
        }

        logFileDataPly.close();
        qDebug() << "SensorFuser::~SensorFuser(): done writing ply file.";
    }

    // Close global-point logfile
    mLogFileGlobalPoints->close();
    mLogFileGlobalPoints->deleteLater();
}

void SensorFuser::cleanUnusableData()
{
    qDebug() << "SensorFuser::cleanUnusableData(): scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
    qDebug() << "SensorFuser::cleanUnusableData(): gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");
    qDebug() << "SensorFuser::cleanUnusableData(): pose stamps:" << getTimeStamps(mPoses).join(",");

    // Delete poses that are far older than the first/oldest scan. These cannot be used anymore because we can only get newer scans from now on.
    while(mScansTimestampGps.size() && mPoses.at(0).timestamp + mMaximumTimeBetweenFusedPoseAndScanMsec < mScansTimestampGps.begin().key())
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first pose from" << mPoses.at(0).timestamp << "because the first of" << mScansTimestampGps.size() << "scans was much later at" << mScansTimestampGps.begin().key();
        mPoses.removeFirst();
    }

    // Delete gps-scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mScansTimestampGps.size() && mPoses.at(0).timestamp - mMaximumTimeBetweenFusedPoseAndScanMsec > mScansTimestampGps.begin().key())
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first scanGps from" << mScansTimestampGps.begin().key() << "because the first of" << mPoses.size() << "poses was much later at" << mPoses.at(0).timestamp;
        mScansTimestampGps.erase(mScansTimestampGps.begin());
    }

    // Delete scanner-scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mScansTimestampScanner.size() && mPoses.at(0).timestamp - mMaximumTimeBetweenFusedPoseAndScanMsec > mScansTimestampScanner.begin().key())
    {
        qDebug() << "SensorFuser::cleanUnusableData(): removing first scanScanner from" << mScansTimestampScanner.begin().key() << "because the first of" << mPoses.size() << "poses was much later at" << mPoses.at(0).timestamp;
        mScansTimestampScanner.erase(mScansTimestampScanner.begin());
    }
}

void SensorFuser::transformScanData()
{
    // We have scan data from previous scans in mSavedScansTimestampGps and poses in mSavedPoses, lets work out the world coordinates.
    qDebug() << "SensorFuser::transformScanData(): gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");
    qDebug() << "SensorFuser::transformScanData(): pose stamps:" << getTimeStamps(mPoses).join(",");

    // Now lets look at every scan...
    QMap<qint32, std::vector<long>* >::iterator iteratorSavedScans = mScansTimestampGps.begin();
    while (iteratorSavedScans != mScansTimestampGps.end())
    {
        // mSavedScansTimestampGps can still contain scans with empty values (pointer is 0) because they
        // weren't populated with matched scans-with-laserscanner-timestamps. Don't try to process those.
        if(iteratorSavedScans.value() == 0)
        {
            ++iteratorSavedScans;
            continue;
        }

        qDebug() << "SensorFuser::transformScanData(): trying to fuse scan from" << iteratorSavedScans.key();
        qint32 timestampMiddleOfScan = iteratorSavedScans.key() - 13; // timestamp is from end of scan, which lasts 25ms

        // find poses in mSavedPoses with timestampScan-maximumMillisecondsBetweenPoseAndScan <= timestampScan <= timestampScan+maximumMillisecondsBetweenPoseAndScan
        QList<Pose*> posesForThisScan;
        for(int j = 0; j < mPoses.size() && posesForThisScan.size() < 4; ++j)
        {
            if(abs(mPoses.at(j).timestamp - timestampMiddleOfScan) < mMaximumTimeBetweenFusedPoseAndScanMsec)
                posesForThisScan.append(&mPoses[j]);
        }


        // For debugging, show which poses could be found:
        QStringList poseTimes;
        for(int i=0;i<posesForThisScan.size();i++) poseTimes << QString::number((uint)posesForThisScan.at(i)->timestamp);
        qDebug() << "SensorFuser::transformScanData(): for scan from" << timestampMiddleOfScan + 13 << "" << posesForThisScan.size() << "poses from" << poseTimes.join(",") << "exist.";

        if(posesForThisScan.size() > 3)
        {
            QVector<QVector3D> scannedPoints;/*(mScanDistancesCurrent->size());*/ // Do not reserve full length, will be less poins due to reflections on the vehicle being filtered

            std::vector<long>* scanDistances = iteratorSavedScans.value();

            for(int index=0; index < scanDistances->size(); index++)
            {
                // Convert millimeters to meters.
                const float distance = (*scanDistances)[index] / 1000.0f;

                // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                if(distance < 0.5f || distance > 10.0f) continue;

                // Interpolate not using a parameter mu with 0.0<=mu<=1.0, but rather by passing a time argument,
                // which is probably the better idea, as it also works when scans and poses don't interleave so well.
                const float scanTime = (float)25.0f;//mScanner.scanMsec();
                const quint32 timeOfThisRay = timestampMiddleOfScan - (25.0f / 2.0f) // when this scan started 180deg in the rear
                                              + scanTime / 8.0f // after running 45degree, (1/8th of angular view), it records first ray
                                              + (scanTime * 0.75f * ((float)index) / ((float)scanDistances->size()));

                // For debugging, show which poses could be found:
                QStringList poseTimes;
                for(int i=0;i<posesForThisScan.size();i++) poseTimes << QString::number((uint)posesForThisScan.at(i)->timestamp);
                qDebug() << "SensorFuser::slotScanFinished(): scanmiddle at" << timestampMiddleOfScan << "ray-index is" << index << "raytime is" << timeOfThisRay << "posetimes:" << poseTimes.join(",");

                const Pose interpolatedPose = Pose::interpolateCubic(
                            posesForThisScan[0],
                            posesForThisScan[1],
                            posesForThisScan[2],
                            posesForThisScan[3],
                            timeOfThisRay
                            );

                scannedPoints.append(mLaserScanner->getWorldPositionOfScannedPoint(interpolatedPose, index, distance));
                mPointCloudSize++;
            }

            slotLogScannedPoints(posesForThisScan[1]->position, scannedPoints);

            emit newScannedPoints(posesForThisScan[1]->position, scannedPoints);

            // This scan has been processed. Delete it.
            delete iteratorSavedScans.value();
            mScansTimestampGps.erase(iteratorSavedScans);
        }
        else
        {
            // We could NOT find enough poses for this scan. This may only happen if this scan is so new that the next poses required for interpolation haven't yet arrived.
            // Make sure that the last pose is not much later than this scan. If it is, we must have missed a pose for this scan. Scan will be deleted later.
            if(mPoses.last().timestamp - timestampMiddleOfScan > mMaximumTimeBetweenFusedPoseAndScanMsec)
            {
                qDebug() << "SensorFuser::transformScanData(): deleting scan data with gps timestamp" << timestampMiddleOfScan + 13 << "because the latest pose is MUCH later, so no new poses helping this scan.";

                delete iteratorSavedScans.value();
                mScansTimestampGps.erase(iteratorSavedScans);
            }
            else
            {
                qDebug() << "SensorFuser::transformScanData(): not enough poses, will try later because latest pose is not THAT old, there might be newer poses helping us.";
            }
        }

        // Whatever happened to this scan, try the next scan!
        ++iteratorSavedScans;
    }

    qDebug() << "SensorFuser::transformScanData(): after processing all data, there's" << mScansTimestampGps.size() << "scans and" << mPoses.size() << "poses left.";
}

qint8 SensorFuser::matchTimestamps()
{
    /* FIXME: it might be better to remove the first gps timestamp:
"18:39:22:022" SensorFuser::slotNewVehiclePose(): received a gps pose pose t499162000 (-46.81/65.73/131.95) YPR (302.76/-1.66/-3.71)
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161507
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161521 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161521 to populate gps timestamp 499161507 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161532
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161546 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161546 to populate gps timestamp 499161532 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161557
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161571 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161571 to populate gps timestamp 499161557 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161582
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161596 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161596 to populate gps timestamp 499161582 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161607
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161621 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161621 to populate gps timestamp 499161607 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161632
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161646 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161646 to populate gps timestamp 499161632 time difference 14
SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp 499161657
SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to 499161671 msecs
SensorFuser::matchTimestamps(): using data from scanner timestamp 499161671 to populate gps timestamp 499161657 time difference 14
*/
    qint16 scansMatched = 0;
    qint16 scansUnmatched = 0;

    qDebug() << "SensorFuser::matchTimestamps(): scnr stamps:" << getTimeStamps(mScansTimestampScanner).join(",");
    qDebug() << "SensorFuser::matchTimestamps(): gnss stamps:" << getTimeStamps(mScansTimestampGps).join(",");

    // clean

    // Iterate through the list of gps-timestamps and try to find fitting laserscanner-timestamps
    QMap<qint32, std::vector<long>*>::const_iterator iteratorScansTimestampGps = mScansTimestampGps.begin();
    while(iteratorScansTimestampGps != mScansTimestampGps.end())
    {
        const qint32 timestampGps = iteratorScansTimestampGps.key();

        // Only treat unmatched entries
        if(iteratorScansTimestampGps.value() == 0)
        {
            // Find the timestamp from laserscanner closest to this one.
            qDebug() << "SensorFuser::matchTimestamps(): looking for a matching scanner timestamp for gps timestamp" << timestampGps;

            qint32 bestFittingScannerTime = 2147483647; // qint32 MAX
            qint32 smallestTimeDifference = 2147483647; // qint32 MAX, will always be an absolute value

            // Iterate through scanner list and find temporally closest entry
            QMap<qint32, std::vector<long>*>::const_iterator iteratorScansTimestampScanner = mScansTimestampScanner.begin();
            while(iteratorScansTimestampScanner != mScansTimestampScanner.end())
            {
                const qint32 timestampScanner = iteratorScansTimestampScanner.key();
                if(abs(timestampScanner - timestampGps) < smallestTimeDifference)
                {
                    qDebug() << "SensorFuser::matchTimestamps(): improved match between gps and laser timestamps to" << timestampScanner << "msecs";
                    smallestTimeDifference = abs(timestampScanner - timestampGps);
                    bestFittingScannerTime = timestampScanner;
                }

                ++iteratorScansTimestampScanner;
            }

            // If we found a close-enough match, use it.
            if(smallestTimeDifference < 16)
            {
                qDebug() << "SensorFuser::matchTimestamps(): using data from scanner timestamp" << bestFittingScannerTime << "to populate gps timestamp" << timestampGps << "time difference" << bestFittingScannerTime - timestampGps;

                // fill gps scanlist
                mScansTimestampGps[timestampGps] = mScansTimestampScanner.value(bestFittingScannerTime);

                // remove used entry form scanner scanlist, but don't delete the scandata, it lives on in mSavedScansTimestampGps and is delete()d in transformScanData().
                mScansTimestampScanner.remove(bestFittingScannerTime);

                scansMatched++;
            }
            else
            {
                qDebug() << "SensorFuser::matchTimestamps(): couldn't match scan from" << timestampGps << "with any gps timestamp, smallestTimeDifference was" << smallestTimeDifference;
                scansUnmatched++;
            }
        }
        ++iteratorScansTimestampGps;
    }

    // TODO: iterate mSavedScansTimestampScanner and find really old data to delete. This data shouldn't exist,
    // because every scan must have caused a SBF-packet creating an entry in the GPS list
    Q_ASSERT(mScansTimestampScanner.size() < 200);

    qDebug() << "SensorFuser::matchTimestamps(): gps scans: matched" << scansMatched << "unmatched" << scansUnmatched << "total" << mScansTimestampGps.size();
    return scansMatched;
}

void SensorFuser::slotNewVehiclePose(const Pose& pose)
{
    if(!mLaserScanner->isScanning())
    {
        qDebug() << t() << "SensorFuser::slotNewVehiclePose(): ignoring a received gps pose, laserscanner isn't scanning at" << pose.timestamp;
        return;
    }

    qDebug() << t() << "SensorFuser::slotNewVehiclePose(): received a gps" << pose;

    // Write log data: pose[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
//    QTextStream out(mLogFileDataRaw);
//    out << "pose " << pose.timestamp << " x" << pose.position.x() << " y" << pose.position.y() << " z" << pose.position.z() << " p" << pose.getPitchDegrees() << " r" << pose.getRollDegrees() << " y" << pose.getYawDegrees() << "\n";

    // Append pose to our list
    mPoses.append(Pose(pose + mLaserScanner->getRelativePose()));

    Q_ASSERT(mPoses.last().timestamp == pose.timestamp && "mangled pose timestamp is off.");

    // Make sure the timestamp from the incoming pose has survived the mangling.
    /*if(mSavedPoses.last().timestamp != pose.timestamp)
        qDebug() << "SensorFuser::slotNewVehiclePose(): setting SensorFuser pose, incoming t" << pose.timestamp
                 << "mRelativePose t" << mRelativeScannerPose.timestamp
                 << "resulting t" << mSavedPoses.last().timestamp;*/

    cleanUnusableData();

    // Fuse pose and scans if it was possible to correct at least one lasertimestamp with a gps timestamp
    if(matchTimestamps())
        transformScanData();
}

void SensorFuser::slotScanFinished(const quint32 &timestampScanGps)
{
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanGps - 1000))
    {
//        qDebug() << t() << "SensorFuser::slotScanFinished(): no/old poses, ignoring scanfinished gps signal at time" << timestampScanGps;
        return;
    }

    qDebug() << t() << "SensorFuser::slotScanFinished(): gps says scanner finished a scan at time" << timestampScanGps;

    // Our gps board tells us that a scan is finished. The scan data itself might already be saved in mSavedScans - or it might not.
    mScansTimestampGps.insert(timestampScanGps, 0);
    //mSavedScansTimestampGps.insert(timestampScanGps - (mLaserScanner->getScanDuration() / 2), 0);

    QTextStream out(mLogFileRawData);
    out << "extevent: scan finished at time " << timestampScanGps << " in gnss receiver timeframe\n";
}

void SensorFuser::slotNewScanData(const quint32& timestampScanScanner, std::vector<long> * const distances)
{
    if(!mPoses.size() || mPoses.last().timestamp < (timestampScanScanner - 1000))
    {
        qDebug() << t() << "SensorFuser::slotNewScanData(): no/old poses, ignoring scandata at time" << timestampScanScanner;
        // We cannot ignore the scandata, we must at least delete() it!
        delete distances;
        return;
    }

    qDebug() << t() << "SensorFuser::slotNewScanData(): received" << distances->size() << "distance values at scannertime" << timestampScanScanner;

    mScansTimestampScanner.insert(timestampScanScanner, distances);

    // Write log data: scan[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
    QTextStream out(mLogFileRawData);
    out << "scannerdata: " << timestampScanScanner;
    std::vector<long>::iterator itr;
    for(itr=distances->begin();itr != distances->end(); ++itr) out << " " << *itr;
    out << "\n";
}

void SensorFuser::slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points)
{
    qDebug() << "SensorFuser::logScannedPoints(): logging" << points.size() << "points.";
    QTextStream out(mLogFileGlobalPoints);
    out << "pointcloud: " << points.size() << " points scanned from world pos: " << vehiclePosition.x() << " " << vehiclePosition.y() << " " << vehiclePosition.z() << "\n";

    for (int i = 0; i < points.size(); ++i)
        out << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << "\n";
}
