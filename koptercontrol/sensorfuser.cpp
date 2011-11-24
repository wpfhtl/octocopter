#include "sensorfuser.h"

SensorFuser::SensorFuser(const Pose& relativeScannerPose) : QObject(), mRelativeScannerPose(relativeScannerPose)
{
    mLogFileScanData = new QFile(QString("scannerdata-fused-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileScanData->open(QIODevice::ReadWrite | QIODevice::Text))
        qFatal("SensorFuser::SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileScanData->fileName()));
}

SensorFuser::~SensorFuser()
{
    // If we have global data, write a new file in PLY format
    if(mLogFileScanData->size())
    {
        QFile logFileDataPly(QString("pointcloud-%1-%2.ply").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
        if(!logFileDataPly.open(QIODevice::WriteOnly | QIODevice::Text))
            qFatal("SensorFuser::~SensorFuser(): Couldn't open logfile %s for writing, exiting.", qPrintable(logFileDataPly.fileName()));

        // Create ply header, "element vertex" requires count argument
        logFileDataPly.write(QString("ply\nformat ascii 1.0\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n").arg(mNumberOfScannedPoints).toAscii());

        // Read the processed scanned points, then write ply file
        // Seek to the file's beginning before reading it
        qDebug() << "SensorFuser::~SensorFuser(): writing pointcloud file of about" << ((mNumberOfScannedPoints*24.5f)/1000000.0f) << "mb, this might take some time...";
        mLogFileScanData->reset();
        while (!mLogFileScanData->atEnd())
        {
            const QByteArray line = mLogFileScanData->readLine();
            if(!line.contains("comment")) logFileDataPly.write(line);
        }

        logFileDataPly.close();
        qDebug() << "SensorFuser::~SensorFuser(): done writing ply file.";
    }

    // Close other logfiles
    mLogFileScanData->close();
}

void SensorFuser::slotNewVehiclePose(const Pose& pose)
{
//    qDebug() << "SensorFuser::slotNewVehiclePose(): received a gps pose" << pose;

    // Write log data: pose[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
//    QTextStream out(mLogFileDataRaw);
//    out << "pose " << pose.timestamp << " x" << pose.position.x() << " y" << pose.position.y() << " z" << pose.position.z() << " p" << pose.getPitchDegrees() << " r" << pose.getRollDegrees() << " y" << pose.getYawDegrees() << "\n";

    // Append pose to our list
    mSavedPoses.append(Pose(pose + mRelativeScannerPose));

    // Make sure the timestamp from the incoming pose has survived the mangling.
    if(mSavedPoses.last().timestamp != pose.timestamp)
        qDebug() << "SensorFuser::slotNewVehiclePose(): setting SensorFuser pose, incoming t" << pose.timestamp
                 << "mRelativePose t" << mRelativeScannerPose.timestamp
                 << "resulting t" << mSavedPoses.last().timestamp;
}

void SensorFuser::transformScanData()
{
    // We have scan data from previous scans in mLastScans and poses in mLastPoses, lets work out the world coordinates.

    // How much time difference from a scan to a pose (in past of future) for the pose to be usable for interpolation?
    const quint8 maximumMillisecondsBetweenPoseAndScan = 60;

    // Delete poses that are far older than the first/oldest scan. These cannot be used anymore because we can only get newer scans from now on.
    while(mSavedScans.size() && mSavedPoses.at(0).timestamp + maximumMillisecondsBetweenPoseAndScan < mSavedScans.begin().key())
    {
        qDebug() << "SensorFuser::transformScan(): removing first pose from" << mSavedPoses.at(0).timestamp << "because the first of" << mSavedScans.size() << "scans was much later at" << mSavedScans.begin().key();
        mSavedPoses.removeFirst();
    }

    // Delete scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mSavedScans.size() && mSavedPoses.at(0).timestamp - maximumMillisecondsBetweenPoseAndScan > mSavedScans.begin().key())
    {
        qDebug() << "SensorFuser::transformScan(): removing first scan from" << mSavedScans.begin().key() << "because the first of" << mSavedPoses.size() << "poses was much later at" << mSavedPoses.at(0).timestamp;
        mSavedScans.erase(mSavedScans.begin());
    }

    // Now lets look at every scan...
    QMap<quint32, std::vector<long> >::iterator i = mSavedScans.begin();
    while (i != mSavedScans.end())
    {
        quint32 timestampScanMiddle = i.key();

        // find poses in mSavedPoses with timestampScan-maximumMillisecondsBetweenPoseAndScan <= timestampScan <= timestampScan+maximumMillisecondsBetweenPoseAndScan
        QList<Pose*> posesForThisScan;
        for(int j = 0; j < mSavedPoses.size() && posesForThisScan.size() < 4; ++j)
        {
            if(abs(mSavedPoses.at(j).timestamp - timestampScanMiddle) < maximumMillisecondsBetweenPoseAndScan)
                posesForThisScan.append(&mSavedPoses[j]);
        }

        if(posesForThisScan.size() == 4)
        {
            QVector<QVector3D> scannedPoints;/*(mScanDistancesCurrent->size());*/ // Do not reserve full length, will be less poins due to reflections on the vehicle being filtered

            std::vector<long>* scanDistances = &(i.value());

            for(int index=0; index < scanDistances->size(); index++)
            {
                // Convert millimeters to meters.
                const float distance = (*scanDistances)[index] / 1000.0f;

                // Skip reflections on vehicle (=closer than 50cm) and long ones (bad platform orientation accuracy)
                if(distance < 0.5f || distance > 10.0f) continue;

                // Interpolate using the last 4 poses. Do NOT interpolate between 0.0 and 1.0, as
                // the scan actually only takes place between 0.125 and 0.875 (the scanner only
                // scans the central 270 degrees of the 360 degree-circle).
        /*                const Pose interpolatedPose = Pose::interpolateCubic(
                            mScannerPoseFirst,
                            mScannerPoseBefore,
                            mScannerPoseAfter,
                            mScannerPoseLast,
                            (float)(0.125f + (0.75f * index / mScanDistancesCurrent->size()))
                            );*/

                // Interpolate not using a parameter mu with 0.0<=mu<=1.0, but rather by passing a time argument,
                // which is probably the better idea, as it also works when scans and poses don't interleave so well.
                const float scanTime = (float)25.0f;//mScanner.scanMsec();
                const quint32 timeOfThisRay = timestampScanMiddle - (25.0f / 2.0f) // when this scan started 180deg in the rear
                                              + scanTime / 8.0f // after running 45degree, (1/8th of angular view), it records first ray
                                              + (scanTime * 0.75f * ((float)index) / ((float)scanDistances->size()));

                qDebug() << "SensorFuser::slotScanFinished(): scanmiddle at" << timestampScanMiddle << "ray-index is" << index << "raytime is" << timeOfThisRay << "before" << posesForThisScan[1]->timestamp << "after" << posesForThisScan[2]->timestamp;

                const Pose interpolatedPose = Pose::interpolateCubic(
                            posesForThisScan[0],
                            posesForThisScan[1],
                            posesForThisScan[2],
                            posesForThisScan[3],
                            timeOfThisRay
                            );

                scannedPoints.append(getWorldPositionOfScannedPoint(interpolatedPose, index, distance));
                mNumberOfScannedPoints++;
            }

            slotLogScannedPoints(posesForThisScan[1]->position, scannedPoints);

            emit newScannedPoints(posesForThisScan[1]->position, scannedPoints);

            // This can has been processed. Delete it.
            i = mSavedScans.erase(i);
        }
        else
        {
            // We could NOT find enough poses for this scan. This may only happen if this scan is so new that the next poses required for interpolation haven't yet arrived.
            // Make sure that the last pose is not much later than this scan. If it is, we must have missed a pose for this scan. Scan will be deleted later.
            if(mSavedPoses.last().timestamp - timestampScanMiddle > maximumMillisecondsBetweenPoseAndScan)
            {
                qDebug() << "SensorFuser::transformScan(): couldn't find 4 poses for scan from" << timestampScanMiddle << ", latest pose is from" << mSavedPoses.last().timestamp << ", deleting scan";
                i = mSavedScans.erase(i);
            }
        }
    }

    qDebug() << "SensorFuser::transformScan(): after processing all data, there's" << mSavedScans.size() << "scans and" << mSavedPoses.size() << "poses left.";
}


void SensorFuser::slotScanFinished(const quint32 &timestamp)
{
    qDebug() << t() << "SensorFuser::slotScanFinished(): gps says scanner finished a scan at time" << timestamp;

    // Our gps board tells us when a scan is finished. The scan might already be saved in mSavedScans or it might not.
    mScanTimestampsFromGps.append(timestamp - (mScanner.scanMsec() / 2));
}


void SensorFuser::slotNewScanData(const quint32& timestampScanner, const std::vector<long>& points)
{
    mSavedScans.insert(timestampScanner, points);
}

void SensorFuser::slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points)
{
    qDebug() << "SensorFuser::logScannedPoints(): logging" << points.size() << "points.";
    QTextStream out(mLogFileDataGlobal);
    out << "comment: " << points.size() << " points scanned from world pos: " << vehiclePosition.x() << " " << vehiclePosition.y() << " " << vehiclePosition.z() << "\n";

    for (int i = 0; i < points.size(); ++i)
        out << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << "\n";
}
