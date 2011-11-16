#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &pose)
{
    mRelativePose = pose;
    qDebug() << "LaserScanner::LaserScanner(): initializing the laserscanner with a relative pose of" << mRelativePose;

    mDeviceFileName = deviceFileName;
    mNumberOfScannedPoints = 0;

    mLogFileDataRaw = new QFile(QString("scannerdata-raw-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileDataRaw->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileDataRaw->fileName()));

    mLogFileDataGlobal = new QFile(QString("scannerdata-global-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileDataGlobal->open(QIODevice::ReadWrite | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileDataGlobal->fileName()));

    mIsEnabled = true;

    mScannerPoseFirst = 0;
    mScannerPoseBefore = 0;
    mScannerPoseAfter = 0;
    mScannerPoseLast = 0;

    mScanDistancesPrevious = new std::vector<long>;
//     scanDistances = new std::vector<long>;
    mScanDistancesNext = new std::vector<long>;

    if(mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "succeeded.";
    }
    else
    {
        emit message(Error, "LaserScanner::LaserScanner()", "Connecting to " + mDeviceFileName + " failed: UrgCtrl::connect gave: " + QString(mScanner.what()));
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
        return;
    }

    mScanner.setCaptureMode(qrk::IntensityCapture);

    //QTimer::singleShot(mScanner.scanMsec(), this, SLOT(slotSimulateScanning()));
}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner(): shutting down laserscanner...";
    mScanner.setLaserOutput(false);
    mScanner.stop();
    mScanner.disconnect();

    // If we have global data, write a new file in PLY format
    if(mLogFileDataGlobal->size())
    {
        QFile logFileDataPly(QString("scannerdata-global-%1-%2.ply").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
        if(!logFileDataPly.open(QIODevice::WriteOnly | QIODevice::Text))
            qFatal("LaserScanner::~LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(logFileDataPly.fileName()));

        // Create ply header, "element vertex" requires count argument
        logFileDataPly.write(QString("ply\nformat ascii 1.0\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n").arg(mNumberOfScannedPoints).toAscii());

        // Read the processed scanned points, then write ply file
        // Seek to the file's beginning before reading it
        qDebug() << "LaserScanner::~LaserScanner(): writing pointcloud file of about" << ((mNumberOfScannedPoints*24.5f)/1000000.0f) << "mb, this might take some time...";
        mLogFileDataGlobal->reset();
        while (!mLogFileDataGlobal->atEnd())
        {
            const QByteArray line = mLogFileDataGlobal->readLine();
            if(!line.contains("comment")) logFileDataPly.write(line);
        }

        logFileDataPly.close();
        qDebug() << "LaserScanner::~LaserScanner(): done writing ply file.";
    }

    // Close other logfiles
    mLogFileDataGlobal->close();
    mLogFileDataRaw->close();

    // Delete logfiles with a size of 0 (emtpty) or 100 (just ply header, no data)
    const QFileInfoList list = QDir().entryInfoList(QStringList("scannerdata-*"), QDir::Files | QDir::NoSymLinks);
    for(int i = 0; i < list.size(); ++i)
    {
        const QFileInfo fileInfo = list.at(i);
        if(fileInfo.size() == 0 || fileInfo.size() == 100)
        {
            qDebug() << "LaserScanner::~LaserScanner(): moving useless logfile to /tmp:" << fileInfo.fileName();
            QFile::rename(fileInfo.canonicalFilePath(), fileInfo.fileName().prepend("/tmp/"));
        }
    }
}

void LaserScanner::slotSimulateScanning()
{
//    Q_ASSERT(mScanner.isConnected() && "not connected");

    static quint32 counter = 0;
    const quint32 timeInMs = 1000.0 * (counter / (1000.0 / mScanner.scanMsec()));

    const float currentYaw = fmod((timeInMs/1000.0)*10, 360.0);

    qDebug() << "LaserScanner::slotSimulateScanning(): runtime is" << timeInMs/1000.0 << ", yaw is " << currentYaw << "degrees";

    if(counter % 2 == 0)
    {
        // send a new pose every second scan...
        slotNewVehiclePose(
                    Pose(
                        QVector3D(0, 0, 0),
                        currentYaw, // thats yawing CCW as seen from top.
                        0, // look up (~28.76 degree when resting on landing gear and long arm)
                        0.0,
                        timeInMs
                        )
                    );


//        getWorldPositionOfScannedPoint(*mScannerPoseLast, 900, 1.0);
//        qFatal("ende, index of 135 deg is %d", mScanner.deg2index(135));
    }

    // Process laserscanner data.
    slotScanFinished(timeInMs);

    counter++;
    QTimer::singleShot(mScanner.scanMsec(), this, SLOT(slotSimulateScanning()));
}

void LaserScanner::slotLogScannedPoints(const QVector3D& vehiclePosition, const QVector<QVector3D>& points)
{
    qDebug() << "LaserScanner::logScannedPoints(): logging" << points.size() << "points.";
    QTextStream out(mLogFileDataGlobal);
    out << "comment: " << points.size() << " points scanned from world pos: " << vehiclePosition.x() << " " << vehiclePosition.y() << " " << vehiclePosition.z() << "\n";

    for (int i = 0; i < points.size(); ++i)
        out << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << "\n";
}

bool LaserScanner::isScanning(void) const
{
    return mScannerPoseFirst !=0;
    return mIsEnabled;
}

float LaserScanner::getHeightAboveGround() const
{
    // WARNING: the index and offset can be calculated from the pose.
    const int index = mScanner.deg2index(-90);
//     if(scanDistances->size() > index)
//     {
//         return (float)((*scanDistances)[index]) - 0.21;
//     }
//     else
//     {
        // WARNING: this might cause trouble.
        return -1.0;
//     }
}

void LaserScanner::transformScanData()
{
    // We have scan data from previous scans in mLastScans and poses in mLastPoses, lets work out the world coordinates.

    // How much time difference from a scan to a pose (in past of future) for the pose to be usable for interpolation?
    const quint8 maximumMillisecondsBetweenPoseAndScan = 60;

    // Delete poses that are far older than the first/oldest scan. These cannot be used anymore because we can only get newer scans from now on.
    while(mSavedScans.size() && mSavedPoses.at(0).timestamp + maximumMillisecondsBetweenPoseAndScan < mSavedScans.begin().key())
    {
        qDebug() << "LaserScanner::transformScan(): removing first pose from" << mSavedPoses.at(0).timestamp << "because the first of" << mSavedScans.size() << "scans was much later at" << mSavedScans.begin().key();
        mSavedPoses.removeFirst();
    }

    // Delete scans that are far older than the first/oldest pose. These cannot be used anymore because we can only get newer poses from now on.
    while(mSavedScans.size() && mSavedPoses.at(0).timestamp - maximumMillisecondsBetweenPoseAndScan > mSavedScans.begin().key())
    {
        qDebug() << "LaserScanner::transformScan(): removing first scan from" << mSavedScans.begin().key() << "because the first of" << mSavedPoses.size() << "poses was much later at" << mSavedPoses.at(0).timestamp;
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
                const float scanTime = (float)mScanner.scanMsec();
                const quint32 timeOfThisRay = timestampScanMiddle - (mScanner.scanMsec()/2) // when this scan started 180deg in the rear
                                              + scanTime / 8.0f // after running 45degree, (1/8th of angular view), it records first ray
                                              + (scanTime * 0.75f * ((float)index) / ((float)scanDistances->size()));

                qDebug() << "LaserScanner::slotScanFinished(): scanmiddle at" << timestampScanMiddle << "ray-index is" << index << "raytime is" << timeOfThisRay << "before" << posesForThisScan[1]->timestamp << "after" << posesForThisScan[2]->timestamp;

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
                qDebug() << "LaserScanner::transformScan(): couldn't find 4 poses for scan from" << timestampScanMiddle << ", latest pose is from" << mSavedPoses.last().timestamp << ", deleting scan";
                i = mSavedScans.erase(i);
            }
        }
    }

    qDebug() << "LaserScanner::transformScan(): after processing all data, there's" << mSavedScans.size() << "scans and" << mSavedPoses.size() << "poses left.";
}

QVector3D LaserScanner::getWorldPositionOfScannedPoint(const Pose& scannerPose, const quint16 scannerIndex, const float& distance) const
{
    // Determine vector from laserscanner to scanned point, using the normal OpenGl coordinate system as seen from scanner,
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

void LaserScanner::slotNewVehiclePose(const Pose& pose)
{
    QMutexLocker locker(&mMutex);
//    qDebug() << "LaserScanner::slotNewVehiclePose(): received a gps pose" << pose;

    // Write log data: pose[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
    QTextStream out(mLogFileDataRaw);
    out << "pose " << pose.timestamp << " x" << pose.position.x() << " y" << pose.position.y() << " z" << pose.position.z() << " p" << pose.getPitchDegrees() << " r" << pose.getRollDegrees() << " y" << pose.getYawDegrees() << "\n";

    if(!mIsEnabled) return;

    // Append pose to our list
    mSavedPoses.append(Pose(pose + mRelativePose));

    mLastTimeOfPoseOrScan = std::max(pose.timestamp, mLastTimeOfPoseOrScan);

    // Make sure the timestamp from the incoming pose has survived the mangling.
    if(mSavedPoses.last().timestamp != pose.timestamp)
        qDebug() << "LaserScanner::slotNewVehiclePose(): setting laserscanner pose, incoming t" << pose.timestamp
                 << "mRelativePose t" << mRelativePose.timestamp
                 << "resulting t" << mScannerPoseLast->timestamp;

}

void LaserScanner::slotSetScannerTimeStamp(const quint32& timestamp)
{
    // We were called after the host was synchronized with the GPS clock, so @timestamp
    // should be pretty much now.

    // Because Hokuyo UTM30LX only supports time-stamp values up to 2^24=16M milliseconds and
    // starts with 0 on bootup, it wraps after 4.66 hours. This means, we cannot feed the GPS
    // TOW into the scanner, that would only work from Sunday Morning/Midnight to 04:39:37 in
    // the morning. So what now?
    //
    // We ask the scanner for its current timestamp and use that information to create a
    // mapping from scanner-timestamp to GPS' TOW.

    mScanner.recentTimestamp()

            wtf?

    mScanner.setTimestamp(timestamp);
}

void LaserScanner::slotEnableScanning(const bool& value)
{
    mIsEnabled = value;
}


void LaserScanner::slotScanFinished(const quint32 &timestamp)
{
    QMutexLocker locker(&mMutex);
//     qDebug() << t() << "LaserScanner::slotScanFinished(): scanner finished a scan at time" << timestamp;

    if(!mIsEnabled) return;

    const quint32 timeStampMiddleOfScan = timestamp - (mScanner.scanMsec() / 2);

    mLastTimeOfPoseOrScan = std::max(timeStampMiddleOfScan, mLastTimeOfPoseOrScan);

    // We now have a problem: The hokuyo expects us to retrieve the data within 2ms. So, lets retrieve it quickly:
    // (only if we have enough poses to interpolate that scan, true after 4 scans)
//     std::vector<long>* peter = new std::vector<long>;
    long timestampScanner;
    int numRays= mScanner.capture(*mScanDistancesNext, &timestampScanner);
    qDebug() << "sys" << getCurrentGpsTowTime() << "gps" << timestamp << "means" << getCurrentGpsTowTime() - timestamp << "ms delay, scanner timestamp" << timestampScanner << "numrays:" << numRays;
    
    if(numRays <= 0)
    {
//         qWarning() << "LaserScanner::slotScanFinished(): weird, less than 1 samples received from lidar";
//         mSavedScans.remove(timeStampMiddleOfScan);
    }
    else
    {
      
      mSavedScans.insert(timeStampMiddleOfScan, *mScanDistancesNext);
    }

    // Write log data: scan[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
    QTextStream out(mLogFileDataRaw);
    out << "scan " << timestamp;
    std::vector<long>::iterator itr;
    for(itr=mSavedScans[timeStampMiddleOfScan].begin();itr != mSavedScans[timeStampMiddleOfScan].end(); ++itr) out << " " << *itr;
    out << "\n";

    if(mSavedScans.size() > 1 && mSavedPoses.size() > 3)
        transformScanData();
}
