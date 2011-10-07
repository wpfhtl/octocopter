#include "laserscanner.h"

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &pose)
{
    qDebug() << "LaserScanner::LaserScanner()";

    mDeviceFileName = deviceFileName;
    mRelativePose = pose;

    mLogFile = new QFile(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("scannerdata-").append(".log"));
    if(!mLogFile->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));

    mIsEnabled = false;

    mPoints.reserve(1200); // should be 1080 points (270 degrees, 0.25° resolution)

    mScannerPoseFirst = mScannerPoseBefore = mScannerPoseAfter = mScannerPoseLast = 0;

    mScanDistancesPrevious = new vector<long>;
    mScanDistancesCurrent = new vector<long>;
    mScanDistancesNext = new vector<long>;

    if (! mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
        emit message(Error, "LaserScanner::LaserScanner()", "Connecting to " + mDeviceFileName + " failed: UrgCtrl::connect gave: " + QString(mScanner.what()));
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
        return;
    }

    mMilliSecondsPerScan = mScanner.scanMsec();
    mScanner.setCaptureMode(AutoCapture); // use IntensityCapture for high distance?
}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner()";

}

/*
void LaserScanner::run()
{

    // We don't want to loop and retrieve scans all the time. Instead, we want to retrieve a scan
    // only after GpsDevice tells us that a scan has finished. This will be handled in a slot below,
    // called by a queued connection from GpsDevice.

    exec();
}
*/

//Pose LaserScanner::getRelativePose(void) const
//{
//    return mRelativePose;
//}

void LaserScanner::slotSetRelativePose(const Pose &pose)
{
    mRelativePose = pose;
}

bool LaserScanner::isScanning(void) const
{
    return mScannerPoseFirst !=0;
    return mIsEnabled;
}

float LaserScanner::getHeightAboveGround() const
{
    // WARNING: the index and offset can be calculated from the pose.
    const int index = mScanner.deg2index(90);
    if(mScanDistancesCurrent->size() > index)
    {
        return (float)((*mScanDistancesCurrent)[index]) - 0.21;
    }
    else
    {
        // WARNING: this might cause trouble.
        return -1.0;
    }
}

void LaserScanner::slotScanFinished(const quint32 &timestamp)
{
    QMutexLocker locker(&mMutex);
    qDebug() << "LaserScanner::slotScanFinished(): scanner finished a scan at time" << timestamp;

    // We now have a problem: The hokuyo expects us to retrieve the data within 2ms. So, lets retrieve it quickly:
    // (only if we have enough poses to interpolate that scan, true after 4 scans)
    if(mScannerPoseBefore != 0)
    {
        mScanDistancesNext = mScanDistancesPrevious;
        mScanDistancesPrevious = mScanDistancesCurrent;
        mScanDistancesCurrent = mScanDistancesNext;

        long timestamp = 0;
        if(mScanner.capture(*mScanDistancesNext, &timestamp) <= 0)
            qWarning() << "LaserScanner::slotScanFinished(): weird, less than 1 samples sent by lidar";

        // Write log data: scan[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
        QTextStream out(mLogFile);
        out << "scan " << timestamp;
        std::vector<long>::iterator itr;
        for(itr=mScanDistancesNext->begin();itr != mScanDistancesNext->end(); ++itr) out << " " << *itr;
        out << "\n";

        if(mScannerPoseFirst != 0 && mIsEnabled)
        {
            // Now we have scan data from the previous scan in mScanDistancesPrevious and enough
            // poses around it to interpolate. Go ahead and convert this data to 3d cloud points.
            for(int i=0; i < mScanDistancesCurrent->size(); i++)
            {
                // Skip reflections on vehicle (=closer than 50cm)
                if((*mScanDistancesCurrent)[i] < 500) continue;

                // Interpolate using the last 4 poses. Do NOT interpolate between 0.0 and 1.0, as
                // the scan actually only takes place between 0.125 and 0.875 (the scanner only
                // scans the central 270 degrees of the 360 degree-circle).
                Pose p = Pose::interpolateCubic(
                            mScannerPoseFirst,
                            mScannerPoseBefore,
                            mScannerPoseAfter,
                            mScannerPoseLast,
                            (float)(0.125 + (0.75 * i / mScanDistancesCurrent->size()))
                            );

                // position of the point relative to the physical laserscanner
                // x is -left/+right, y is -back/+front and z i always 0.
                const QVector3D rayPos(
                            tan(mScanner.index2rad(i)) * (float)((*mScanDistancesCurrent)[i]),
                            (*mScanDistancesCurrent)[i],
                            0.0);

                LidarPoint point(p.getOrientation().rotatedVector(rayPos) + p.position, QVector3D(), (*mScanDistancesCurrent)[i]);
                qDebug() << "scanned lidarpointpoint from" << p << "is" << point;

                mPoints.append(point);
            }

            emit newLidarPoints(mPoints);
            mPoints.clear();
        }
    }

//    sendPointsToBase();
}

void LaserScanner::slotNewVehiclePose(const Pose& pose)
{
    QMutexLocker locker(&mMutex);
//    qDebug() << "LaserScanner::slotNewVehiclePose(): received a pose from gps-device at time" << pose.timestamp;

    // Write log data: pose[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
    QTextStream out(mLogFile);
    out << "pose " << pose.timestamp << " " << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " " << pose.getPitchDegrees() << " " << pose.getRollDegrees() << " " << pose.getYawDegrees() << "\n";

    delete mScannerPoseFirst;

    mScannerPoseFirst = mScannerPoseBefore;
    mScannerPoseBefore = mScannerPoseAfter;
    mScannerPoseAfter = mScannerPoseLast;
    mScannerPoseLast = new Pose(pose + mRelativePose);

    Q_ASSERT(pose.timestamp == mScannerPoseLast->timestamp);

}

void LaserScanner::slotEnableScanning(const bool& value)
{
    mIsEnabled = value;
}
