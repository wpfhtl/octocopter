#include "laserscanner.h"

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &pose)
{
    qDebug() << "LaserScanner::LaserScanner()";

    mDeviceFileName = deviceFileName;
    mRelativePose = pose;

    mScannerPoseFirst = mScannerPoseBefore = mScannerPoseAfter = mScannerPoseLast = 0;

    mScanDistancesPrevious = new vector<long>;
    mScanDistancesCurrent = new vector<long>;
    mScanDistancesNext = new vector<long>;

    if (! mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
      qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
      exit(1);
    }

    mMilliSecondsPerScan = mScanner.scanMsec();
    mScanner.setCaptureMode(AutoCapture); // use IntensityCapture for high distance?
}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner()";

}

void LaserScanner::run()
{

    // We don't want to loop and retrieve scans all the time. Instead, we want to retrieve a scan
    // only after GpsDevice tells us that a scan has finished. This will be handled in a slot below,
    // called by a queued connection from GpsDevice.

    exec();
}

Pose LaserScanner::getPose(void) const
{
    return mRelativePose;
}

void LaserScanner::setPose(const Pose &pose)
{
    mRelativePose = pose;
}


bool LaserScanner::isScanning(void) const
{
    return mScannerPoseFirst !=0;
}

void LaserScanner::slotScanFinished(Pose* pose)
{
    QMutexLocker locker(&mMutex);
    qDebug() << "LaserScanner::slotScanFinished(): received a scan-pose from gps-device!";

    // The hokuyo finished a scan and sent a falling edge to the GpsDevice, which now notifies us of
    // this scan together with the pose in which scanning finished.

    delete mScannerPoseFirst;

    mScannerPoseFirst = mScannerPoseBefore;
    mScannerPoseBefore = mScannerPoseAfter;
    mScannerPoseAfter = mScannerPoseLast;
    mScannerPoseLast = *pose + mRelativePose;

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

        if(mScannerPoseFirst != 0)
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
                            0.125 + (0.75 * i / mScanDistancesCurrent->size())
                            );

                // position of the point relative to the physical laserscanner
                // x is -left/+right, y is -back/+front and z i always 0.
                const QVector3D rayPos(
                            tan(mScanner.index2rad(i)) * (float)((*mScanDistancesCurrent)[i]),
                            (*mScanDistancesCurrent)[i],
                            0.0);

                QVector3D point = p.orientation.rotatedVector(rayPos) + p.position;
                qDebug() << "scanned point from" << p << "is" << point;

                mPoints.append(point);
            }
        }
    }

//    sendPointsToBase();
}
