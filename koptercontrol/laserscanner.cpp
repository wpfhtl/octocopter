#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &pose)
{
    qDebug() << "LaserScanner::LaserScanner()";

    mDeviceFileName = deviceFileName;
    mRelativePose = pose;
    mNumberOfScannedPoints = 0;

    mLogFileDataRaw = new QFile(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("scannerdata-raw-").append(".log"));
    if(!mLogFileDataRaw->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileDataRaw->fileName()));

    mLogFileDataGlobal = new QFile(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("scannerdata-global-").append(".log"));
    if(!mLogFileDataGlobal->open(QIODevice::ReadWrite | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileDataGlobal->fileName()));

    mIsEnabled = false;

    mScannerPoseFirst = mScannerPoseBefore = mScannerPoseAfter = mScannerPoseLast = 0;

    mScanDistancesPrevious = new std::vector<long>;
    mScanDistancesCurrent = new std::vector<long>;
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

    // Read the processed scanned points, then write ply file
    // Seek to the file's beginning before reading it
    mLogFileDataGlobal->reset();
    QByteArray plyContents = mLogFileDataGlobal->readAll();

    // Then write a new file
    QFile logFileDataPly(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("scannerdata-global-").append(".ply"));
    if(!logFileDataPly.open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(logFileDataPly.fileName()));

    // Create ply header, "element vertex" requires count argument
    QTextStream out(&logFileDataPly);
    out << "ply\nformat ascii 1.0\nelement vertex " << mNumberOfScannedPoints << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    out.flush();

    // Now write points to new file
    logFileDataPly.write(plyContents);
    logFileDataPly.close();

    // Close other logfiles
    mLogFileDataGlobal->close();
    mLogFileDataRaw->close();
}

void LaserScanner::slotSimulateScanning()
{
//    Q_ASSERT(mScanner.isConnected() && "not connected");

    mIsEnabled = true;

    static quint32 counter = 0;
    const quint32 timeInMs = 1000.0 * (counter / (1000.0 / mScanner.scanMsec()));

    const float currentYaw = fmod((timeInMs/1000.0)*10, 360.0);

    qDebug() << "LaserScanner::slotSimulateScanning(): runtime is" << timeInMs/1000.0 << ", yaw is " << currentYaw << "degrees";

    if(counter == 0)
    {
        qDebug() << "LaserScanner::slotSimulateScanning(): connecting laserpoint emit to logwrite...";
        connect(this, SIGNAL(newScannedPoints(QVector3D,QVector<QVector3D>)), this, SLOT(slotLogScannedPoints(QVector3D,QVector<QVector3D>)));
    }

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
    //out << "comment " << points.size() << " points scanned from world pos: " << vehiclePosition.x() << " " << vehiclePosition.y() << " " << vehiclePosition.z() << "\n";

    for (int i = 0; i < points.size(); ++i)
        out << points.at(i).x() << " " << points.at(i).y() << " " << points.at(i).z() << "\n";

//    if(points.size()) out << "\n";
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
//    qDebug() << "LaserScanner::slotScanFinished(): scanner finished a scan at time" << timestamp;
Q_ASSERT(false && "WOOOOOHOOOOOO, FLANKE VOM SCANNER!");

    // We now have a problem: The hokuyo expects us to retrieve the data within 2ms. So, lets retrieve it quickly:
    // (only if we have enough poses to interpolate that scan, true after 4 scans)
    if(mScannerPoseBefore != 0)
    {
        mScanDistancesNext = mScanDistancesPrevious;
        mScanDistancesPrevious = mScanDistancesCurrent;
        mScanDistancesCurrent = mScanDistancesNext;

        if(mScanner.capture(*mScanDistancesNext) <= 0) qWarning() << "LaserScanner::slotScanFinished(): weird, less than 1 samples received from lidar";

        // Write log data: scan[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
        QTextStream out(mLogFileDataRaw);
        out << "scan " << timestamp;
        std::vector<long>::iterator itr;
        for(itr=mScanDistancesNext->begin();itr != mScanDistancesNext->end(); ++itr) out << " " << *itr;
        out << "\n";

        if(mScannerPoseFirst != 0 && mIsEnabled)
        {
            // Now we have scan data from the previous scan in mScanDistancesPrevious and enough
            // poses around it to interpolate. Go ahead and convert this data to 3d cloud points.
            QVector<QVector3D> scannedPoints;/*(mScanDistancesCurrent->size());*/ // Do not reserve full length, will be less poins due to reflections on the vehicle being filtered

            for(int index=0; index < mScanDistancesCurrent->size(); index++)
            {
                // Convert millimeters to meters.
                const float distance = (*mScanDistancesCurrent)[index] / 1000.0;

                // Skip reflections on vehicle (=closer than 50cm)
//                if(rayLength < 0.5) continue;

                // Interpolate using the last 4 poses. Do NOT interpolate between 0.0 and 1.0, as
                // the scan actually only takes place between 0.125 and 0.875 (the scanner only
                // scans the central 270 degrees of the 360 degree-circle).
                Pose interpolatedPose = Pose::interpolateCubic(
                            mScannerPoseFirst,
                            mScannerPoseBefore,
                            mScannerPoseAfter,
                            mScannerPoseLast,
                            (float)(0.125 + (0.75 * index / mScanDistancesCurrent->size()))
                            );

                scannedPoints.append(getWorldPositionOfScannedPoint(interpolatedPose, index, distance));
                mNumberOfScannedPoints++;
            }

//            qDebug() << "LaserScanner::slotScanFinished(): now emitting" << scannedPoints.size() << "points generated from a vector of" << mScanDistancesCurrent->size() << "points.";

            emit newScannedPoints(
                        Pose::interpolateCubic(
                            mScannerPoseFirst,
                            mScannerPoseBefore,
                            mScannerPoseAfter,
                            mScannerPoseLast,
                            (float)0.5
                            ).position,
                        scannedPoints
                        );
        }
        else qDebug() << "LaserScanner::slotScanFinished(): either not enabled or mScannerPoseFirst is still 0 (happens 3 times?!)";
    }
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
    qDebug() << "LaserScanner::slotNewVehiclePose(): received a gps pose" << pose;

    // Write log data: pose[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
    QTextStream out(mLogFileDataRaw);
    out << "pose t" << pose.timestamp << " x" << pose.position.x() << " y" << pose.position.y() << " z" << pose.position.z() << " p" << pose.getPitchDegrees() << " r" << pose.getRollDegrees() << " y" << pose.getYawDegrees() << "\n";

    delete mScannerPoseFirst;

    mScannerPoseFirst = mScannerPoseBefore;
    mScannerPoseBefore = mScannerPoseAfter;
    mScannerPoseAfter = mScannerPoseLast;

    qDebug() << "LaserScanner::slotNewVehiclePose(): setting laser pose";
    mScannerPoseLast = new Pose(pose + mRelativePose);
    qDebug() << "LaserScanner::slotNewVehiclePose(): set laser pose" << *mScannerPoseLast;

    // Make sure the timestamp from the incoming pose has survived the mangling.
// FAILS; CHECK THIS!!    Q_ASSERT(pose.timestamp == mScannerPoseLast->timestamp);
}

void LaserScanner::slotEnableScanning(const bool& value)
{
    mIsEnabled = value;
}
