#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &relativeScannerPose) : QObject(), mRelativeScannerPose(relativeScannerPose)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mDeviceFileName = deviceFileName;

    mLastScannerTimeStamp = 0;

    mIsEnabled = false;

    mOffsetTimeScannerToTow = 0;

    if(mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "succeeded.";
    }
    else
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
        return;
    }

//    mScanner.setCaptureMode(qrk::IntensityCapture);

    mTimerScan = new QTimer(this);
    mTimerScan->setInterval(mScanner.scanMsec());
    connect(mTimerScan, SIGNAL(timeout()), SLOT(slotCaptureScanData()));
}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner(): shutting down laserscanner...";
    mScanner.setLaserOutput(false);
    mScanner.stop();
    mScanner.disconnect();
    mTimerScan->stop();
    mTimerScan->deleteLater();
}

const bool LaserScanner::isScanning() const
{
    return mIsEnabled;
}

const Pose& LaserScanner::getRelativePose() const
{
    return mRelativeScannerPose;
}

const float LaserScanner::getHeightAboveGround() const
{
    // WARNING: the index and offset can be calculated from the pose.
    const int index = mScanner.deg2index(180);
    // FIXME: need data!
    return -1.0;
}

void LaserScanner::slotSetScannerTimeStamp(const quint32& timestamp)
{
    // We were called after the host was synchronized with the GPS clock, so @timestamp
    // should be pretty much now.
    //
    // Because Hokuyo UTM30LX only supports time-stamp values up to 2^24=16M milliseconds and
    // starts with 0 on bootup, it wraps after 4.66 hours. This means we cannot feed the GPS
    // TOW into the scanner, that would only work from Sunday Morning/Midnight to 04:39:37 in
    // the morning.
    //
    // The scanner's clock actually cannot be set. Instead, UrgCtrl defines an offset to the
    // scanner's clock when you call UrgCtrl::setTimestamp(time, delay, useless_var). Looking
    // at the method, you'll see that one roundtrip to the scanner to enable timestamp-mode
    // is unaccounted for and adds another ~4ms delay, while the calculated delay should be
    // halved (its not a roundtrip). So, instead of 4ms RTT, it should be 4msRTT + 2ms = 6ms
    // of delay.
    //
    // Whats worse, the clock seems to wrap at arbitrary moments, going from values of e.g.
    // 73975 to 16852193 between two consecutive scans. When this happens, we could call Urg::
    // setTimestamp() method, but that would power-cycle the laser and thus cause data loss.
    // So, we just call this method once and store a second local offset when this happens.

    mOffsetTimeScannerToTow = timestamp + 2; // additional delay for buggy UrgCtrl::setTimestamp()
    mScanner.setTimestamp(0);
}

void LaserScanner::slotEnableScanning(const bool& value)
{
    if(mOffsetTimeScannerToTow == 0)
    {
        qDebug() << "LaserScanner::slotEnableScanning(): scannertime still unset, will not enable scanning.";
        return;
    }

    if(mIsEnabled != value)
    {
        mIsEnabled = value;

        if(mIsEnabled)
            mTimerScan->start();
        else
            mTimerScan->stop();
    }
}

void LaserScanner::slotCaptureScanData()
{
    Q_ASSERT(mIsEnabled && "scanning not enabled, but slotCaptureScanData() still called?!");

    long timestampScanner = 0;
    std::vector<long>* distances = new std::vector<long>;
    const int numRays = mScanner.capture(*distances, &timestampScanner);

    // The scanner's timestamp randomly jumps by values of 2^^24 = 16.8M msecs. Fix it.
    if(timestampScanner != 0 && mLastScannerTimeStamp != 0)
    {
        // If the scanner's value jumps positive, fix it by fixing the offset
        if(timestampScanner - mLastScannerTimeStamp > 15000000)
        {
            mOffsetTimeScannerToTow -= 16777216;
            qDebug() << "LaserScanner::slotCaptureScanData(): WARNING: SCANNER JUMPED +2^24";
            emit message(Warning, "LaserScanner::slotCaptureScanData()", "Scanner timestamp jumped by +2^24");
        }
        else if(timestampScanner - mLastScannerTimeStamp < -15000000)
        {
            mOffsetTimeScannerToTow += 16777216;
            qDebug() << "LaserScanner::slotCaptureScanData(): WARNING: SCANNER JUMPED -2^24";
            emit message(Warning, "LaserScanner::slotCaptureScanData()", "Scanner timestamp jumped by -2^24");
        }
    }

    mLastScannerTimeStamp = timestampScanner;

    if(numRays <= 0)
    {
         qWarning() << "LaserScanner::slotScanFinished(): weird, less than 1 samples received from lidar";
    }
    else
    {
        emit newScanData(mLastScannerTimeStamp + mOffsetTimeScannerToTow, distances);
    }
}
