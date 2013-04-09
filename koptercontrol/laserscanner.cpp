#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &relativeScannerPose, const QString& logFilePrefix) : QObject(), mRelativeScannerPose(relativeScannerPose)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mDeviceFileName = deviceFileName;

    mLogFile = new LogFile(logFilePrefix + QString("scannerdata.lsr"), LogFile::Encoding::Binary);

    mLastScannerTimeStamp = 0;

    mOffsetTimeScannerToTow = 0;

    mHeightOverGroundClockDivisor = 0;

    mTimerScan = new QTimer(this);
    connect(mTimerScan, SIGNAL(timeout()), SLOT(slotCaptureScanData()));

    if(mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "succeeded.";
    }
    else
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
        qFatal("LaserScanner::LaserScanner(): exiting.");
    }

//    mScanner.setCaptureMode(qrk::IntensityCapture);

}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner(): shutting down laserscanner and closing logfile...";
    mScanner.setLaserOutput(false);
    mScanner.stop();
    mScanner.disconnect();
    mTimerScan->stop();
    mTimerScan->deleteLater();

    delete mLogFile;

    qDebug() << "LaserScanner::~LaserScanner(): done.";
}

const bool LaserScanner::isScanning() const
{
    return mTimerScan->isActive();
}

const Pose& LaserScanner::getRelativePose() const
{
    return mRelativeScannerPose;
}

/*
const float LaserScanner::getHeightAboveGround() const
{
    // WARNING: the index and offset can be calculated from the pose.
    const int index = mScanner.deg2index(180);
    // FIXME: need data!
    return -1.0;
}*/

void LaserScanner::slotSetScannerTimeStamp(const qint32 timestamp)
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

    // We only want to set the laserscanner's clock once on startup, because re-setting it
    // means toggling laser power, hence causing data loss when flying. Also, the clock seems
    // to be pretty stable, so that shouldn't be detrimental to data quality
    if(mOffsetTimeScannerToTow == 0)
    {
        mOffsetTimeScannerToTow = timestamp + 2; // additional delay for buggy UrgCtrl::setTimestamp()
        mScanner.setTimestamp(0);
        qDebug() << "LaserScanner::slotSetScannerTimeStamp(): setting laserscanner time once.";
    }
    else
    {
        qDebug() << "LaserScanner::slotSetScannerTimeStamp(): not setting laserscanner time again.";
    }
}

void LaserScanner::slotEnableScanning(const bool value)
{
    if(mOffsetTimeScannerToTow == 0)
    {
        qDebug() << "LaserScanner::slotEnableScanning(): scannertime still unset, should not enable scanning.";
        // disable for logreplay: return;
    }

    if(mTimerScan->isActive() != value)
    {
        qDebug() << "LaserScanner::slotEnableScanning(): setting enabled state to" << value;

        if(value)
        {
            // 2013-04-08: disabled for now, we want to see if not-capturing decreases pose-latency in flightcontroller!
            //mTimerScan->start(mScanner.scanMsec());
        }
        else
        {
            mTimerScan->stop();
        }
    }
}

void LaserScanner::slotCaptureScanData()
{
    if(!mTimerScan->isActive()) qDebug() << "LaserScanner::slotCaptureScanData(): scanning not enabled, but slotCaptureScanData() still called. Expect trouble.";
    mHeightOverGroundClockDivisor++;
    mHeightOverGroundClockDivisor %= (500 / mTimerScan->interval()); // Emit heightOverGround twice per second

    long timestampScanner = 0;
    const int numRays = mScanner.capture(mScannedDistances, &timestampScanner);

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
        // Emit the scandata and add 12msecs of time. The scanner PROBABLY sets the time to the beginning of
        // each scan -135deg (0deg is front) and our convention is to store the time of the middle of a  scan.
        //emit bottomBeamLength(mScannedDistances[540]/1000.0f);
        qint32 timeStampScanMiddle = mLastScannerTimeStamp + mOffsetTimeScannerToTow + 9;

        // Create a copy of the data in a quarter/half the size by using quint16 instead of long (32bit on x86_32, 64bit on x86_64)
	// This next line is untested.
        std::vector<quint16>* distancesToEmit = new std::vector<quint16>(mScannedDistances.begin(), mScannedDistances.end());

        // Always write log data in binary format for later replay. Format is:
        // PackageLengthInBytes(quint16) TOW(qint32) StartIndex(quint16) N-DISTANCES(quint16)
        // StarIndex denotes the start of usable data (not 1s)

        // A usual dataset contains 200 1's at the beginning and 200 1's at the end.
        // We RLE-compress the leading 1s and drop the trailing 1s
        quint16 indexStart = 0;
        while((*distancesToEmit)[indexStart] == 1)
            indexStart++;

        quint16 indexStop = numRays-1;
        while((*distancesToEmit)[indexStop] == 1)
            indexStop--;

        // Write the total amount of bytes of this scan into the stream
        quint16 length = 5 // LASER
                + sizeof(quint16) // length at beginning
                + sizeof(qint32) // timeStampScanMiddle
                + sizeof(quint16) // indexStart
                + ((indexStop - indexStart ) + 1) * sizeof(quint16); // number of bytes for the distance-data

        QByteArray magic("LASER");

        mLogFile->write(magic.constData(), magic.size());
        mLogFile->write((const char*)&length, sizeof(length));
        mLogFile->write((const char*)&timeStampScanMiddle, sizeof(timeStampScanMiddle));
        mLogFile->write((const char*)&indexStart, sizeof(indexStart));

        // Instead of looping through the indices, lets write everything at once.
        mLogFile->write(
                    (const char*)(distancesToEmit->data() + indexStart), // where to start writing.
                    sizeof(quint16) * ((indexStop - indexStart) + 1)     // how many bytes to write
                    );

        // Every full moon, emit the distance from vehicle center to the ground in meters (scanner to vehicle center is 3cm)
        if(mHeightOverGroundClockDivisor == 0) emit heightOverGround(distancesToEmit->at(540)/1000.0f + 0.03f);

        // With this call, we GIVE UP OWNERSHIP of the data. It might get deleted immediately!
        emit newScanData(timeStampScanMiddle, distancesToEmit);
    }
}
