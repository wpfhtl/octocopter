#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &relativeScannerPose) : QObject(), mRelativeScannerPose(relativeScannerPose)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mDeviceFileName = deviceFileName;

    mLogFile = new QFile(QString("log/log-%1-%2-scannerdata.lsr").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
    if(!mLogFile->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));
    else
        qDebug() << "LaserScanner::LaserScanner(): Successfully opened logfile" << mLogFile->fileName() << "for writing";

    QTextStream out(mLogFile);
    out << "first line test!" << endl;

    mLastScannerTimeStamp = 0;

    mIsEnabled = false;

    mOffsetTimeScannerToTow = 0;

    mTimerScan = new QTimer(this);
    connect(mTimerScan, SIGNAL(timeout()), SLOT(slotCaptureScanData()));

    if(mScanner.connect(mDeviceFileName.toAscii().constData()))
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "succeeded.";
    }
    else
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed: UrgCtrl::connect gave" << mScanner.what();
        return;
    }

    // Do this after initializing hte scanner, otherwise the values might be off.
    mTimerScan->setInterval(mScanner.scanMsec());

//    mScanner.setCaptureMode(qrk::IntensityCapture);

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

void LaserScanner::slotEnableScanning(const bool& value)
{
    if(mOffsetTimeScannerToTow == 0)
    {
        qDebug() << "LaserScanner::slotEnableScanning(): scannertime still unset, should not enable scanning.";
        // disable for logreplay: return;
    }

    if(mIsEnabled != value)
    {
        mIsEnabled = value;
        qDebug() << "LaserScanner::slotEnableScanning(): setting enabled state to" << value;

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
        // Emit the scandata and add 12msecs of time. The scanner PROBABLY sets the time to the beginning of
        // each scan -135deg (0deg is front) and our convention is to store the time of the middle of a  scan.
        //emit bottomBeamLength((*distances)[540]/1000.0f);
        qint32 timeStampScanMiddle = mLastScannerTimeStamp + mOffsetTimeScannerToTow + 9;

        // Always write log data for later replay: scannerdata:[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
        Q_ASSERT(mLogFile->isOpen());
        QTextStream out(mLogFile);
        out << timeStampScanMiddle;
        std::vector<long>::iterator itr;
        for(itr=distances->begin();itr != distances->end(); ++itr) out << " " << *itr;
        out << endl;

        qDebug() << "LaserScanner::slotCaptureScanData(): wrote" << distances->size() << "scanned values to file, emitting now...";

        // With this call, we GIVE UP OWNERSHIP of the data. It might get deleted immediately!
        emit newScanData(timeStampScanMiddle, distances);
    }
}
