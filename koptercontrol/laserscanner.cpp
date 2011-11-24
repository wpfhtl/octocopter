#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mDeviceFileName = deviceFileName;
    mNumberOfScannedPoints = 0;

    mLogFileScanData = new QFile(QString("scannerdata-raw-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFileScanData->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("LaserScanner::LaserScanner(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFileScanData->fileName()));

    mIsEnabled = false;

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

    mLogFileScanData->close();
    mLogFileScanData->deleteLater();

    // Delete logfiles with a size of 0 (emtpty) or 100 (just ply header, no data)
    const QFileInfoList list = QDir().entryInfoList((QStringList() << "scannerdata-*" << "pointcloud-*"), QDir::Files | QDir::NoSymLinks);
    for(int i = 0; i < list.size(); ++i)
    {
        const QFileInfo fileInfo = list.at(i);
        if(fileInfo.size() == 0 || fileInfo.size() == 100)
        {
            qDebug() << "LaserScanner::~LaserScanner(): moving useless logfile to /tmp:" << fileInfo.fileName();
            QFile::rename(fileInfo.canonicalFilePath(), fileInfo.fileName().prepend("/tmp/"));
        }
    }

    mTimerScan->deleteLater();
}

const bool LaserScanner::isScanning() const
{
    return mIsEnabled;
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

    mOffsetTimeScannerToTow = timestamp;

    mScanner.setTimestamp(2); // additional delay for buggy UrgCtrl::setTimestamp().
}

void LaserScanner::slotEnableScanning(const bool& value)
{
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

    long timestampScanner;
    std::vector<long> distances;
    const int numRays = mScanner.capture(distances, &timestampScanner);

    if(numRays <= 0)
    {
         qWarning() << "LaserScanner::slotScanFinished(): weird, less than 1 samples received from lidar";
    }
    else
    {
        // Write log data: scan[space]timestamp[space]V1[space]V2[space]...[space]Vn\n
        QTextStream out(mLogFileScanData);
        out << "scan " << timestamp;
        std::vector<long>::iterator itr;
        for(itr=mSavedScans[timeStampMiddleOfScan].begin();itr != mSavedScans[timeStampMiddleOfScan].end(); ++itr) out << " " << *itr;
        out << "\n";

        emit newScanData(timestampScanner, distances);
    }
}
