#include "laserscanner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>

LaserScanner::LaserScanner(const QString &deviceFileName, const Pose &relativeScannerPose, const QString& logFilePrefix) : QObject(), mRelativeScannerPose(relativeScannerPose)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mDeviceFileName = deviceFileName;

//    mLogFile = new LogFile(logFilePrefix + QString("scannerdata.lsr"), LogFile::Encoding::Binary);

//    mLastScannerTimeStamp = 0;

    mHokuyo = new Hokuyo(logFilePrefix);

    if(mHokuyo->open(mDeviceFileName))
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "succeeded.";
    }
    else
    {
        qDebug() << "LaserScanner::LaserScanner(): connecting to" << mDeviceFileName << "failed";
        qFatal("LaserScanner::LaserScanner(): exiting.");
    }

    mThreadReadScanner = new QThread;
    mHokuyo->moveToThread(mThreadReadScanner);

    // These should come before the worker slot is connected. Otherwise, slotThreadStarted() is called
    // after the thread has ended. Weird...
    connect(mThreadReadScanner, SIGNAL(started()), this, SLOT(slotThreadStarted()));
    connect(mThreadReadScanner, SIGNAL(finished()), this, SLOT(slotThreadFinished()));
    connect(mThreadReadScanner, SIGNAL(terminated()), this, SLOT(slotThreadTerminated()));

    connect(mThreadReadScanner, SIGNAL(started()), mHokuyo, SLOT(slotProcessScans()));
    connect(mHokuyo, SIGNAL(finished()), mThreadReadScanner, SLOT(quit()));

    connect(mHokuyo, SIGNAL(heightOverGround(float)), SIGNAL(heightOverGround(float)));
    connect(mHokuyo, SIGNAL(newScanData(qint32,std::vector<quint16>*const)), SIGNAL(newScanData(qint32,std::vector<quint16>*const)));
}

LaserScanner::~LaserScanner()
{
    if(!mThreadReadScanner->isRunning())
    {
        qDebug() << "LaserScanner::~LaserScanner(): laserscanner still running, stopping...";
        slotEnableScanning(false);
        qDebug() << "LaserScanner::~LaserScanner(): laserscanner shut down sucessfully";
    }

    delete mThreadReadScanner;
    delete mHokuyo;

    qDebug() << "LaserScanner::~LaserScanner(): done.";
}

const bool LaserScanner::isScanning() const
{
//    return mTimerScan->isActive();
}

const Pose& LaserScanner::getRelativePose() const
{
    return mRelativeScannerPose;
}

/*
void LaserScanner::slotSetScannerTimeStamp(const qint32 timestamp)
{
    // We were called after the host was synchronized with the GPS clock, so @timestamp
    // should be pretty much now.

    // Now that everything is running, we should

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
}*/

void LaserScanner::slotEnableScanning(const bool value)
{
    if(mThreadReadScanner->isRunning() != value)
    {
        qDebug() << "LaserScanner::slotEnableScanning(): setting enabled state to" << value;

        if(value)
        {
            // The scanner will determine latency if it hasn't been determined yet. It is important
            // to do this in its own thread, so that we continue processing in this thread, keeping
            // the load on system and USB bus constant during latency determination.
            mThreadReadScanner->start();
            qDebug() << "LaserScanner::slotEnableScanning(): started!";
        }
        else
        {
            // Request scanning stop. Actual stopping might happen later.
            mHokuyo->slotStopScanning();
            qDebug() << "LaserScanner::slotEnableScanning(): requested scanning thread to stop, now waiting for it to happen...";
        }
    }
}

void LaserScanner::slotThreadStarted()
{
    qDebug() << GnssTime::currentTow() << "LaserScanner::slotThreadStarted() in process" << getpid() << "thread" << pthread_self();
}

void LaserScanner::slotThreadFinished()
{
    qDebug() << GnssTime::currentTow() << "LaserScanner::slotThreadFinished() in process" << getpid() << "thread" << pthread_self();
}

void LaserScanner::slotThreadTerminated()
{
    qDebug() << GnssTime::currentTow() << "LaserScanner::slotThreadTerminated() in process" << getpid() << "thread" << pthread_self();
}
