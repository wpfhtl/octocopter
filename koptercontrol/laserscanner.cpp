#include "laserscanner.h"

#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

quint32 LaserScanner::instanceCounter = 0;

LaserScanner::LaserScanner(const QString &deviceFileName, const QString& logFilePrefix, bool isConnectedToEventPin) :
    QObject(),
    mDeviceFileName(deviceFileName)
{
    qDebug() << "LaserScanner::LaserScanner(): initializing laserscanner";

    mLogFile = new LogFile(logFilePrefix + QString(".ldr") + QString::number(instanceCounter++), LogFile::Encoding::Binary);

    mHokuyo = new Hokuyo(mLogFile, &mRelativeScannerPose, isConnectedToEventPin);

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
    connect(mThreadReadScanner, &QThread::started, this, &LaserScanner::slotThreadStarted);
    connect(mThreadReadScanner, &QThread::finished, this, &LaserScanner::slotThreadFinished);

    connect(mThreadReadScanner, &QThread::started, mHokuyo, &Hokuyo::slotStartScanning);
    connect(mHokuyo, &Hokuyo::finished, mThreadReadScanner, &QThread::quit);

    connect(mHokuyo, SIGNAL(distanceAtFront(float)), SIGNAL(distanceAtFront(float)));
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

void LaserScanner::slotSetRelativeScannerPose(const Pose& p)
{
    mRelativeScannerPose = p.getMatrixCopy();

    // Write the packet only if system time is synchronized to GNSS time.
    // This must be true when the laserscanner is synrchonized.
    if(mHokuyo->isTimeSynchronized())
    {
        writeRelativeScannerPoseToLogFile();
    }
}

void LaserScanner::writeRelativeScannerPoseToLogFile()
{
    // Write the new relative pose into the logfile. Format is:
    // RPOSE PacketLengthInBytes(quint16) TOW(qint32) QDataStreamedPoseMatrix
    //
    // PacketLengthInBytes is ALL bytes of this packet

    // LogFile is thread-safe, so its ok for us to write an RPOSE while
    // Hokuyo writes its scans!

    const QByteArray magic("RPOSE");

    QByteArray byteArrayPoseMatrix;
    QDataStream ds(&byteArrayPoseMatrix, QIODevice::WriteOnly);
    ds << mRelativeScannerPose;

    quint16 length =
            magic.size()                    // size of MAGIC bytes
            + sizeof(quint16)               // totalPacketLength
            + sizeof(qint32)                // TOW
            + byteArrayPoseMatrix.size();   // size of streamed pose

    const qint32 tow = GnssTime::currentTow();
    qDebug() << __PRETTY_FUNCTION__ << "writing the relative scanner pose to logfile TOW" << tow;

    mLogFile->write(magic.constData(), magic.size());
    mLogFile->write((const char*)&length, sizeof(quint16));
    mLogFile->write((const char*)&tow, sizeof(qint32)); // I hope this lines in well with scanner timestamps...
    mLogFile->write(byteArrayPoseMatrix.constData(), byteArrayPoseMatrix.size());
}

void LaserScanner::slotSetScannerTimeStamp()
{
    // We were called after the host was synchronized with the GPS clock, so @timestamp
    // should be pretty much now.

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

    const bool timeWasSynchronizedForTheFirstTime = mHokuyo->synchronizeScannerTime();
    if(timeWasSynchronizedForTheFirstTime)
    {
        qDebug() << __PRETTY_FUNCTION__ << "scanner time was synchronized successfully, now writing the relative scanner pose to logfile";
        writeRelativeScannerPoseToLogFile();
    }
    
    if(mHokuyo->getState() == Hokuyo::State::ScanRequestedButTimeUnknown)
    {
        qDebug() << __PRETTY_FUNCTION__ << "deferred start after time is known...";
        slotEnableScanning(true);
    }
}

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
