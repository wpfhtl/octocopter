#include "hokuyo.h"
//#include <sys/types.h>
#include <unistd.h>

Hokuyo::Hokuyo(const QString& logFilePrefix) : QObject()
{
    qDebug() << "Hokuyo::Hokuyo(): initializing Hokuyo";

    mLogFile = new LogFile(logFilePrefix + QString("scannerdata.lsr"), LogFile::Encoding::Binary);

    mLastScannerTimeStamp = 0;

    mOffsetTimeScannerToTow = 0;

    mHeightOverGroundClockDivisor = 0;

    mState = State::Stopped;
}

bool Hokuyo::open(const QString& deviceFilename)
{
    if(mScanner.connect(qPrintable(deviceFilename)))
    {
        qDebug() << "Hokuyo::Hokuyo(): connecting to" << deviceFilename << "succeeded.";
        return true;
    }
    else
    {
        qDebug() << "Hokuyo::Hokuyo(): connecting to" << deviceFilename << "failed: UrgCtrl::connect gave" << mScanner.what();
        return false;
    }

    // mScanner.setCaptureMode(qrk::IntensityCapture);
}

Hokuyo::~Hokuyo()
{
    qDebug() << "Hokuyo::~Hokuyo(): shutting down laserscanner and closing logfile...";
    mScanner.setLaserOutput(false);
    mScanner.stop();
    mScanner.disconnect();

    delete mLogFile;

    qDebug() << "Hokuyo::~Hokuyo(): done.";
}

void Hokuyo::slotSetScannerTimeStamp(const qint32 timestamp)
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
        qDebug() << "Hokuyo::slotSetScannerTimeStamp(): setting laserscanner time once.";
        mScanner.setTimestamp(0);
    }
    else
    {
        qDebug() << "Hokuyo::slotSetScannerTimeStamp(): not setting laserscanner time again.";
    }
}

void Hokuyo::slotStartScanning()
{
    qDebug() << GnssTime::currentTow() << "Hokuyo::slotStartScanning(): starting in process" << getpid() << "thread" << pthread_self();

    if(mOffsetTimeScannerToTow == 0)
    {
        qDebug() << __PRETTY_FUNCTION__ << "scanner time not set, will not scan!";
        return;
    }

    mState = State::Scanning;

    do
    {
        mHeightOverGroundClockDivisor++;
        mHeightOverGroundClockDivisor %= 20; // Emit heightOverGround every 20 scans, so thats 2Hz

        long timestampScanner = 0;
        const int numRays = mScanner.capture(mScannedDistances, &timestampScanner);
        // TODO: handle wrapping of lasertime after 4.66 hours!
        mLastScannerTimeStamp = timestampScanner;

        if(numRays <= 0)
        {
             qWarning() << "Hokuyo::slotScanFinished(): weird, less than 1 samples received from lidar";
        }
        else
        {
            // Emit the scandata and add 12msecs of time. The scanner PROBABLY sets the time to the beginning of
            // each scan -135deg (0deg is front) and our convention is to store the time of the middle of a scan.


            qint32 timeStampScanMiddle = mLastScannerTimeStamp + mOffsetTimeScannerToTow + 9;

            // Create a copy of the data in a quarter/half the size by using quint16 instead of long (32bit on x86_32, 64bit on x86_64)
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
                        sizeof(quint16) * ((indexStop - indexStart) + 1) // how many bytes to write
                        );

            // Every full moon, emit the distance from vehicle center to the ground in meters (scanner to vehicle center is 3cm)
            if(mHeightOverGroundClockDivisor == 0 && mScannedDistances.size() > 540)
                emit heightOverGround(distancesToEmit->at(540)/1000.0f + 0.03f);

            // With this call, we GIVE UP OWNERSHIP of the data. It might get deleted immediately!
            emit newScanData(timeStampScanMiddle, distancesToEmit);
        }
    } while (mState == State::Scanning);

    Q_ASSERT(mState == State::StopRequested);

    qDebug() << "Hokuyo::slotStartScanning(): stop requested, setting state to stopped";
    mState = State::Stopped;

    qDebug() << "Hokuyo::slotStartScanning(): emiting finished()...";
    emit finished();
    qDebug() << "Hokuyo::slotStartScanning(): ...done.";
}
