#include "hokuyo.h"
//#include <sys/types.h>
#include <unistd.h>

Hokuyo::Hokuyo(LogFile* const logFile, const QMatrix4x4 * const relativePose, bool isConnectedToEventPin) : QObject()
{
    qDebug() << "Hokuyo::Hokuyo(): initializing Hokuyo";

    mLogFile = logFile;

    mRelativeScannerPose = relativePose;

    mIsConnectedToEventPin = isConnectedToEventPin;

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

bool Hokuyo::synchronizeScannerTime()
{
    // We were called after the host was synchronized with the GPS clock, so we can now use
    // the system time to set the scanner's time.
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

    // We only want to set the laserscanner's clock once on startup, because re-setting it
    // means toggling laser power, hence causing data loss when flying. Also, the clock seems
    // to be pretty stable, so that shouldn't be detrimental to data quality

    const qint32 timestamp = GnssTime::currentTow();

    if(mOffsetTimeScannerToTow == 0)
    {
        mOffsetTimeScannerToTow = timestamp + 2; // additional delay for buggy UrgCtrl::setTimestamp()
        qDebug() << "Hokuyo::slotSetScannerTimeStamp(): setting laserscanner time once to time" << timestamp;
        mScanner.setTimestamp(0);
        return true;
    }
    else
    {
        qDebug() << "Hokuyo::slotSetScannerTimeStamp(): not setting laserscanner time again.";
        return false;
    }
}

void Hokuyo::slotStartScanning()
{
    qDebug() << GnssTime::currentTow() << "Hokuyo::slotStartScanning(): starting in process" << getpid() << "thread" << pthread_self();

    if(mOffsetTimeScannerToTow == 0)
    {
        qDebug() << __PRETTY_FUNCTION__ << "scanner time not set, will scan as soon as time is set!";
    mState = State::ScanRequestedButTimeUnknown;
    emit finished();
        return;
    }

    mState = State::Scanning;

    do
    {
        mHeightOverGroundClockDivisor++;
        mHeightOverGroundClockDivisor %= 20; // Emit heightOverGround every 20 scans, so thats 2Hz

        // TODO: handle wrapping of lasertime after 4.66 hours!
        long timestampScanner = 0;
        if(mScanner.capture(mScannedDistances, &timestampScanner) <= 0)
        {
             qWarning() << "Hokuyo::slotScanFinished(): weird, less than 1 samples received from lidar";
        }
        else
        {
            mLastScannerTimeStamp = timestampScanner;


            RawScan* rawScan = new RawScan;

            rawScan->relativeScannerPose = mRelativeScannerPose;

            // Emit the scandata and add 12msecs of time. The scanner PROBABLY sets the time to the beginning of
            // each scan -135deg (0deg is front) and our convention is to store the time of the middle of a scan.
            rawScan->timeStampScanMiddleScanner = mLastScannerTimeStamp + mOffsetTimeScannerToTow + 9;
            // Set the gnss time to the scanner time if we're not connected to an event-pin.
            if(!mIsConnectedToEventPin) rawScan->timeStampScanMiddleGnss = rawScan->timeStampScanMiddleScanner;

            // Fill the values in the RawScan. It will allocate memory for the values.
            rawScan->setDistances(mScannedDistances);

            rawScan->log(mLogFile);

            //qDebug() << "scan from scanner connected to pin:" << mIsConnectedToEventPin << ": tostring:" << rawScan->toString();

            //mLogFile->write(
                        //(const char*)(distancesToEmit->data() + indexStart), // where to start writing.
                        //sizeof(quint16) * ((indexStop - indexStart) + 1) // how many bytes to write
                        //);

            // Every full moon, emit the distance at the front ray. Can be used for height estimation.
            // Of course, we only do that when there actually IS a measurement
            if(mHeightOverGroundClockDivisor == 0 && mScannedDistances.size() > 540 && rawScan->distances[540] != 0)
            {
                const float distance = rawScan->distances[540]/1000.0f;
                qDebug() << __PRETTY_FUNCTION__ << "emitting distanceAtFront of" << distance;
                emit distanceAtFront(distance);
            }

            // With this call, we GIVE UP OWNERSHIP of the data. It might get deleted immediately!
            //qDebug() << __PRETTY_FUNCTION__ << "emitting scanData(qint32, quint16," << rawScan->numberOfDistances << "), connectedToEventPin:" << mIsConnectedToEventPin;

            emit scanRaw(rawScan);
        }
    } while (mState == State::Scanning);

    Q_ASSERT(mState == State::StopRequested);

    qDebug() << "Hokuyo::slotStartScanning(): stop requested, setting state to stopped";
    mState = State::Stopped;

    qDebug() << "Hokuyo::slotStartScanning(): emiting finished()...";
    emit finished();
    qDebug() << "Hokuyo::slotStartScanning(): ...done.";
}
