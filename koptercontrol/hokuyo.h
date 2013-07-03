#ifndef HOKUYO
#define HOKUYO

#include <QtCore>
#include <urg/UrgCtrl.h>
#include <pose.h>
#include "logfile.h"
#include "rawscan.h"

// We don't use lidarpoints on the rover, because storing the ray back to laserscanner from
// each scanned point is a large overhead with little gain. Instead, we store a list of
// QVector3Ds (points in world frame) and also emit the mean position together with that list.

class Hokuyo : public QObject
{
    Q_OBJECT
    
public:
    enum class State
    {
        Scanning,
	ScanRequestedButTimeUnknown,
        StopRequested,
        Stopped
    };

private:
    LogFile* mLogFile;
    qrk::UrgCtrl mScanner;
    quint8 mHeightOverGroundClockDivisor;
    qint64 mOffsetTimeScannerToTow;
    qint32 mLastScannerTimeStamp;
    bool mIsConnectedToEventPin;

    std::vector<long> mScannedDistances;


    State mState;

public:
    // The pose specifies translation from vehicle frame to the laser source, so the scanner's
    // physical dimensions are ignored completely. Further down, this class receives high-
    // frequency updates of the vehicle's poses. Using the vehicle-frame poses and its static
    // offset defined in this constructor, it can emit scanpoints in world-coordinates.
    Hokuyo(LogFile * const logFile, bool isConnectedToEventPin = false);
    ~Hokuyo();

    bool open(const QString &deviceFilename);
    
    State getState() const {return mState;}

public slots:
    // This will not return, but run endlessly, so start it in a new thread!
    void slotStartScanning();

    // Call this from another thread to stop. No mutex necessary, because the scanning thread only reads mState
    void slotStopScanning() { mState = State::StopRequested; }

    // To set the laserscanner's timestamp to the gps time. Hopefully.
    void slotSetScannerTimeStamp();

signals:
    // emitted when scanning -> stop requested -> stopped
    void finished();

    // the distance from the vehicle's center to the ground in meters
    void distanceAtFront(const float&);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);

    // Emits new scan data, allocated on heap. Ownership is passed to receiver(s).
    void scanData(RawScan*);
};

#endif
