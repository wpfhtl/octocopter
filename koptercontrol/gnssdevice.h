#ifndef GNSSDEVICE_H
#define GNSSDEVICE_H

#include <QCoreApplication>
#include <QFile>
#include <QDebug>
#include <QTimer>
#include <QtSerialPort/QSerialPort>

#include <unistd.h> // sleep()
#include <sys/time.h> // for syncing time
#include <common.h>
#include "pose.h"
#include "logfile.h"
#include "sbfparser.h"
#include <math.h>
#include <errno.h>

class GnssDevice : public QObject
{
    Q_OBJECT

public:
    GnssDevice(const QString &serialDeviceUsb, const QString &serialDeviceCom, QString logFilePrefix, QObject *parent = 0);
    ~GnssDevice();

    // Our parent (koptercontrol) needs a handle to connect SbfParser to BaseConnection and LaserScanner (to set time)
    SbfParser* const getSbfParser(void) {return mSbfParser;}

private:
    LogFile* mLogFileSbf;
    LogFile* mLogFileCmd;

    // To emit status in regular intervals
    QTimer* mStatusTimer;

    SbfParser* mSbfParser;

    // Here, we remember if we're waiting for a reply to a command on a port (e.g. USB1 => getReceiverCapabilities)
    QMap<QString, QByteArray> mLastCommandToGnssDevice;
    //bool mWaitingForCommandReply; // true when we're waiting for a reply from septentrio board (on usb port).

    // This is a guess, there might be shorter packets!
    constexpr static quint32 mMinimumDataPacketSize = 40;

    unsigned int mDiffCorrDataCounter;
//    QByteArray mLastCommandToDeviceUsb;
    QSerialPort *mSerialPortUsb, *mSerialPortCom;
    bool mGnssDeviceIsConfigured; // so we only feed it rtk data when the device is ready for it.

    // The ports we use to talk to the receiver have a name on the receiver-side, e.g. COM1 or USB2
    // We need to use these names to tell the receiver what communication comes in/out of what ports.
    QString mSerialPortOnDeviceUsb, mSerialPortOnDeviceCom;

    QByteArray mReceiveBufferUsb, mReceiveBufferCom;

    // When a packet of N bytes has been processed at the beginning of a buffer, we could remove the buffer's
    // first N bytes. This is a slow operation which we don't want to do 100 times per second with small N.
    // So instead, we keep indices up to where the buffers have been processed.
    quint32 mDataCursorUsb, mDataCursorCom;
    // If these cursors grow larger than
    constexpr static quint32 mMaximumDataBufferSize = 1024*1024; // 1 MB
    // we remove the first mDataCursorX bytes of mReceiveBufferX and reset mDataCursorX = 0. Basically,
    // this is a poor man's RingBuffer. Compared to always removing single packets after processing, this
    // rather simple change leads to a >100x speedup!

    QList<QByteArray> mCommandQueueUsb;

    // This method finds out how many seconds are left before the TOW (time-of-week)
    // value in the receiver rolls over, potentially screwing up our calculcations.
    quint32 getTimeToTowRollOver();

    // If the given port is waiting for a command reply, this function checks
    // if the reply has arrived, parses it and advances the corresponding dataCursor.
    bool parseCommandReply(const QString& portNameOnDevice, const QByteArray * const receiveBuffer, quint32 * const dataCursorToAdvance);


private slots:
    // Sends @command via USB port
    void slotQueueCommand(QString command);

//    void slotLogProcessedSbfPacket(const qint32 tow, const char *sbfData, quint16 length);

    void slotSerialPortStatusChanged(const QString& status, const QDateTime& time);
    void slotCommunicationSetup();
    quint8 slotFlushCommandQueue();
    void slotDataReadyOnUsb();
    void slotDataReadyOnCom();
    void slotDetermineSerialPortsOnDevice();
    void slotSetPoseFrequency(bool highSpeed = true);

    // Call this method with a valid TOW and it'll sync the system time to this time.
    void slotSetSystemTime(const qint32& tow);

public slots:
    void slotSetDifferentialCorrections(const QByteArray* const differentialCorrections);
    void slotShutDown(); // just calls communicationStop. To be called from the main program's signal handler.

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);
};

#endif // GnssDevice_H
