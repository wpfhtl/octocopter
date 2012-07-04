#ifndef GPSDEVICE_H
#define GPSDEVICE_H

#include <QCoreApplication>
#include <QFile>
#include <QDebug>
#include <QTimer>

#include <sys/time.h> // for syncing time
#include <common.h>
#include "pose.h"
#include "sbfparser.h"
#include <math.h>
#include <errno.h>

#include <abstractserial.h>


class GpsDevice : public QObject
{
    Q_OBJECT

public:
    GpsDevice(const QString &serialDeviceUsb, const QString &serialDeviceCom, QString logFilePrefix, QObject *parent = 0);
    ~GpsDevice();

    // Our parent (koptercontrol) needs a handle to conect SbfParser to BaseConnection and LaserScanner (to set time)
    SbfParser* getSbfParser(void) {return mSbfParser;}

private:
    QFile* mLogFileSbf;
    QFile* mLogFileCmd;

    // To emit status in regular intervals
    QTimer* mStatusTimer;

    SbfParser* mSbfParser;

    bool mWaitingForCommandReply; // true when we're waiting for a reply from septentrio board (on usb port).
    unsigned int mRtkDataCounter;
    QByteArray mLastCommandToDeviceUsb;
    AbstractSerial *mSerialPortUsb, *mSerialPortCom;
    bool mDeviceIsInitialized; // so we only feed it rtk data when the device is ready for it.

    // The ports we use to talk to the receiver have a name on the receiver-side, e.g. COM1 or USB2
    // We need to use these names to tell the receiver what communication comes in/out of what ports.
    QString mSerialPortOnDeviceUsb, mSerialPortOnDeviceCom;



    QByteArray mReceiveBufferUsb, mReceiveBufferCom;
    QList<QByteArray> mCommandQueueUsb;


    // Sets the internal pose according to the given GPS readings
//    void setPose(const qint32& lon, const qint32& lat, const qint32& alt, const quint16& heading, const qint16& pitch, const qint16& roll, const qint32& tow);

    // This method finds out how many seconds are left before the TOW (time-of-week)
    // value in the receiver rolls over, potentially screwing up our calculcations.
    quint32 getTimeToTowRollOver();


private slots:
    // Sends @command via USB port
    void slotQueueCommand(QString command);

    void slotLogProcessedSbfPacket(const QByteArray&, const qint32&);

    void slotSerialPortStatusChanged(const QString& status, const QDateTime& time);
    void slotCommunicationSetup();
    quint8 slotFlushCommandQueue();
    void slotDataReadyOnUsb();
    void slotDataReadyOnCom();
    void slotDetermineSerialPortsOnDevice();
    void slotSetPoseFrequency(bool highSpeed);

    // Call this method with a valid TOW and it'll sync the system time to this time.
    void slotSetSystemTime(const qint32& tow);

public slots:
    void slotSetRtkData(const QByteArray &data);
    void slotShutDown(); // just calls communicationStop. To be called from the main program's signal handler.

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);
};

#endif // GPSDEVICE_H
