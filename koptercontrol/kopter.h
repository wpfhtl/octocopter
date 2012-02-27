#ifndef KOPTER_H
#define KOPTER_H

#include <QCoreApplication>
#include <QFile>
#include <QTimer>
#include <QDebug>

#include <abstractserial.h>
#include "koptermessage.h"

class Kopter : public QObject
{
    Q_OBJECT

    struct ExternControl
    {
        unsigned char Digital[2];	 //two 1/0-buttons, not used atm
        unsigned char RemoteButtons; // ? something to control the lcd menues?!
        signed char   Nick;
        signed char   Roll;
        signed char   Gier;
        unsigned char Gas;		//gas value is limited (in the fc sw) to the gas *stick* value
        signed char   Height;	//sets the hight for the barometric-hight-sensor
        unsigned char free;		//not used atm
        unsigned char Frame;	//value, which is send back by the fc for confirmation
        unsigned char Config;	//if set to 1 the ExternControl is set active

        ExternControl() {}; // Hopefully, this c'tor will initialize all members to zero.
    };

    struct VersionInfo
    {
        unsigned char SWMajor;
        unsigned char SWMinor;
        unsigned char ProtoMajor;
        unsigned char ProtoMinor;
        unsigned char SWPatch;
        unsigned char HardwareError[5];

        VersionInfo() {};
    };

    struct DebugOut
    {
        unsigned char Status[2];
        qint16 Analog[32];    // Debugwerte

        DebugOut() {};
    };

public:
    Kopter(QString &serialDeviceFile, QObject *parent = 0);
    ~Kopter();

private:
    int maxreplytime;

    QMap<quint8, QString> mAnalogValueLabels;
    struct ExternControl mStructExternControl;

    QMap<QChar,QTime> mPendingReplies;

    // How many milliseconds between two debug outputs?
    int mDesiredDebugDataInterval;

    AbstractSerial *mSerialPortFlightCtrl;
    QByteArray mReceiveBuffer;
    QTime mMissionStartTime;
    QTimer* mTimerPpmChannelPublisher;

private slots:
    void slotSerialPortDataReady();

public slots:
    void slotSetMotion(const quint8& thrust, const qint8& yaw, const qint8& pitch, const qint8& roll, const qint8& height);
    void slotTestMotors(const QList<unsigned char> &speeds);
    void slotReset();
    // The PPM Channels have the values from the human-remote-control
    void slotGetPpmChannelValues();
    void slotGetVersion();
    void slotGetDebugLabels(quint8 index);

    // Subscribe debug info every @interval milliseconds. 0 disables.
    void slotSubscribeDebugValues(int interval = -1);

signals:
    void kopterStatus(const quint32 missionRunTimeMsecs, const qint16& baroheight, const float& voltage);
    void ppmChannelValues(quint8 thrust, qint8 yaw, qint8 pitch, qint8 roll, bool motorSafety, bool externalControl);
    void externControlReplyReceived();

};

#endif // Kopter_H
