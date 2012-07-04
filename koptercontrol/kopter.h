#ifndef KOPTER_H
#define KOPTER_H

#include <QCoreApplication>
#include <QFile>
#include <QTimer>
#include <QDebug>

#include <abstractserial.h>
#include "koptermessage.h"
#include "common.h"

class MotionCommand;

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

        ExternControl() {} // Hopefully, this c'tor will initialize all members to zero.
    };

    /*
        These channel values are experienced when read from incoming data. They do NOT match the mikrokopter-tool levels!
        Of course, this mapping depends on the configuration of remotecontrol and octocopter! So don't change that!

        ppmChannels[1] is Thrust: -127 is min, 14 is max -> use offset +127 to convert to [0;255]
        ppmChannels[2] is Roll: 93 is max (left on R/C), -93 is min (right on R/C). Positive rolls positive on the Z axis.
        ppmChannels[3] is Pitch: 94 is max (up on R/C), -94 is min (down on R/C). Positive pitches negative on the X axis.
        ppmChannels[4] is Yaw: 96 is max (left on R/C), -90 is min (right on R/C). Positive yaws positive on the Y axis
        ppmChannels[5] is SW3 / MotorSafety. -122 is disabled (motors can be toggled), 127 is enabled (motor switching blocked)
        ppmChannels[6] is CTRL7. -122 is disabled (motors can be toggled), 127 is enabled (motor switching blocked)
        ppmChannels[7] is SW1 / ExternalControl. -122 is disabled, 127 is enabled
        ppmChannels[8] is SW4PB8 / Calibration. -122 and 127 are the two states it can reach.
    */
    struct PpmChannels
    {
        qint16 unused;
        qint16 thrust;
        qint16 roll;
        qint16 pitch;
        qint16 yaw;
        qint16 motorSafety;     // SW3
        qint16 poti;            // CTRL7
        qint16 externalControl; // SW1
        qint16 calibration;     // SW4/PB8

        PpmChannels() {}
    };

    struct VersionInfo
    {
        unsigned char SWMajor;
        unsigned char SWMinor;
        unsigned char ProtoMajor;
        unsigned char ProtoMinor;
        unsigned char SWPatch;
        unsigned char HardwareError[5];

        VersionInfo() {}
    };

    struct DebugOut
    {
        unsigned char Status[2];
        qint16 Analog[32];    // Debugwerte

        DebugOut() {}
    };

public:
    Kopter(QString &serialDeviceFile, QObject *parent = 0);
    ~Kopter();

private:
    qint32 mMaxReplyTime;

    QMap<quint8, QString> mAnalogValueLabels;
    ExternControl mStructExternControl;

    QMap<QChar,QTime> mPendingReplies;

    // How many milliseconds between two debug outputs?
    int mDesiredDebugDataInterval;

    AbstractSerial *mSerialPortFlightCtrl;
    QByteArray mReceiveBuffer;
    QTime mMissionStartTime;
    QTimer* mTimerPpmChannelPublisher;

    bool mExternalControlActive;
    qint8 mLastCalibrationSwitchValue;

private slots:
    void slotSerialPortDataReady();

public slots:
    void slotSetMotion(const MotionCommand& mc);
    void slotTestMotors(const QList<unsigned char> &speeds);
    void slotReset();
    // The PPM Channels have the values from the human-remote-control
    void slotGetPpmChannelValues();
    void slotGetVersion();
    void slotGetDebugLabel(quint8 index);

    // Subscribe debug info every @interval milliseconds. 0 disables.
    void slotSubscribeDebugValues(int interval = -1);

signals:
    void kopterStatus(const quint32 missionRunTimeMsecs, const qint16& baroheight, const float& voltage);
    //void ppmChannelValues(PpmChannels);
    // Mikrokopter calls control from non-remote-control sources "ExternalControl". This reflects SW1 on the RC.
    void computerControlStatusChanged(bool);
    void calibrationSwitchToggled();
//    void externControlReplyReceived();

};

#endif // Kopter_H
