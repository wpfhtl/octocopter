#ifndef KOPTER_H
#define KOPTER_H

#include <QCoreApplication>
#include <QFile>
#include <QTimer>
#include <QDebug>

#include <abstractserial.h>
#include "koptermessage.h"
#include "flightstateswitch.h"
#include "motioncommand.h"
#include "vehiclestatus.h"
#include "common.h"

#include <unistd.h> // usleep()

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
        ppmChannels[6] is CTRL7 / poti. Unused.
        ppmChannels[7] is SW6/7 / ExternalControl, 3-State-Switch. Set to non-linear in remote-control, so that only the first
                          state is below 127 (ExternControl disabled), while state 2 and 3 are above, enabling ExternalControl.
                          Values in KopterTool: 0 (towards user)/154(center)/254(away from user), in code we read -127, X and 127.
        ppmChannels[8] is SW4PB8 / LiftOff. -122 and 127 are the two states it can reach.
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
        qint16 externalControl; // SW6/7, three-state
        qint16 pushButton;     // SW4/PB8

        PpmChannels() {}

        QString toString() const
        {
            return QString("PpmChannels::toString(): unused %1, thrust %2, roll %3, pitch %4, yaw %5, motorSafety %6, poti %7, externalControl %8, pushButton %9")
                    .arg(unused)
                    .arg(thrust)
                    .arg(roll)
                    .arg(pitch)
                    .arg(yaw)
                    .arg(motorSafety)
                    .arg(poti)
                    .arg(externalControl)
                    .arg(pushButton);
        }
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

    enum PushButtonValue
    {
        PushButtonValueUndefined,
        PushButtonValueHigh,
        PushButtonValueLow
    };

public:
    Kopter(QString &serialDeviceFile, QObject *parent = 0);
    ~Kopter();

private:

    // I'm afraid the FC's serial link doesn't support any kind of flow control?!
    // To try, please read http://code.google.com/p/qextserialport/issues/detail?id=90
    // If we write() multiple messages in sequence WITHOUT waiting at least ~3ms in
    // between, data is lost - somwhere. No idea whether the FC can't handle even
    // 57600baud, or whether some FIFO overflows...
    // This is especially noticeable when sending multiple 'a' packets to retrieve
    // debug labels on startup. Thus, we add some latency (and complexity :) here to
    // avoid sending too fast.
    QTime mLastSendTime;

    QMap<QChar,qint16> mMaxReplyTimes; // in milliseconds, per packet-type

    // Here, we keep all the messages to-be-sent to the kopter...
    QList<KopterMessage> mSendMessageQueue;

    // ... which are NOT sent as long as we're still waiting for a reply (within a specified time).
    QChar mPendingReply;

    QMap<quint8, QString> mAnalogValueLabels;
    ExternControl mStructExternControl;

    // How many milliseconds between two debug outputs?
    int mDesiredDebugDataInterval;

    AbstractSerial *mSerialPortFlightCtrl;
    QByteArray mReceiveBuffer;
    QTime mMissionStartTime;
    QTimer* mTimerPpmChannelPublisher;

    VehicleStatus mVehicleStatus;
    FlightStateSwitch mLastFlightStateSwitch;
    PushButtonValue mLastPushButtonValue;
    qint8 mLastFlightSpeed; // maps to CTRL7, used to set ApproachWaypoint flightspeed in meters per second

    void send(const KopterMessage& message);

private slots:
    void slotSerialPortDataReady();
    void slotFlushMessageQueue();

public slots:
    void slotSetMotion(const MotionCommand *const mc);
    void slotTestMotors(const QList<unsigned char> &speeds);
    void slotReset();
    // The PPM Channels have the values from the human-remote-control
    void slotGetPpmChannelValues();
    void slotRequestVersion();
    void slotRequestDebugLabel(quint8 index);

    // Subscribe debug info every @interval milliseconds. 0 disables.
    void slotSubscribeDebugValues(int interval = -1);

signals:
    void vehicleStatus(const VehicleStatus* const vs);
    // Mikrokopter calls control from non-remote-control sources "ExternalControl". This reflects SW1 on the RC.
    void flightStateSwitchValueChanged(const FlightStateSwitch* const fss);
    void pushButtonToggled();

    void flightSpeedChanged(const float speed);

};

#endif // Kopter_H
