#ifndef KOPTER_H
#define KOPTER_H

#include <QCoreApplication>
#include <QFile>
#include <QTimer>
#include <QDebug>

#include "qextserialport/src/qextserialport.h"
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
        signed char   Hight;	//sets the hight for the barometric-hight-sensor
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
    struct VersionInfo mStructVersionInfo;
    struct DebugOut mStructDebugOut;

    QMap<QChar,QTime> mPendingReplies;

    // How many milliseconds between two debug outputs?
    int mDesiredDebugDataInterval;

    QextSerialPort *mSerialPortFlightCtrl;
    QByteArray mReceiveBuffer;

    void initialize();

private slots:
    void slotSerialPortDataReady();

public slots:
    void slotSetMotion(quint8 thrust, qint8 nick, qint8 roll, qint8 yaw, qint8 height);
    void slotTestMotors(const QList<unsigned char> &speeds);
    void slotReset();
    void slotGetVersion();
    void slotGetDebugLabels(quint8 index);

    // Subscribe debug info every @interval milliseconds. 0 disables.
    void slotSubscribeDebugValues(int interval = -1);

signals:
    void voltage(float);
    void height(qint16);
    void externControlReplyReceived();

};

#endif // Kopter_H
