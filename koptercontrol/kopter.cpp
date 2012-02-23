#include "kopter.h"

Kopter::Kopter(QString &serialDeviceFile, QObject *parent) : QObject(parent)
{
    mSerialPortFlightCtrl = new AbstractSerial();
    mSerialPortFlightCtrl->setDeviceName(serialDeviceFile);
    if(!mSerialPortFlightCtrl->open(AbstractSerial::ReadWrite)) qFatal("Kopter::Kopter(): Opening serial port %s failed, exiting.", qPrintable(serialDeviceFile));
    mSerialPortFlightCtrl->setBaudRate(AbstractSerial::BaudRate57600);
    mSerialPortFlightCtrl->setDataBits(AbstractSerial::DataBits8);
    mSerialPortFlightCtrl->setParity(AbstractSerial::ParityNone);
    mSerialPortFlightCtrl->setStopBits(AbstractSerial::StopBits1);
    mSerialPortFlightCtrl->setFlowControl(AbstractSerial::FlowControlOff);

    qDebug() << "Kopter::Kopter(): Opening serial port" << serialDeviceFile << "succeeded, flowControl is" << mSerialPortFlightCtrl->flowControl();

    connect(mSerialPortFlightCtrl, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));

    initialize();
}

Kopter::~Kopter()
{
    qDebug() << "Kopter::~Kopter(): closing serial port";
    mSerialPortFlightCtrl->close();
}

void Kopter::initialize()
{
    slotGetVersion();

    for(int i=0;i<32;i++)
    {
        slotGetDebugLabels(i);
    }

    // The client decides about the debugvalue frequency, not ourselves.
//    slotSubscribeDebugValues(2000);
}

void Kopter::slotTestMotors(const QList<unsigned char> &speeds)
{
    Q_ASSERT(speeds.size() == 16);

    QByteArray payload;
//    payload.resize(16);

    for(int i=0;i<16;i++)
    {
        payload[i] = speeds.at(i);
    }

    KopterMessage message(1, 't', payload);
    message.send(mSerialPortFlightCtrl, &mPendingReplies);
}

void Kopter::slotSetMotion(const quint8& thrust, const qint8& yaw, const qint8& pitch, const qint8& roll, const qint8& height)
{
    if(!mMissionStartTime.isValid()) mMissionStartTime = QTime::currentTime();

    qDebug() << "Kopter::slotSetMotion(): setting motion" << thrust << yaw << pitch << roll << height;

    /*
      The kopter has different conventions, at least with default settings (which I intent to keep):
       - Positive pitch makes it pitch forward, nose-down.
         This is opposite to our expectations, because we have the right-handed X axis pointing right

       - Positive roll makes it roll its left side down, which matches our conventions

       - Positive yaw makes it rotate CW, as seen from the top.
         This is opposite to our expectations, because we have the right-handed Y axis pointing upwards

      For this reason, we send negative pitch and yaw values.

      For the Extern(al)Control commands to take effect, Poti7 has to be > 128 on the kopter (as con-
      figured in "Stick"). And Poti7 is assigned to channel 7 (as configured in "Kanäle"), which maps
      to switch SW1 on the remote control, which then has to be switched DOWN to active ExternalControl.

      Even with ExternalControl enabled, the kopter's gas value is always upper-bound by the remote-
      control's value. This seems to make a lot of sense.

      Under "Verschiedenes/Miscellaneous", max gas is configured to 200, which might also apply to
      ExternalControl. I don't know.
      */

    if(mPendingReplies.contains('b')) qWarning() << "Still waiting for a 'B', should not send right now!";

    mStructExternControl.Frame = 1;

    mStructExternControl.Config = 1;
    mStructExternControl.Nick = -pitch;
    mStructExternControl.Roll = roll;
    mStructExternControl.Gas = thrust;
    mStructExternControl.Gier = -yaw;
    mStructExternControl.Height = height;

    KopterMessage message(1, 'b', QByteArray((const char *)&mStructExternControl, sizeof(mStructExternControl)));
    message.send(mSerialPortFlightCtrl, &mPendingReplies);
}

void Kopter::slotReset()
{
    KopterMessage message(1, 'R', QByteArray());
    message.send(mSerialPortFlightCtrl, &mPendingReplies);
}

void Kopter::slotGetDebugLabels(quint8 index)
{
    Q_ASSERT(index < 32);

    KopterMessage message(1, 'a', QByteArray((const char*)&index, 1));
    message.send(mSerialPortFlightCtrl, &mPendingReplies);
}

void Kopter::slotGetVersion()
{
    KopterMessage message(1, 'v', QByteArray());
    message.send(mSerialPortFlightCtrl, &mPendingReplies);
}

void Kopter::slotSubscribeDebugValues(int interval)
{
    Q_ASSERT(interval < 2551);

    // If this method is called directly (and not recursively), remember the
    // desired debug interval.
    if(interval != -1)
    {
        qDebug() << "Kopter::slotSubscribeDebugValues(): setting up subscription to" << interval;
        mDesiredDebugDataInterval = interval;
    }

    if(mDesiredDebugDataInterval > 2550) qWarning() << "Kopter::slotSubscribeDebugValues(): Cannot get debug every" << mDesiredDebugDataInterval << "subscribing every 2550ms instead.";

    // In the MK, the interval value is multiplied by ten and then used as
    // milliseconds to make up for the small range of a uint8.
    // Using 0 will disable debug output.
    quint8 interval_byte = std::min(255, mDesiredDebugDataInterval/10);
//    qDebug() << "Kopter::slotSubscribeDebugValues(): effective subscription in ms" << ((int)interval_byte)*10;
    KopterMessage message(1, 'd', QByteArray((const char*)&interval_byte));
    message.send(mSerialPortFlightCtrl, &mPendingReplies);

    // The subscription only lasts for 4 seconds as defined by ABO_TIMEOUT
    // in the FC source. To make up for this, we call ourselves periodically
    // to re-subscribe.
    if(mDesiredDebugDataInterval > 0)
    {
        const int maximumSubscriptionDuration = 4000;
        const int timeToNextSubscription = maximumSubscriptionDuration - (maximumSubscriptionDuration % mDesiredDebugDataInterval) + mDesiredDebugDataInterval;
//        qDebug() << "Kopter::slotSubscribeDebugValues(): interval" << mDesiredDebugDataInterval << "refreshing subscription in" << timeToNextSubscription;
        QTimer::singleShot(timeToNextSubscription, this, SLOT(slotSubscribeDebugValues()));
    }
}

void Kopter::slotSerialPortDataReady()
{
//    usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBuffer.append(mSerialPortFlightCtrl->readAll());

    while(!mReceiveBuffer.startsWith('#')) mReceiveBuffer.remove(0, 1);

//    qDebug() << "Kopter::slotSerialPortDataReady(): bytes:" << mReceiveBuffer.size() << "data:" << mReceiveBuffer.replace("\r", " ");

    while(mReceiveBuffer.indexOf('\r') != -1)
    {
        KopterMessage message(&mReceiveBuffer);

        if(!message.isValid())
        {
            qWarning() << "Kopter::slotSerialPortDataReady(): got invalid KopterMessage:" << message.toString();
        }
        else
        {
            if(message.getAddress() == KopterMessage::Address_FC)
            {
                // Remove the record of the pending reply
                QTime timeOfRequest = mPendingReplies.take(message.getId().toLower());
//                if(timeOfRequest.isNull()) qWarning() << "Kopter::slotSerialPortDataReady(): got a reply to an unsent message:" << message.toString();
//                if(mPendingReplies.contains(message.getId().toLower()))  qWarning() << "Kopter::slotSerialPortDataReady(): there's another pending message of type" << message.getId().toLower();

                if(message.getId() == 'B') maxreplytime = std::max(maxreplytime, timeOfRequest.msecsTo(QTime::currentTime()));

//                qDebug() << "Kopter::slotSerialPortDataReady(): received reply to" << message.getId().toLower() << "after ms:" << timeOfRequest.msecsTo(QTime::currentTime()) << "worst" << maxreplytime;

                if(message.getId() == 'A')
                {
                    QByteArray payload = message.getPayload();
                    char labelChar[16];
                    memcpy(labelChar, payload.data()+1, 15);
//                    qDebug() << "Kopter::slotSerialPortDataReady(): label" << (quint8)payload[0] << "is" << QString(labelChar).simplified();
                    mAnalogValueLabels.insert((quint8)payload[0], QString(labelChar).simplified());
                }
                else if(message.getId() == 'B')
                {
//                    qDebug() << "Kopter::slotSerialPortDataReady(): received confirmation for externalControl frame:" << (quint8)message.getPayload()[0];
                    // why here? Q_ASSERT(!mPendingReplies.contains('b'));
                    emit externControlReplyReceived();
                }
                else if(message.getId() == 'D')
                {
                    QByteArray payload = message.getPayload();
                    const DebugOut* debugOut = (DebugOut*)payload.data();

                    emit kopterStatus(
                                mMissionStartTime.isValid() ? (quint32)mMissionStartTime.msecsTo(QTime::currentTime()) : 0,
                                debugOut->Analog[5],
                                (float)(debugOut->Analog[9])/10.0
                                );

                    /*
                    for(int i=0;i<32;i++)
                    {
                        qDebug() << QTime::currentTime() << "Kopter::slotSerialPortDataReady(): value" << i<< mAnalogValueLabels.value(i) << "is" << debugOut->Analog[i];

                        switch(i)
                        {
                        // Here you can emit values you're interested in.
                        case 5:
                            qDebug() << "kopter height is" << debugOut->Analog[i];
                            emit height(debugOut->Analog[i]);
                            break;
                        case 9:
                            emit voltage((float)(debugOut->Analog[i])/10);
                            break;
                        }
                    }
                    */
                }
                else if(message.getId() == 'T')
                {
                }
                else if(message.getId() == 'V')
                {
                    QByteArray payload = message.getPayload();
                    const VersionInfo* versionInfo = (VersionInfo*)payload.data();
//                    memcpy(&mStructVersionInfo, payload.data(), sizeof(mStructVersionInfo));
                    qDebug() << "Kopter::slotSerialPortDataReady(): MK protocol version is" << versionInfo->ProtoMajor << versionInfo->ProtoMinor;
                    qDebug() << "Kopter::slotSerialPortDataReady(): MK software version is" << versionInfo->SWMajor << versionInfo->SWMinor << versionInfo->SWPatch;

                    if(versionInfo->ProtoMajor != 11 || versionInfo->ProtoMinor != 0) qFatal("Kopter::slotSerialPortDataReady(): MK protocol version mismatch, exiting.");
                    if(versionInfo->SWMajor != 0 || versionInfo->SWMinor != 86 || versionInfo->SWPatch != 3) qFatal("Kopter::slotSerialPortDataReady(): MK software version mismatch, this is untested, exiting.");
                }
            }
            else
            {
//                qWarning() << "Kopter::slotSerialPortDataReady(): got KopterMessage from ignored address:" << message.getAddress();
            }
        }
    }
}
