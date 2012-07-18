#include "kopter.h"
#include "motioncommand.h"

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

    mExternalControlActive = false;
    mLastCalibrationSwitchValue = CalibrationSwitchUndefined;
    mStructExternControl.Frame = 0;

    mLastSendTime = QTime::currentTime();

    qDebug() << "Kopter::Kopter(): Opening serial port" << serialDeviceFile << "succeeded, flowControl is" << mSerialPortFlightCtrl->flowControl();

    connect(mSerialPortFlightCtrl, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));

    mTimerPpmChannelPublisher = new QTimer(this);
    mTimerPpmChannelPublisher->start(500);
    connect(mTimerPpmChannelPublisher, SIGNAL(timeout()), SLOT(slotGetPpmChannelValues()));

    slotRequestVersion();

    slotSubscribeDebugValues(2000);

    for(int i=0;i<32;i++) // reply only works up until 28! higher values are never returned!
    {
        slotRequestDebugLabel(i);
    }
}

Kopter::~Kopter()
{
    qDebug() << "Kopter::~Kopter(): closing serial port, max reply times in milliseconds were" << mMaxReplyTimes;
    mSerialPortFlightCtrl->close();
}

void Kopter::slotTestMotors(const QList<unsigned char> &speeds)
{
    if(speeds.size() != 16)
    {
        qDebug() << "Kopter::slotTestMotors(): 16 values required, ignoring request";
        return;
    }

    QByteArray payload;

    for(int i=0;i<16;i++)
        payload[i] = speeds.at(i);

    send(KopterMessage(KopterMessage::Address_FC, 't', payload));
}

void Kopter::slotFlushMessageQueue()
{

    if(!mPendingReply.isNull())
    {
//        qDebug() << "Kopter::slotFlushMessageQueue(): pending reply:" << mPendingReply << "-" << mSendMessageQueue.size() << "messages to send.";
        if(mLastSendTime.msecsTo(QTime::currentTime()) > 100)
        {
            qDebug() << "Kopter::slotFlushMessageQueue(): pending reply" << mPendingReply << "is" << mLastSendTime.msecsTo(QTime::currentTime()) << "ms old, deleting and trying again.";
            mPendingReply = QChar();
            slotFlushMessageQueue();
        }
    }
    else if(mSendMessageQueue.size())
    {
        // There is no outstanding reply for this queued packet, we can send.
        KopterMessage msg = mSendMessageQueue.takeFirst();

        while(mLastSendTime.msecsTo(QTime::currentTime()) < 10)
        {
            qDebug() << "KopterMessage::slotFlushMessageQueue(): waiting for serial port to clear send-queue...";
            usleep(5000);
        }

        //qDebug() << "KopterMessage::slotFlushMessageQueue(): no pending replies - now sending packet" << msg.getId();
        msg.send(mSerialPortFlightCtrl);
        mPendingReply = msg.getId().toUpper();
        mLastSendTime = QTime::currentTime();
    }
}

void Kopter::send(const KopterMessage& message)
{
//    qDebug() << "Kopter::send(): queueing message" << message.getId() << ", then trying to flush...";
    mSendMessageQueue.append(message);
    slotFlushMessageQueue();
}

void Kopter::slotSetMotion(const MotionCommand& mc)
{
    if(!mMissionStartTime.isValid()) mMissionStartTime = QTime::currentTime();

    const MotionCommand motionClamped = mc.clampedToSafeLimits();

    qDebug() << t() << "Kopter::slotSetMotion(): setting clamped motion, frame:" << mStructExternControl.Frame << "thrust:" << motionClamped.thrust << "yaw:" << motionClamped.yaw << "pitch:" << motionClamped.pitch << "roll:" << motionClamped.roll;

    /*
      The kopter has different conventions, at least with default settings (which I intent to keep):

       - Positive pitch makes it pitch forward, nose-down.
         This is opposite to our expectations, because we have the right-handed X axis pointing right

       - Positive roll makes it roll its left side down, which matches our conventions

       - Positive yaw makes it rotate CW, as seen from the top.
         This is opposite to our expectations, because we have the right-handed Y axis pointing upwards

         WARNING: This is weird, as ppmChannels[4] below contradicts this. But as long as we're clear
         to the outside, we should be fine.

      For this reason, we send negative pitch and yaw values.

      For the Extern(al)Control commands to take effect, Poti7 has to be > 128 on the kopter (as con-
      figured in "Stick"). And Poti7 is assigned to channel 7 (as configured in "Kanäle"), which maps
      to switch SW1 on the remote control, which then has to be switched DOWN to activate ExternalControl.

      Even with ExternalControl enabled, the kopter's thrust value is always upper-bound by the remote-
      control's value. This seems to make a lot of sense.

      Under "Verschiedenes/Miscellaneous", max gas is configured to 200, which might also apply to
      ExternalControl. I don't know.
    */

    if(mPendingReply == 'B') qWarning() << "Kopter::slotSetMotion(): Still waiting for a 'B', should not send right now," << mSendMessageQueue.size() << "pending commands in queue";

    mStructExternControl.Config = 1;
    mStructExternControl.Nick = -motionClamped.pitch;
    mStructExternControl.Roll = motionClamped.roll;
    mStructExternControl.Gas = motionClamped.thrust;
    mStructExternControl.Gier = -motionClamped.yaw;
    mStructExternControl.Height = 0; // I don't know what this is for.

    send(KopterMessage(KopterMessage::Address_FC, 'b', QByteArray((const char *)&mStructExternControl, sizeof(mStructExternControl))));

    // The frame is an unsigned char, so it should overflow safely
    mStructExternControl.Frame++;
}

void Kopter::slotReset()
{
    send(KopterMessage(KopterMessage::Address_FC, 'R', QByteArray()));
}

void Kopter::slotRequestDebugLabel(quint8 index)
{
    if(index >= 32)
        qDebug() << "Kopter::slotGetDebugLabel(): received impossible index" << index;

    send(KopterMessage(KopterMessage::Address_FC, 'a', QByteArray((const char*)&index, 1)));
}

void Kopter::slotRequestVersion()
{
    send(KopterMessage(KopterMessage::Address_FC, 'v', QByteArray()));
}

void Kopter::slotGetPpmChannelValues()
{
    send(KopterMessage(KopterMessage::Address_FC, 'p', QByteArray()));
}

void Kopter::slotSubscribeDebugValues(int interval)
{
    // If this method is called directly (and not recursively), remember the
    // desired debug interval.
    if(interval != -1)
    {
        qDebug() << "Kopter::slotSubscribeDebugValues(): setting up subscription to" << interval;
        mDesiredDebugDataInterval = interval;
    }

    if(mDesiredDebugDataInterval > 2550) qWarning() << "Kopter::slotSubscribeDebugValues(): Cannot get debug data every" << mDesiredDebugDataInterval << "ms, subscribing every 2550ms instead.";

    // In the MK, the interval value is multiplied by ten and then used as
    // milliseconds to make up for the small range of a uint8.
    // Using 0 will disable debug output.
    quint8 interval_byte = std::min(255, mDesiredDebugDataInterval/10);
    //qDebug() << "Kopter::slotSubscribeDebugValues(): effective subscription in ms" << ((int)interval_byte)*10;
    send(KopterMessage(KopterMessage::Address_FC, 'd', QByteArray((const char*)&interval_byte)));

    // The subscription only lasts for 4 seconds as defined by ABO_TIMEOUT in the FC source.
    // To make up for this, we call ourselves periodically to re-subscribe.
    if(mDesiredDebugDataInterval > 0)
    {
        const int maximumSubscriptionDuration = 4000;
        const int timeToNextSubscription = maximumSubscriptionDuration - (maximumSubscriptionDuration % mDesiredDebugDataInterval) + mDesiredDebugDataInterval;
        //qDebug() << "Kopter::slotSubscribeDebugValues(): interval" << mDesiredDebugDataInterval << "refreshing subscription in" << timeToNextSubscription;
        QTimer::singleShot(timeToNextSubscription, this, SLOT(slotSubscribeDebugValues()));
    }
}

void Kopter::slotSerialPortDataReady()
{
    mReceiveBuffer.append(mSerialPortFlightCtrl->readAll());

    // Remove crap before the message starts
    mReceiveBuffer.remove(0, std::max(0, mReceiveBuffer.indexOf('#')));

    //qDebug() << "Kopter::slotSerialPortDataReady(): bytes:" << mReceiveBuffer.size() << "data:" << mReceiveBuffer.replace("\r", " ");

    while(mReceiveBuffer.indexOf('\r') != -1)
    {
        const KopterMessage message(&mReceiveBuffer);

        if(!message.isValid())
        {
            qWarning() << "Kopter::slotSerialPortDataReady(): got invalid KopterMessage:" << message.toString();
            continue;
        }

        if(message.getAddress() == KopterMessage::Address_FC)
        {

            // Remove the record of the pending reply
            if(mPendingReply == message.getId())
            {
                qint16 replyTime = mLastSendTime.msecsTo(QTime::currentTime());

                // Ignore the first (=V) packet, as this is the first time when we're actually slower than the kopter
                const qint16 maxReplyTimeForThisPacketType = mMaxReplyTimes.value(mPendingReply, 0);
                if(replyTime > maxReplyTimeForThisPacketType)
                {
                    qDebug() << t() << "Kopter::slotSerialPortDataReady(): max reply time increased from" << maxReplyTimeForThisPacketType << "to" << replyTime << "for packet:" << mPendingReply;
                    mMaxReplyTimes[mPendingReply] = replyTime;
                }
                mPendingReply = QChar();
            }
            else
            {
                // If we receive somethign we didn't ask for, start bitching.
                //  k is mkmag's compass data which cannot be disabled, event though no mkmag is installed
                //  D  is debug out, which we subscribed to. Its just that one subscription gives multiple replies.
                if(message.getId() != 'k' && message.getId() != 'D')
                {
                    qWarning() << "Kopter::slotSerialPortDataReady(): got reply" << message.getId() << "to an unsent request, mPendingReply:" << (mPendingReply.isNull() ? "null" : QString(mPendingReply));
                }
            }

            if(message.getId() == 'A')
            {
                const QByteArray payload = message.getPayload();
                char labelChar[16];
                memcpy(labelChar, payload.data()+1, 15);
                qDebug() << "Kopter::slotSerialPortDataReady(): label" << (quint8)payload[0] << "is" << QString(labelChar).simplified();
                mAnalogValueLabels.insert((quint8)payload[0], QString(labelChar).simplified());
            }
            else if(message.getId() == 'B')
            {
                qDebug() << t() << "Kopter::slotSerialPortDataReady(): received confirmation for externalControl frame:" << (quint8)message.getPayload()[0];
            }
            else if(message.getId() == 'D')
            {
//                qDebug() << "Kopter::slotSerialPortDataReady(): received debug packet!";

                const QByteArray payload = message.getPayload();
                const DebugOut* debugOut = (DebugOut*)payload.data();

                emit kopterStatus(
                            mMissionStartTime.isValid() ? (quint32)mMissionStartTime.msecsTo(QTime::currentTime()) : 0,
                            debugOut->Analog[5],
                            (float)(debugOut->Analog[9])/10.0
                            );
            }
            else if(message.getId() == 'P')
            {
                // Read ppm channels request reply.
                const QByteArray payload = message.getPayload();
//                const qint16* ppmChannels = (qint16*)payload.data();
                const PpmChannels* ppmChannels = (PpmChannels*)payload.data();

                //for(int i=0; i < payload.size()/2; i++) qDebug() << "Kopter::slotSerialPortDataReady(): ppm channel" << i << ":" << ppmChannels[i];

                // These channel values are experienced when read from incoming data. They do NOT match the mikrokopter-tool levels!
                // ppmChannels[1] is Thrust: -127 is min, 14 is max
                // ppmChannels[2] is Roll: 93 is max (left on R/C), -93 is min (right on R/C). Positive rolls positive on the Z axis.
                // ppmChannels[3] is Pitch: 94 is max (up on R/C), -94 is min (down on R/C). Positive pitches negative on the X axis.
                // ppmChannels[4] is Yaw: 96 is max (left on R/C), -90 is min (right on R/C). Positive yaws positive on the Y axis
                // ppmChannels[5] is SW3 / MotorSafety. -122 is disabled (motors can be toggled), 127 is enabled (motor switching blocked)
                // ppmChannels[6] is CTRL7. -122 is disabled (motors can be toggled), 127 is enabled (motor switching blocked)
                // ppmChannels[7] is SW1 / ExternalControl. -122 is disabled, 127 is enabled
                // ppmChannels[8] is SW4PB8 / Calibration. -122 and 127 are the two states it can reach.

                qDebug() << "Kopter::slotSerialPortDataReady(): remote control limits thrust to" << ppmChannels->thrust + 127;

                if(ppmChannels->externalControl > 0 != mExternalControlActive)
                {
                    mExternalControlActive = ppmChannels->externalControl > 0;
                    qDebug() << "Kopter::slotSerialPortDataReady(): externalControlActive:" << mExternalControlActive << "ppm[7]:" << ppmChannels->externalControl;
                    emit computerControlStatusChanged(mExternalControlActive);
                }

                if(ppmChannels->calibration > 0 != (mLastCalibrationSwitchValue == CalibrationSwitchHigh) && mLastCalibrationSwitchValue != CalibrationSwitchUndefined)
                {
                    //qDebug() << "Kopter::slotSerialPortDataReady(): calibration switch toggled from" << mLastCalibrationSwitchValue << "to:" << (ppmChannels->calibration > 0);
                    emit calibrationSwitchToggled();
                }
                mLastCalibrationSwitchValue = ppmChannels->calibration > 0 ? CalibrationSwitchHigh : CalibrationSwitchLow;
            }
            else if(message.getId() == 'T')
            {
                // engine test reply, contains no data
            }
            else if(message.getId() == 'V')
            {
                const QByteArray payload = message.getPayload();
                const VersionInfo* versionInfo = (VersionInfo*)payload.data();
                qDebug() << t() << "Kopter::slotSerialPortDataReady(): MK protocol version is" << versionInfo->ProtoMajor << versionInfo->ProtoMinor;
                qDebug() << t() << "Kopter::slotSerialPortDataReady(): MK software version is" << versionInfo->SWMajor << versionInfo->SWMinor << versionInfo->SWPatch;

                if(versionInfo->ProtoMajor != 11 || versionInfo->ProtoMinor != 0) qFatal("Kopter::slotSerialPortDataReady(): MK protocol version mismatch, exiting.");
                if(versionInfo->SWMajor != 0 || versionInfo->SWMinor != 86 || versionInfo->SWPatch != 3) qFatal("Kopter::slotSerialPortDataReady(): MK software version mismatch, this is untested, exiting.");
            }
            else
            {
                qWarning() << "Kopter::slotSerialPortDataReady(): got KopterMessage with unknown id:" << message.getId();
            }

            // Only flush after we received a message from FC, not from mkmag or others.
            slotFlushMessageQueue();
        }
        else
        {
            //qWarning() << "Kopter::slotSerialPortDataReady(): got KopterMessage with id" <<  message.getId() << "from ignored address:" << message.getAddress();
        }
    }

}
