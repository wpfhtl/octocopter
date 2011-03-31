#include "kopter.h"

Kopter::Kopter(QString &serialDeviceFile, QObject *parent) : QObject(parent)
{
    mSerialPortFlightCtrl = new QextSerialPort(serialDeviceFile, QextSerialPort::EventDriven);
    mSerialPortFlightCtrl->setBaudRate(BAUD57600);
    mSerialPortFlightCtrl->setFlowControl(FLOW_OFF);
    mSerialPortFlightCtrl->setParity(PAR_NONE);
    mSerialPortFlightCtrl->setDataBits(DATA_8);
    mSerialPortFlightCtrl->setStopBits(STOP_1);
    mSerialPortFlightCtrl->setTimeout(5000);

    mSerialPortFlightCtrl->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPortFlightCtrl->isOpen())
    {
        qDebug() << "Kopter::Kopter(): Opening serial port" << serialDeviceFile << "failed:" << mSerialPortFlightCtrl->errorString() << "Exiting.";
        exit(1);
    }

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

void Kopter::slotSetMotion(quint8 thrust, qint8 nick, qint8 roll, qint8 yaw, qint8 height)
{
    if(mPendingReplies.contains('b')) qWarning() << "Still waiting for a 'B', should not send right now!";

    mStructExternControl.Frame = 1;

    mStructExternControl.Config = 1;
    mStructExternControl.Nick = nick;
    mStructExternControl.Roll = roll;
    mStructExternControl.Gas = thrust;
    mStructExternControl.Gier = yaw;
    mStructExternControl.Hight = height;

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
        qDebug() << "Kopter::slotSubscribeDebugValues(): interval" << mDesiredDebugDataInterval << "refreshing subscription in" << timeToNextSubscription;
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
                if(timeOfRequest.isNull()) qWarning() << "Kopter::slotSerialPortDataReady(): got a reply to an unsent message:" << message.toString();
                if(mPendingReplies.contains(message.getId().toLower()))  qWarning() << "Kopter::slotSerialPortDataReady(): there's another pending message of type" << message.getId().toLower();

                if(message.getId() == 'B') maxreplytime = std::max(maxreplytime, timeOfRequest.msecsTo(QTime::currentTime()));

                qDebug() << "Kopter::slotSerialPortDataReady(): received reply to" << message.getId().toLower() << "after ms:" << timeOfRequest.msecsTo(QTime::currentTime()) << "worst" << maxreplytime;

                if(message.getId() == 'A')
                {
                    QByteArray payload = message.getPayload();
                    char labelChar[16];
                    memcpy(labelChar, payload.data()+1, 15);
                    qDebug() << "Kopter::slotSerialPortDataReady(): label" << (quint8)payload[0] << "is" << QString(labelChar).simplified();
                    mAnalogValueLabels.insert((quint8)payload[0], QString(labelChar).simplified());
                }
                else if(message.getId() == 'B')
                {
                    qDebug() << "Kopter::slotSerialPortDataReady(): received confirmation for externalControl frame:" << (quint8)message.getPayload()[0];
                    Q_ASSERT(!mPendingReplies.contains('b'));
                    emit externControlReplyReceived();
                }
                else if(message.getId() == 'D')
                {
                    QByteArray payload = message.getPayload();
                    memcpy(&mStructDebugOut, payload.data(), sizeof(mStructDebugOut));

                    for(int i=0;i<32;i++)
                    {
//                        qDebug() << QTime::currentTime() << "Kopter::slotSerialPortDataReady(): value" << i<< mAnalogValueLabels.value(i) << "is" << mStructDebugOut.Analog[i];

                        switch(i)
                        {
                        // Here you can emit values you're interested in.
                        case 5:
                            qDebug() << "kopter height is" << mStructDebugOut.Analog[i];
                            emit height(mStructDebugOut.Analog[i]);
                            break;
                        case 9:
                            emit voltage((float)(mStructDebugOut.Analog[i])/10);
                            break;
                        }
                    }
                }
                else if(message.getId() == 'T')
                {
                }
                else if(message.getId() == 'V')
                {
                    QByteArray payload = message.getPayload();
                    memcpy(&mStructVersionInfo, payload.data(), sizeof(mStructVersionInfo));
                    qDebug() << "Kopter::slotSerialPortDataReady(): MK protocol version is" << mStructVersionInfo.ProtoMajor << mStructVersionInfo.ProtoMinor;
                    qDebug() << "Kopter::slotSerialPortDataReady(): MK software version is" << mStructVersionInfo.SWMajor << mStructVersionInfo.SWMinor << mStructVersionInfo.SWPatch;

                    if(mStructVersionInfo.ProtoMajor != 11 || mStructVersionInfo.ProtoMinor != 0) qFatal("Kopter::slotSerialPortDataReady(): MK protocol version mismatch, exiting.");
                    if(mStructVersionInfo.SWMajor != 0 || mStructVersionInfo.SWMinor != 82 || mStructVersionInfo.SWPatch != 1) qFatal("Kopter::slotSerialPortDataReady(): MK software version mismatch, exiting.");
                }
            }
            else
            {
//                qWarning() << "Kopter::slotSerialPortDataReady(): got KopterMessage from ignored address:" << message.getAddress();
            }
        }
    }
}
