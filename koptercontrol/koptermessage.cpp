#include "koptermessage.h"

QDateTime KopterMessage::mLastSentTimeStamp = QDateTime::currentDateTime();

KopterMessage::KopterMessage(unsigned int address, QChar id, QByteArray payload) : QObject()
{
    mDirection = Outgoing;
    mAddress = address;
    mId = id;
    mPayload = payload;
}

KopterMessage::KopterMessage(QByteArray* receiveBuffer) : QObject()
{
    // http://www.mikrokopter.de/ucwiki/en/SerialProtocol
    // Decode the packet: Startbyte (#), Addressbyte ('a'+Address), ID-Byte (e.g. 'B'), n-Data-Bytes, Byte CRC1, Byte CRC2, "\r"
    const int position = receiveBuffer->indexOf('\r');
    if(position != -1 && position > 5)
    {
        // Copy the packet from the receivebuffer into our own data-field.
        mData = receiveBuffer->left(position+1);

        // We actually remove the packet from the receive-buffer!
        receiveBuffer->remove(0, position+1);

        // For some reason, messages sometimes start with TWO # instead of just one. Repair this here.
        mData.remove(0, mData.lastIndexOf("#"));
        Q_ASSERT(mData.at(0) == '#');

        mDirection = Incoming;
        mAddress = mData.at(1) - 'a';
        mId = mData.at(2);
        mPayload = mData.right(mData.size()-3).left(mData.size()-6);

//        qDebug() << "KopterMessage::KopterMessage(): from stream:" << toString();

        // FIXME: this fails at times. Do not crash, but rather discard the message.
        if(!isValid())
            qDebug() << "KopterMessage::KopterMessage(): warning, incoming message with address" << mAddress << "id" << mId << "is invalid";
    }
    else
    {
        if(position > 0)
        {
            qDebug() << "KopterMessage::KopterMessage(): from stream: invalid, position of \\r is" << position << "data is" << receiveBuffer->left(position);
        }
        else
        {
            qDebug() << "KopterMessage::KopterMessage(): from stream: invalid, position of \\r is" << position;
        }

        mDirection = Incoming;
        mAddress = -1;
        mId = -1;
    }
}

void KopterMessage::setPayload(const QByteArray &payload)
{
    mPayload = payload;
}

int KopterMessage::send(QIODevice* port, QMap<QChar, QTime>* pendingReplies)
{
    Q_ASSERT(mDirection == Outgoing);

    mData.clear();

    mData.append('#');
    mData.append('a' + mAddress);
    mData.append(mId);

    QString stringPendingReplies;

    foreach(const QChar& reply, pendingReplies->keys())
        stringPendingReplies.append(reply);

//    qDebug() << "KopterMessage::send(): waiting for replies:" << stringPendingReplies;

// #warning    if(pendingReplies->count(mId)) qWarning() << "KopterMessage::send():" << pendingReplies->count(mId) << "messages of type" << mId << "is still underway, should not resend!";

    pendingReplies->insertMulti(mId, QTime::currentTime());

    mData.append(encode(mPayload));

    mData.append(getChecksum(mData));

    qDebug() << "KopterMessage::send():" << mData;

    mData.append('\r');

    // I'm afraid the FC's serial link doesn't support any kind of flow control?!
    // To try, please read http://code.google.com/p/qextserialport/issues/detail?id=90
    // If we write() multiple messages in sequence WITHOUT waiting at least ~3ms in
    // between, data is lost - somwhere. No idea whether the FC can't handle even
    // 57600baud, or whether some FIFO overflows...
    // This is especially noticeable when sending multiple 'a' packets to retrieve
    // debug labels on startup. Thus, we add some latency (and complexity :) here.

    while(mLastSentTimeStamp.msecsTo(QDateTime::currentDateTime()) < 3)
    {
//        qDebug() << "KopterMessage::send(): waiting for serial port to clear send-queue...";
        usleep(10000);
    }

    mLastSentTimeStamp = QDateTime::currentDateTime();

    return port->write(mData);
}

/*AddCRC calculates two byte sized CRC checksums for the encoded frame and
adds them to the end of the data-frame. Additionally it adds an \r escape
sequence to the end of the frame (defined by the Mikrokopter-Protocol) */
QByteArray KopterMessage::getChecksum(const QByteArray &data)
{
    QByteArray crc;
    unsigned int tmpCRC = 0;
    int i;

    for(i=0; i < data.size(); i++)
    {
        tmpCRC += data.at(i);
    }

    tmpCRC %= 4096;

    crc.append('=' + tmpCRC / 64);
    crc.append('=' + tmpCRC % 64);
//    qDebug() << "getCRC bytes are" << crc;
    return crc;
}

QByteArray KopterMessage::encode(const QByteArray &data)
{
    QByteArray encoded;

    unsigned char a,b,c;
    int position = 0;

    while(position < data.size())
    {
        if(position < data.size())
            a = data.at(position++);
        else
            a = 0;

        if(position < data.size())
            b = data.at(position++);
        else
            b = 0;

        if(position < data.size())
            c = data.at(position++);
        else
            c = 0;

        encoded.append('=' + (a >> 2));
        encoded.append('=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4)));
        encoded.append('=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6)));
        encoded.append('=' + ( c & 0x3f));
    }

    return encoded;
}

QByteArray KopterMessage::decode(const QByteArray &data)
{
    QByteArray decoded;
    int position = 0;
    unsigned char a,b,c,d;

    while(position + 3 < data.size())
    {
        a = (unsigned char)data.at(position++) - '=';
        b = (unsigned char)data.at(position++) - '=';
        c = (unsigned char)data.at(position++) - '=';
        d = (unsigned char)data.at(position++) - '=';

        decoded.append((a << 2) | (b >> 4));
        decoded.append(((b & 0x0f) << 4) | (c >> 2));
        decoded.append(((c & 0x03) << 6) | d);
    }

    return decoded;
}

bool KopterMessage::isValid()
{
    if(mDirection == Outgoing)
    {
        return true;
    }
    else
    {
        // This is an incoming message, compare beginning, end, and its CRC (bytes n-2 and n-1) with the CRC we would compute.
        if(
                mData.size() > 5
                &&
                mData.at(0) == '#'
                &&
                mData.right(3).left(2) == getChecksum(mData.left(mData.size()-3))
                &&
                mData.right(1) == "\r"
        )
        {
            return true;
        }
        else
        {
            qDebug () << "KopterMessage::isValid(): data is:" << mData;
            if(mData.right(3).left(2) != getChecksum(mData.left(mData.size()-3))) qDebug() << "KopterMessage::isValid(): false, checksum error, data:" << mData;
            if(mData.right(1) != "\r") qDebug() << "KopterMessage::isValid(): false, no CR at end, data:" << mData;
            return false;
        }
    }
}

quint8 KopterMessage::getAddress() const { return mAddress; }

QChar KopterMessage::getId() const { return mId; }

QByteArray KopterMessage::getPayload() const { return decode(mPayload); }

QString KopterMessage::toString()
{
    QString addressText;
    switch(mAddress)
    {
    case 1:
        addressText = "FC";
        break;
    case 2:
        addressText = "NC";
        break;
    case 3:
        addressText = "MK3MAG";
        break;
    default:
        addressText = "INVALID";
    }

    QByteArray temp;
    temp.append('#').append('a' + mAddress).append(mId.toAscii()).append(encode(mPayload));

    return QString("# ADR %2-%3, ID %4, PAYLOAD %5 (decoded %6)")
            .arg(mAddress)
            .arg(addressText)
            .arg(mId)
            .arg(QString(mPayload))
            .arg(QString(decode(mPayload)));
}
