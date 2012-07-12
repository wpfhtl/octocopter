#include "koptermessage.h"

KopterMessage::KopterMessage(unsigned int address, QChar id, QByteArray payload) : QObject()
{
    mDirection = Outgoing;
    mIsValid = true; // always for outgoing packets

    mAddress = address;
    mId = id;
    mPayload = payload;
}


KopterMessage::KopterMessage(const KopterMessage& other) : QObject()
{
    mDirection = other.mDirection;
    mAddress = other.mAddress;
    mId = other.mId;
    mPayload = other.mPayload;
    mIsValid = other.mIsValid;
}

KopterMessage::KopterMessage(QByteArray* receiveBuffer) : QObject()
{
    // http://www.mikrokopter.de/ucwiki/en/SerialProtocol
    // Decode the packet: Startbyte (#), Addressbyte ('a'+Address), ID-Byte (e.g. 'B'), n-Data-Bytes, Byte CRC1, Byte CRC2, "\r"
    const int position = receiveBuffer->indexOf('\r');
    if(position != -1 && position > 5)
    {
        // Copy the packet from the receivebuffer into our own data-field.
        QByteArray packetData = receiveBuffer->left(position+1);

        // We actually remove the packet from the receive-buffer!
        receiveBuffer->remove(0, position+1);

        // For some reason, messages sometimes start with TWO # instead of just one. Repair this here.
        packetData.remove(0, packetData.lastIndexOf("#"));
        Q_ASSERT(packetData.at(0) == '#');

        mDirection = Incoming;
        mAddress = packetData.at(1) - 'a';
        mId = packetData.at(2);

        // The payload is everything except the first and last three bytes
        mPayload = packetData.right(packetData.size()-3).left(packetData.size()-6);

//        qDebug() << "KopterMessage::KopterMessage(): from stream:" << toString();

        // FIXME: this fails at times. Do not crash, but rather discard the message.

        // This is an incoming message, compare beginning, end, and its CRC (bytes n-2 and n-1) with the CRC we would compute.
        if(
                packetData.size() > 5
                &&
                packetData.at(0) == '#'
                &&
                packetData.right(3).left(2) == getChecksum(packetData.left(packetData.size()-3))
                &&
                packetData.right(1) == "\r"
        )
        {
            mIsValid = true;
        }
        else
        {
            qDebug () << "KopterMessage::KopterMessage(): invalid, data is:" << packetData;
            if(packetData.right(3).left(2) != getChecksum(packetData.left(packetData.size()-3))) qDebug() << "KopterMessage::KopterMessage(): false, checksum error, data:" << packetData;
            if(packetData.right(1) != "\r") qDebug() << "KopterMessage::KopterMessage(): false, no CR at end, data:" << packetData;
            mIsValid = false;
        }

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


KopterMessage& KopterMessage::operator=(const KopterMessage& other)
{
    mDirection = other.mDirection;
    mAddress = other.mAddress;
    mId = other.mId;
    mPayload = other.mPayload;
    mIsValid = other.mIsValid;

    return *this;
}


void KopterMessage::setPayload(const QByteArray &payload)
{
    mPayload = payload;
}

bool KopterMessage::send(QIODevice* port) const
{
    Q_ASSERT(mDirection == Outgoing);

    QByteArray data;

    data.append('#');
    data.append('a' + mAddress);
    data.append(mId);
    data.append(encode(mPayload));
    data.append(getChecksum(data));
//    qDebug() << "KopterMessage::send():" << data;
    data.append('\r');

    return port->write(data) > 0;
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

bool KopterMessage::isValid() const
{
        return mIsValid;
}

quint8 KopterMessage::getAddress() const { return mAddress; }

QChar KopterMessage::getId() const { return mId; }

QByteArray KopterMessage::getPayload() const { return decode(mPayload); }

QString KopterMessage::toString() const
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
