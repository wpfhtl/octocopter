#ifndef KOPTERMESSAGE_H
#define KOPTERMESSAGE_H

#include <QObject>
#include <QDebug>
#include <QChar>
#include <QByteArray>
#include <QDateTime>
#include <QIODevice>

class KopterMessage : public QObject
{
    Q_OBJECT
    enum Direction {Incoming, Outgoing};

private:
    static QDateTime mLastSentTimeStamp;
    Direction mDirection;
    unsigned int mAddress;
    QChar mId;

    // The UNENCODED payload data
    QByteArray mPayload;

    // The whole packet will be assembled here for outgoing packets
    // All data fields will be reconstructed from mData for incoming packets
    QByteArray mData;

    static QByteArray getChecksum(const QByteArray &data);
    static QByteArray encode(const QByteArray &data);
    static QByteArray decode(const QByteArray &data);

public:
    // Creates a KopterMessage to be sent to the kopter
    KopterMessage(unsigned int address, QChar id, QByteArray payload);

    // Creates a KopterMessage from a receiveBuffer, so this will create an incoming KopterMessage
    KopterMessage(QByteArray *receiveBuffer);

    enum Address {Address_FC = 1, Address_NC = 2, Address_MK3MAG = 3};

    void setPayload(const QByteArray &data);

    // This method sends this message over the given @port and records an expected reply into a QMap.
    // For example, a 't' message (motortest) yields a 'T' reply, a 'b' message gives a 'B' reply.
    // This way, we can decide whether it makes any sense to send a packet of a type which is
    // still being processed by the FC.
    int send(QIODevice* port, QMap<QChar, QTime>* pendingReplies);

    quint8 getAddress() const;
    QChar getId() const;
    QByteArray getPayload() const;

    bool isValid() const;

    QString toString() const;
};

#endif // KOPTERMESSAGE_H
