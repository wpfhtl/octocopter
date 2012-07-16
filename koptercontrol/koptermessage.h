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
    bool mIsValid; // set immediately after receiving a packet
    Direction mDirection;
    unsigned int mAddress;
    QChar mId;

    // The UNENCODED payload data
    QByteArray mPayload;

    static QByteArray getChecksum(const QByteArray &data);
    static const QByteArray encode(const QByteArray &data);
    static const QByteArray decode(const QByteArray &payload);

public:
    // Creates a KopterMessage to be sent to the kopter
    KopterMessage(unsigned int address, QChar id, QByteArray payload);

    KopterMessage(const KopterMessage& other);

    // Creates a KopterMessage from a receiveBuffer, so this will create an incoming KopterMessage
    KopterMessage(QByteArray *receiveBuffer);

    KopterMessage &operator=(const KopterMessage& other);

    enum Address {Address_FC = 1, Address_NC = 2, Address_MK3MAG = 3};

    void setPayload(const QByteArray &data);

    bool send(QIODevice* port) const;

    quint8 getAddress() const { return mAddress; }

    QChar getId() const { return mId; }

    const QByteArray getPayload() const { return decode(mPayload); }

    bool isValid() const { return mIsValid; }

    QString toString() const;
};

#endif // KOPTERMESSAGE_H
