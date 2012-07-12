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

    // The whole packet will be assembled here for outgoing packets
    // All data fields will be reconstructed from mData for incoming packets
//    QByteArray mData;

    static QByteArray getChecksum(const QByteArray &data);
    static QByteArray encode(const QByteArray &data);
    static QByteArray decode(const QByteArray &data);

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

    quint8 getAddress() const;
    QChar getId() const;
    QByteArray getPayload() const;

    bool isValid() const;

    QString toString() const;
};

#endif // KOPTERMESSAGE_H
