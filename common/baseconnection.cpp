#include "baseconnection.h"

// for getRssi()
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/wireless.h>

BaseConnection::BaseConnection(const QString& interface, QObject* parent) :
    QObject(parent),
    mMutex(QMutex::NonRecursive)
{
    QMutexLocker locker(&mMutex);

    qDebug() << "BaseConnection::BaseConnection()";

    mInterface = interface;
    mTcpSocket = 0;

    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));
    mTcpServer->listen(QHostAddress::Any, 12345);

    mIncomingDataBuffer.clear();
    mOutgoingDataBuffer.clear();
}

BaseConnection::~BaseConnection()
{
    mTcpServer->deleteLater();
}

void BaseConnection::slotConnectionEnded()
{
    QMutexLocker locker(&mMutex);
    mTcpSocket = 0;
}

void BaseConnection::slotNewConnection()
{
    QMutexLocker locker(&mMutex);
    mTcpSocket = mTcpServer->nextPendingConnection();
//    connect(mTcpSocket, SIGNAL(disconnected()), mTcpSocket, SLOT(deleteLater()));
    connect(mTcpSocket, SIGNAL(disconnected()), SLOT(slotConnectionEnded()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotConnectionEnded()));

    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));

//    mSocketBuffers.insert(mTcpSocket, new QByteArray);

    qDebug() << "BaseConnection::slotNewConnection(): incoming connection accepted";
}

void BaseConnection::slotReadSocket(bool lockMutex)
{
    if(lockMutex) QMutexLocker locker(&mMutex);

    mIncomingDataBuffer.append(mTcpSocket->readAll());
//    qDebug() << "BaseConnection::slotReadSocket(): incoming-buffer now has" << mIncomingDataBuffer.size() << "bytes";

    if(mIncomingDataBuffer.size() < 8) return;

    QDataStream stream(mIncomingDataBuffer); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

//    qDebug() << "BaseConnection::slotReadSocket(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBuffer.size();

    if(mIncomingDataBuffer.size() >= packetLength)
    {
        // strip the length from the beginning
        mIncomingDataBuffer.remove(0, sizeof(quint32));

        // pass the packet
        processPacket(mIncomingDataBuffer.left(packetLength - sizeof(quint32)));

        // now remove the packet from our incoming buffer
        mIncomingDataBuffer.remove(0, packetLength - sizeof(quint32));

        // see whether there's another packet lurking around in the array...
        slotReadSocket(false);
    }
}

void BaseConnection::processPacket(QByteArray packet)
{
//    QMutexLocker locker(&mMutex);

//    qDebug() << "BaseConnection::processPacket(): packetLength is" << packet.size();

    QDataStream stream(packet); // byteArray is const!
    QString command;
    stream >> command;

    if(command == "waypointinsert")
    {
        QString hash;
        quint32 index;
        QVector3D wayPoint;
        stream >> hash;
        stream >> index;
        stream >> wayPoint;
//        mFlightController->slotWayPointInsert(hash, index, wayPoint);
        qDebug() << "BaseConnection::processPacket(): inserting waypoint from base to index" << index << wayPoint;
    }
    else if(command == "waypointdelete")
    {
        QString hash;
        quint32 index;
        stream >> hash;
        stream >> index;
//        mFlightController->slotWayPointDelete(hash, index);
        qDebug() << "BaseConnection::processPacket(): deleting waypoint" << index << "from base.";
    }
    else if(command == "getstatus")
    {
        QByteArray data;
        QDataStream stream(&data, QIODevice::WriteOnly);

        stream << QString("status");

//        stream << mFlightController->getPosition();
//        stream << mFlightController->getOrientation();

//        stream << mVehicle->getLinearVelocity();

//        qDebug() << "BaseConnection::processPacket(): getstatus done, sending reply.";

        slotSendData(data, false);
    }
    else
    {
        qDebug() << "UNKNOWN COMMAND" << command;
        Q_ASSERT(false);
    }
}


void BaseConnection::slotSocketError(QAbstractSocket::SocketError socketError)
{
    QMutexLocker locker(&mMutex);
    qDebug() << "BaseConnection::slotSocketImagesError():" << socketError;
}

void BaseConnection::slotSendData(const QByteArray &data, bool lockMutex)
{
    if(lockMutex) QMutexLocker locker(&mMutex);

    // Prepend the length of the datagram
    QByteArray datagramLengthArray;
    QDataStream streamLength(&datagramLengthArray, QIODevice::WriteOnly);
    streamLength << (quint32)(data.length() + sizeof(quint32));
    mOutgoingDataBuffer.append(datagramLengthArray);

//    qDebug() << "BaseConnection::slotQueueWrite(): appending bytes to buffer:" << data.size();
    mOutgoingDataBuffer.append(data);
    QMetaObject::invokeMethod(this, "slotFlushWriteQueue", Qt::QueuedConnection);
}

void BaseConnection::slotFlushWriteQueue()
{
    QMutexLocker locker(&mMutex);

    if(mTcpSocket)
    {
//        qDebug() << "BaseConnection::slotFlushWriteQueue(): flushing buffer, sending bytes to base:" << mOutgoingDataBuffer.size();
        mTcpSocket->write(mOutgoingDataBuffer);
        mOutgoingDataBuffer.clear();
    }
//    else
//        qDebug() << "BaseConnection::slotFlushWriteQueue(): mTcpSocket is 0, skipping.";
}

int BaseConnection::getRssi()
{
    /* Any old socket will do, and a datagram socket is pretty cheap */
    int sockfd;
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        slotNewLogMessage(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Error, "Could not create simple datagram socket");

    struct iwreq iwr;
    struct iw_statistics iws;

    memset(&iwr, 0, sizeof(iwr));
    iwr.u.data.pointer = (char*)&iws;
    iwr.u.data.length  = sizeof(iws);
    iwr.u.data.flags = 1;
    strncpy(iwr.ifr_name, mInterface.toAscii(), IFNAMSIZ);

    if(ioctl(sockfd, SIOCGIWSTATS, &iwr) < 0)
    {
            slotNewLogMessage(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Error, "Could not execute SIOCGIWSTATS IOCTL");
            return -1;
    }

    int signalQuality = iws.qual.level;

    if(signalQuality == 0 || iws.qual.updated & IW_QUAL_INVALID || iws.qual.updated & IW_QUAL_ALL_INVALID)
    {
        slotNewLogMessage(QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), Error, "link quality reading invalid");
        return -1;
    }

    if(iws.qual.updated & IW_QUAL_DBM)
            signalQuality -= 0x100;

    return signalQuality;
}






// called when the rover has changed the waypoints list, will be sent to base
void BaseConnection::slotNewWayPointsFromRover(const QVector<WayPoint>& wayPoints)
{
    qDebug() << "sending new waypoints to base:" << wayPoints;
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypoints");
    stream << wayPoints;
    slotSendData(data, false);
}

// called by rover when it has reached a waypoint, notifies basestation
void BaseConnection::slotWayPointReached(const WayPoint& wpt)
{
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointreached");
    stream << wpt;

    slotSendData(data, false);
}

// called by rover to send updated pose to basestation (called frequently)
void BaseConnection::slotPoseChanged(const Pose& pose, const quint32 receiverTime)
{
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("posechanged");
    stream << pose;
    stream << receiverTime;

    slotSendData(data, false);
}

// called by rover to send lidarpoints to the basestation
void BaseConnection::slotNewLidarPoints(const QVector<LidarPoint>& points)
{
    qDebug() << "sending" << points.size() << "new lidarpoints to base";
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("lidarpoints");
    stream << points;
    slotSendData(data, false);
}

// called by rover to send new vehicle status to basestation
void BaseConnection::slotNewVehicleStatus(
    const float& batteryVoltage,
    const float& barometricHeight
    )
{


    QMutexLocker locker(&mMutex);

    const float wirelessRssi = getRssi();

    slotNewLogMessage(
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                Log,
                QString("sending vehicle status to base: voltage: %1, baro-height: %2, rssi %3.").arg(batteryVoltage).arg(barometricHeight).arg(wirelessRssi)
                );

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("vehiclestatus");
    stream << batteryVoltage;
    stream << barometricHeight;
    stream << wirelessRssi;
    slotSendData(data, false);
}

// called by rover to send new gps status to basestation
void BaseConnection::slotNewGpsStatus(
    const quint8& mode,
    const quint8& info,
    const quint8& error,
    const quint8& numSatellitesTracked,
    const quint8& lastPvtAge,
    const QString& status
    )
{
    qDebug() << "sending new gps status to base:" << mode << info << error << numSatellitesTracked << status;
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("gpsstatus");
    stream << mode;
    stream << info;
    stream << error;
    stream << numSatellitesTracked;
    stream << lastPvtAge;
    stream << status;
    slotSendData(data, false);
}

// called by rover to send new log message to basestation
void BaseConnection::slotNewLogMessage(const QString& source, const BaseConnection::Importance& importance, const QString& text)
{
    QMutexLocker locker(&mMutex);

    qDebug() << "NewLogMsg:" << source << importance << text;

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("logmessage");
    stream << source;
    stream << importance;
    stream << text;
    slotSendData(data, false);
}

