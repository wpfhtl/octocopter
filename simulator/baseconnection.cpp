#include "baseconnection.h"

BaseConnection::BaseConnection(Simulator* simulator) :
    QObject(),
    mMutex(QMutex::NonRecursive)
{
    QMutexLocker locker(&mMutex);

    qDebug() << "BaseConnection::BaseConnection()";
    mSimulator = simulator;
    mFlightController = 0;
    mVehicle = 0;
    mTcpSocket = 0;

    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));
    mTcpServer->listen(QHostAddress::Any, 12345);

    //connect(mFlightController, SIGNAL(wayPointReached(QVector3D)), SLOT(slotWayPointReached(QVector3D)));

    mIncomingDataBuffer.clear();
    mOutgoingDataBuffer.clear();
}

BaseConnection::~BaseConnection()
{
    mTcpServer->deleteLater();
}

void BaseConnection::setVehicle(Vehicle* vehicle)
{
    mVehicle = vehicle;
    mFlightController = vehicle->mFlightController;
    connect(mFlightController, SIGNAL(wayPointReached(QVector3D)), SLOT(slotWayPointReached(QVector3D)));
    connect(mFlightController, SIGNAL(currentWayPoints(QList<QVector3D>)), SLOT(slotCurrentWayPointsChanged(QList<QVector3D>)));
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
        mFlightController->slotWayPointInsert(hash, index, wayPoint);
        qDebug() << "BaseConnection::processPacket(): inserting waypoint from base to index" << index << wayPoint;
    }
    else if(command == "waypointdelete")
    {
        QString hash;
        quint32 index;
        stream >> hash;
        stream >> index;
        mFlightController->slotWayPointDelete(hash, index);
        qDebug() << "BaseConnection::processPacket(): deleting waypoint" << index << "from base.";
    }
    else if(command == "getstatus")
    {
        QByteArray data;
        QDataStream stream(&data, QIODevice::WriteOnly);

        stream << QString("status");

        stream << mFlightController->getPosition();
        stream << mFlightController->getOrientation();

        stream << mVehicle->getLinearVelocity();

//        qDebug() << "BaseConnection::processPacket(): getstatus done, sending reply.";

        slotSendData(data, false);
    }
    else
    {
        qDebug() << "UNKNOWN COMMAND" << command;
        Q_ASSERT(false);
    }
}

void BaseConnection::slotWayPointReached(QVector3D wpt)
{
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointreached");
    stream << wpt;

    slotSendData(data, false);
}


void BaseConnection::slotCurrentWayPointsChanged(QList<QVector3D> wayPoints)
{
    qDebug() << "sending new waypoints to base:" << wayPoints;
    QMutexLocker locker(&mMutex);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypoints");
    stream << wayPoints;
    slotSendData(data, false);
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
