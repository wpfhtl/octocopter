#include "baseconnection.h"

BaseConnection::BaseConnection(Simulator* simulator) : QObject()
{
    QMutexLocker locker(mMutex);

    qDebug() << "BaseConnection::BaseConnection()";
    mSimulator = simulator;
    mFlightController = 0;
    mVehicle = 0;
    mTcpSocket = 0;

    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), SLOT(slotNewConnection()));
    mTcpServer->listen(QHostAddress::Any, 12345);

    connect(mFlightController, SIGNAL(wayPointReached(QVector3D)), SLOT(slotWayPointReached(QVector3D)));

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
}

void BaseConnection::slotConnectionEnded()
{
    QMutexLocker locker(mMutex);
    mTcpSocket = 0;
}

void BaseConnection::slotNewConnection()
{
    QMutexLocker locker(mMutex);
    mTcpSocket = mTcpServer->nextPendingConnection();
//    connect(mTcpSocket, SIGNAL(disconnected()), mTcpSocket, SLOT(deleteLater()));
    connect(mTcpSocket, SIGNAL(disconnected()), SLOT(slotConnectionEnded()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotConnectionEnded()));

    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));

//    mSocketBuffers.insert(mTcpSocket, new QByteArray);

    qDebug() << "BaseConnection::slotNewConnection(): incoming connection accepted";
}

void BaseConnection::slotReadSocket()
{
    QMutexLocker locker(mMutex);

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
        slotReadSocket();
    }
}

void BaseConnection::processPacket(QByteArray packet)
{
    QMutexLocker locker(mMutex);

//    qDebug() << "BaseConnection::processPacket(): packetLength is" << packet.size();

    QDataStream stream(packet); // byteArray is const!
    QString command;
    stream >> command;

    if(command == "setwaypoint")
    {
        QVector3D wayPoint;
        stream >> wayPoint;
        mFlightController->slotSetNextWayPoint(wayPoint);
        qDebug() << "BaseConnection::processPacket(): setting waypoint from base.";
    }
    else if(command == "getstatus")
    {
        QByteArray data;
        QDataStream stream(&data, QIODevice::WriteOnly);

        stream << QString("status");

        stream << mFlightController->getPosition();
        stream << mFlightController->getOrientation();

        stream << mVehicle->getLinearVelocity();
        stream << mFlightController->getWayPoints();

//        qDebug() << "BaseConnection::processPacket(): getstatus done, sending reply.";

        slotSendData(data);
    }
}

void BaseConnection::slotWayPointReached(QVector3D wpt)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointreached");
    stream << wpt;

    slotSendData(data);
}

void BaseConnection::slotSocketError(QAbstractSocket::SocketError socketError)
{
    QMutexLocker locker(mMutex);
    qDebug() << "BaseConnection::slotSocketImagesError():" << socketError;
}

void BaseConnection::slotSendData(const QByteArray &data)
{
    QMutexLocker locker(mMutex);

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
    QMutexLocker locker(mMutex);

    if(mTcpSocket)
    {
//        qDebug() << "BaseConnection::slotFlushWriteQueue(): flushing buffer, sending bytes to base:" << mOutgoingDataBuffer.size();
        mTcpSocket->write(mOutgoingDataBuffer);
        mOutgoingDataBuffer.clear();
    }
//    else
//        qDebug() << "BaseConnection::slotFlushWriteQueue(): mTcpSocket is 0, skipping.";
}
