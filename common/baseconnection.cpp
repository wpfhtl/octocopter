#include "baseconnection.h"

BaseConnection::BaseConnection(const QString& interface) :
    QObject(),
    mMutex(QMutex::NonRecursive)
{
    QMutexLocker locker(&mMutex);

    qDebug() << "BaseConnection::BaseConnection()";

//    mWirelessDevice = new WirelessDevice(interface);
//    mWirelessDevice->getRssi();

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
    mTcpSocket->disconnect();
    mTcpSocket = 0;
    mTcpServer->listen(QHostAddress::Any, 12345);
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

//    qDebug() << "BaseConnection::slotNewConnection(): incoming connection from" << mTcpSocket->peerAddress() << "port" << mTcpSocket->peerPort() << "accepted";
    
    // FIXME: mTcpServer->close() because we cannot handle a second connection, listen() in connectionEnded.
    mTcpServer->close();
}

void BaseConnection::slotReadSocket(bool lockMutex)
{
    if(lockMutex) QMutexLocker locker(&mMutex);

    Q_ASSERT(mTcpSocket);

    mIncomingDataBuffer.append(mTcpSocket->readAll());
//    qDebug() << "BaseConnection::slotReadSocket(): incoming-buffer now has" << mIncomingDataBuffer.size() << "bytes";

    if(mIncomingDataBuffer.size() < 8) return;

//    mIncomingDataBuffer.remove(mIncomingDataBuffer.indexOf(QString("$KPROT").toAscii()), 6);

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
        quint16 index;
        WayPoint wayPoint;
        stream >> index;
        stream >> wayPoint;
        qDebug() << "BaseConnection::processPacket(): inserting waypoint from base to index" << index;
        emit wayPointInsert(index, wayPoint);
        qDebug() << "BaseConnection::processPacket(): inserting waypoint from base to index" << index << "Done.";
    }
    else if(command == "waypointdelete")
    {
        quint16 index;
        stream >> index;
        qDebug() << "BaseConnection::processPacket(): base tells me to delete wpt" << index << "will now emit wptDelete";
        emit wayPointDelete(index);
        qDebug() << "BaseConnection::processPacket(): base tells me to delete wpt" << index << "will now emit wptDelete. Done.";
    }
    else if(command == "waypoints")
    {
        QList<WayPoint> wayPointList;
        stream >> wayPointList;
        qDebug() << "BaseConnection::processPacket(): received" << wayPointList.size() << "waypoints from base.";
        emit wayPoints(wayPointList);
        qDebug() << "BaseConnection::processPacket(): received" << wayPointList.size() << "waypoints from base. Done.";
    }
//    else if(command == "getstatus")
//    {
////        qDebug() << "BaseConnection::processPacket(): getstatus requested, emitting signal.";

//        emit statusRequested();
//    }
    else if(command == "enablescanning")
    {
        bool enable = false;
        stream >> enable;
        emit enableScanning(enable);
    }
    else if(command == "rtkdata")
    {
        QByteArray rtkData;
        stream >> rtkData;
        emit rtkDataReady(rtkData);
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
    qDebug() << "BaseConnection::slotSocketError():" << socketError;
}

void BaseConnection::slotSendData(const QByteArray &data, bool lockMutex)
{
    if(lockMutex) QMutexLocker locker(&mMutex);

    if(!mTcpSocket)
    {
//        qDebug() << "BaseConnection::slotSendData(): no client connected, throwing data away...";
        return;
    }

    // Prepend the length of the datagram
    QByteArray datagramLengthArray;
    QDataStream streamLength(&datagramLengthArray, QIODevice::WriteOnly);
    streamLength << (quint32)(data.length() + sizeof(quint32));
//    mOutgoingDataBuffer.append(QString("$KPROT").toAscii());
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

/*qint8 BaseConnection::getRssi()
{
    if((mSockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        slotNewLogMessage(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Could not create simple datagram socket");
        return -1;
    }

    struct iwreq iwr;
    struct iw_statistics iws;

    memset(&iwr, 0, sizeof(iwr));
    iwr.u.data.pointer = (char*)&iws;
    iwr.u.data.length  = sizeof(iws);
    iwr.u.data.flags = 1;
    strncpy(iwr.ifr_name, mInterface.toAscii(), IFNAMSIZ);

    if(ioctl(mSockfd, SIOCGIWSTATS, &iwr) < 0)
    {
            slotNewLogMessage(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Could not execute SIOCGIWSTATS IOCTL");
            return 100;
    }

    int signalQuality = iws.qual.level;

    if(signalQuality == 0 || iws.qual.updated & IW_QUAL_QUAL_INVALID || iws.qual.updated & IW_QUAL_ALL_INVALID)
    {
        slotNewLogMessage(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "link quality reading invalid");
        return -1;
    }

    if(iws.qual.updated & IW_QUAL_DBM)
            signalQuality -= 0x100;

    return signalQuality;
}*/





// called when the rover has changed the waypoints list, will be sent to base
// Disabled: Why should the rover ever send waypoints?
// Update: Because it can cretae its own ladning-waypoint, the base will want to know!
void BaseConnection::slotRoverWayPointInserted(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "sending new waypoints to base:" << wayPoint;
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinserted");
    stream << (quint16)index;
    stream << wayPoint;
    slotSendData(data, false);
}

void BaseConnection::slotFlightControllerWayPointsChanged(const QList<WayPoint>& wayPoints)
{
    // We're here because the base sent some changes to the rover's waypoint-list, and now
    // we get a chance to send a hash of the rover's list back to the base, so they can compare.
    qDebug() << "BaseConnection::slotFlightControllerWayPointsChanged(): number of waypoints" << wayPoints.size();

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("currentwaypointshash");
    stream << hash(wayPoints);

    slotSendData(data, false);
}

// called by rover when it has reached a waypoint, notifies basestation
void BaseConnection::slotWayPointReached(const WayPoint& wpt)
{
    qDebug() << "BaseConnection::slotWayPointReached(): reached" << wpt;

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointreached");
    stream << wpt;

    slotSendData(data, false);
}

// called by rover to send updated pose to basestation (called frequently)
void BaseConnection::slotPoseChanged(const Pose& pose)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("posechanged");
    stream << pose;

    slotSendData(data, false);
}

// called by rover to send lidarpoints to the basestation
void BaseConnection::slotNewLidarPoints(const QVector3D& scanPosition, const QVector<QVector3D>& points)
{
//    qDebug() << "sending" << points.size() << "new lidarpoints to base";
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("lidarpoints");
    stream << scanPosition;
    stream << points;
    slotSendData(data, false);
}

// called by rover to send new vehicle status to basestation
void BaseConnection::slotNewVehicleStatus(
    const quint32& missionRunTime,
    const qint16& barometricHeight,
    const float& batteryVoltage
    )
{
    const qint8 wirelessRssi = -1;//mWirelessDevice->getRssi();

//    slotNewLogMessage(
//                Information,
//                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
//                QString("sending vehicle status to base: voltage: %1, baro-height: %2, rssi %3.").arg(batteryVoltage).arg(barometricHeight).arg(wirelessRssi)
//                );

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("vehiclestatus");
    stream << (quint32)missionRunTime;
    stream << (qint16)barometricHeight;
    stream << (float)batteryVoltage;
    stream << (qint8)wirelessRssi;
    slotSendData(data, false);
}

// called by rover to send new gps status to basestation
void BaseConnection::slotNewGpsStatus(
    const quint8& gnssMode,
    const quint8& integrationMode,
    const quint16& info,
    const quint8& error,
    const quint8& numSatellitesTracked,
    const quint8& lastPvtAge,
    const quint8& meanCorrAge,
    const QString& status
    )
{
/*    qDebug() << t() << "BaseConnection::slotNewGpsStatus(): -> base:"
             << "gnssMode" << gnssMode
             << "integrationMode" << integrationMode
             << "info" << info
             << "error" << error
             << "numsats" << numSatellitesTracked
             << "lastPvtAge" << lastPvtAge
             << "meanCorrAge" << meanCorrAge
             << "status" << status;
*/
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("gpsstatus");
    stream << gnssMode;
    stream << integrationMode;
    stream << info;
    stream << error;
    stream << numSatellitesTracked;
    stream << lastPvtAge;
    stream << meanCorrAge;
    stream << status;
    slotSendData(data, false);
}

void BaseConnection::slotNewControllerDebugValues(const Pose& pose, const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("controllervalues");
    stream << pose;
    stream << thrust;
    stream << pitch;
    stream << roll;
    stream << yaw;
    stream << height;

    slotSendData(data, false);
}

// called by rover to send new image to basestation
void BaseConnection::slotNewCameraImage(const QString& name, const QSize& imageSize, const QVector3D& position, const QQuaternion& orientation, const QByteArray* compressedImage)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

//qDebug() << QDateTime::currentDateTime().toString("hh:mm:ss:zzz") << "now sending image bytes: count:" << compressedImage->size();

    stream << QString("image");
    stream << name;
    stream << imageSize;
    stream << position;
    stream << orientation;
    stream << *compressedImage;
    slotSendData(data, false);

//qDebug() << QDateTime::currentDateTime().toString("hh:mm:ss:zzz") << "image sent, complete data size was" << data.size();
}

// called by rover to send new log message to basestation
void BaseConnection::slotNewLogMessage(const LogImportance& importance, const QString& source, const QString& text)
{
    qDebug() << "NewLogMsg:" << source << importance << text;

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("logmessage");
    stream << source;
    stream << (quint8)importance;
    stream << text;
    slotSendData(data, false);
}

