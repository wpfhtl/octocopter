#include "roverconnection.h"

RoverConnection::RoverConnection(const QString& hostName, const quint16& port, QObject *parent) : QObject(parent)
{
    mHostName = hostName;
    mPort = port;

    mTimerConnectionWatchdog.setInterval(5000);
    mTimerConnectionWatchdog.start();
    connect(&mTimerConnectionWatchdog, SIGNAL(timeout()), SLOT(slotEmitConnectionTimedOut()));

    mIncomingDataBuffer.clear();

    mTcpSocket = new QTcpSocket(this);
    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));
    connect(mTcpSocket, SIGNAL(connected()), SLOT(slotSocketConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()), SLOT(slotSocketDisconnected()));
}

void RoverConnection::slotEmitConnectionTimedOut()
{
    // This method is called only by a timer, which is reset whenever a packet comes in.
    // So, as lsong as we receive packets within intervals smaller than the timer's, we
    // should never emit a broken connection.
    emit connectionStatusRover(false);
}

void RoverConnection::slotSocketDisconnected()
{
    qDebug() << "RoverConnection::slotSocketDisconnected()";

    emit message(
                Warning,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                "Connection closed, retrying...");

    emit connectionStatusRover(false);

    slotConnectToRover();
}

void RoverConnection::slotSocketConnected()
{
    qDebug() << "RoverConnection::slotSocketConnected()";

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                "Connection established.");

    emit connectionStatusRover(true);
}


void RoverConnection::slotReadSocket()
{
//    qDebug() << "RoverConnection::slotReadSocket()";

    mIncomingDataBuffer.append(mTcpSocket->readAll());

    if(mIncomingDataBuffer.size() < 8) return;

//    mIncomingDataBuffer.remove(mIncomingDataBuffer.indexOf(QString("$KPROT").toAscii()), 6);

    QDataStream stream(mIncomingDataBuffer); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

//    qDebug() << "RoverConnection::slotReadSocket(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBuffer.size();

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


void RoverConnection::processPacket(QByteArray data)
{
//    qDebug() << "RoverConnection::processPacket(): processing bytes:" << data.size();

    // Restart the timer, so we don't get a timeout.
    mTimerConnectionWatchdog.start();
    emit connectionStatusRover(true);

    QDataStream stream(data);

    QString packetType;
    stream >> packetType;

    if(packetType == "lidarpoints")
    {
        QVector3D scannerPosition;
        QVector<QVector3D> points;
        stream >> points;
        stream >> scannerPosition;

        emit scanData(points, scannerPosition);
    }
    else if(packetType == "image")
    {
        QString cameraName;
        QSize imageSize;
        Pose cameraPose;
        QByteArray imageData;

        stream >> cameraName;
        stream >> imageSize;
        stream >> cameraPose;
        stream >> imageData;

        emit image(cameraName, imageSize, cameraPose, imageData);
    }
    else if(packetType == "vehiclestatus")
    {
        quint32 missionRunTime;
        float batteryVoltage;
        qint16 barometricHeight;
        qint8 wirelessRssi;

        stream >> missionRunTime;
        stream >> barometricHeight;
        stream >> batteryVoltage;
        stream >> wirelessRssi;

        emit vehicleStatus(missionRunTime, batteryVoltage, barometricHeight, wirelessRssi);
    }
    else if(packetType == "gpsstatus")
    {
        GpsStatusInformation::GpsStatus gpsStatusInformation;

        stream >> gpsStatusInformation;

        emit message(
                    gpsStatusInformation.error == 0 && gpsStatusInformation.gnssMode & 15 == 4 && gpsStatusInformation.integrationMode == 2 && gpsStatusInformation.gnssAge == 0 && gpsStatusInformation.numSatellitesUsed > 5 ? Information : Error,
                    QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                    GpsStatusInformation::getStatusText(gpsStatusInformation));

        emit gpsStatus(gpsStatusInformation);
    }
    else if(packetType == "posechanged")
    {
        Pose p;
        stream >> p;

        emit vehiclePose(p);
    }
    else if(packetType == "currentwaypointshash")
    {
        // The rover just sent us his waypoints-hash
        QString hash;
        stream >> hash;

        emit wayPointsHashFromRover(hash);
    }
    else if(packetType == "waypointreached")
    {
        WayPoint wpt;
        stream >> wpt;

        qDebug() << "process packet: wpt reached:" << QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z());

        emit wayPointReachedByRover(wpt);
    }
    else if(packetType == "waypointinserted")
    {
        // This happens when the rover reached the last waypoint and appended its own landing waypoint (probably just below the last one)
        quint16 index;
        stream >> index;

        WayPoint wpt;
        stream >> wpt;

        emit wayPointInsertedByRover(index, wpt);

        qDebug() << "process packet: wpt appended by rover:" << QString("waypoint appended by rover: %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z());
    }
    else if(packetType == "logmessage")
    {
        quint8 importance;
        QString source, text;

        stream >> source;
        stream >> importance;
        stream >> text;

        emit message((LogImportance)importance, source, text);
    }
    else if(packetType == "controllervalues")
    {
        Pose pose;
        quint8 thrust;
        qint8 pitch, roll, yaw, height;

        stream >> pose;
        stream >> thrust;
        stream >> pitch;
        stream >> roll;
        stream >> yaw;
        stream >> height;

        // Normalize poseYaw between -180 and 180 for better graphing
        float poseYaw = pose.getYawDegrees() <= 180.0 ? pose.getYawDegrees() : pose.getYawDegrees() - 360.0;

        QVector<float> values;
        values << pitch << roll << thrust << yaw;
        values << pose.getPitchDegrees() << pose.getRollDegrees() << poseYaw;

        emit controllerValues(values);
    }
    else
    {
        qDebug() << "RoverConnection::processPacket(): unknown packetType" << packetType;
        emit message(
                    Error,
                    QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                    "Unknown packetType: " + packetType);

        mIncomingDataBuffer.clear();
    }
}

void RoverConnection::slotRoverWayPointsSet(const QList<WayPoint>& wayPoints)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypoints");
    stream << wayPoints;

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Transmitting %1 waypoints to rover").arg(wayPoints.size()));

    slotSendData(data);
}

void RoverConnection::slotRoverWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "RoverConnection::slotRoverWayPointInsert(): telling rover to insert wpt" << wayPoint << "before index" << index;

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinsert");
//    stream << hash;
    stream << index;
    stream << wayPoint;

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Inserting 1 waypoint at index %2").arg(index));

    slotSendData(data);
}

void RoverConnection::slotRoverWayPointDelete(const quint16& index)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    qDebug() << "RoverConnection::slotRoverWayPointDelete(): telling rover to delete wpt" << index;

    stream << QString("waypointdelete");
//    stream << hash;
    stream << index;

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Deleting waypoint at index %1").arg(index));

    slotSendData(data);
}

void RoverConnection::slotSendRtkDataToRover(const QByteArray& rtkData)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("rtkdata");
    stream << rtkData;

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("sending %1 bytes of rtk data to rover").arg(rtkData.size()));

    slotSendData(data);
}

void RoverConnection::slotSendData(const QByteArray &data)
{
//    qDebug() << "RoverConnection::slotSendData():" << data << mTcpSocket->state();
//    qDebug() << "RoverConnection::slotSendData():" << mTcpSocket->errorString();

    QByteArray dataToSend;
    QDataStream streamDataToSend(&dataToSend, QIODevice::WriteOnly);
//    streamDataToSend << QString("$KPROT").toAscii();
    streamDataToSend << (quint32)(data.length() + sizeof(quint32));

    dataToSend.append(data);

    mTcpSocket->write(dataToSend);
}


void RoverConnection::slotConnectToRover()
{
    Q_ASSERT(!mHostName.isEmpty() && mPort != 0 && "RoverConnection::slotConnectToRover(): hostname or port not set.");

    mTcpSocket->abort();
    mTcpSocket->connectToHost(mHostName, mPort);

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                "Trying to connect to rover...");
}

void RoverConnection::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "RoverConnection::slotSocketError():" << socketError;

    emit connectionStatusRover(false);

    emit message(
                Error,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                "Connection failed: " + mTcpSocket->errorString() + ", retrying...");

    QTimer::singleShot(2000, this, SLOT(slotConnectToRover()));
}
