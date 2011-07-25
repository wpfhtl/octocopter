#include "basestation.h"

#include "flightplannerbasic.h"
#include "flightplannerphysics.h"

BaseStation::BaseStation() : QMainWindow()
{
    qDebug() << "BaseStation::BaseStation()";
    mOctree = new Octree(
            QVector3D(-100, -100, -100), // min
            QVector3D(100, 100, 100),  // max
            1000);

    mProgress = 0;

    mHostNames << "atomboard.dyndns.org" << "localhost" << "192.168.1.1";
    mConnectionDialog = new ConnectionDialog(this);
    slotAskForConnectionHostNames();

    mOctree->setMinimumPointDistance(0.1);
    mOctree->setPointHandler(OpenGlUtilities::drawPoint);

    mIncomingDataBuffer.clear();

    menuBar()->addAction("Connect", this, SLOT(slotConnectToRover()));

    mTcpSocket = new QTcpSocket(this);
    connect(mTcpSocket, SIGNAL(readyRead()), SLOT(slotReadSocket()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(slotSocketError(QAbstractSocket::SocketError)));
    connect(mTcpSocket, SIGNAL(connected()), SLOT(slotSocketConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()), SLOT(slotSocketDisconnected()));

    mControlWidget = new ControlWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);

    mWirelessDevice = new WirelessDevice("wlan0");
    connect(mWirelessDevice, SIGNAL(rssi(qint8)), mControlWidget, SLOT(slotUpdateWirelessRssi(qint8)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    menuBar()->addAction("Save Log", mLogWidget, SLOT(save()));

    mRtkFetcher = new RtkFetcher(mConnectionDialog->getHostNameRtkBase(), 4001, this);
    connect(mRtkFetcher, SIGNAL(rtkData(QByteArray)), SLOT(slotSendRtkDataToRover(QByteArray)));

    mFlightPlanner = new FlightPlannerPhysics(this, &mVehiclePose, mOctree);
    mFlightPlanner->slotSetScanVolume(QVector3D(140, 70, 80), QVector3D(240, 120, 150));
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    connect(mControlWidget, SIGNAL(wayPointInsert(const quint16&, const WayPoint&)), mFlightPlanner, SLOT(slotWayPointInsert(const quint16&, const WayPoint&)));
    connect(mControlWidget, SIGNAL(wayPointDelete(const quint16&)), mFlightPlanner, SLOT(slotWayPointDelete(const quint16&)));
    connect(mControlWidget, SIGNAL(wayPointSwap(quint16,quint16)), mFlightPlanner, SLOT(slotWayPointSwap(quint16,quint16)));

    connect(mFlightPlanner, SIGNAL(wayPointDeleted(quint16)), mControlWidget, SLOT(slotWayPointDeleted(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPointInserted(quint16,WayPoint)), mControlWidget, SLOT(slotWayPointInserted(quint16,WayPoint)));
    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>)), mControlWidget, SLOT(slotSetWayPoints(QList<WayPoint>)));

    connect(mFlightPlanner, SIGNAL(wayPointDeleted(quint16)), SLOT(slotWayPointDeleted(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPointInserted(quint16,WayPoint)), SLOT(slotWayPointInserted(quint16,WayPoint)));

    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>)), SLOT(slotSetWayPoints(QList<WayPoint>)));

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));

    mGlWidget = new GlWidget(this, mOctree, mFlightPlanner);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(update()));
    setCentralWidget(mGlWidget);

    connect(mGlWidget, SIGNAL(visualizeNow()), mFlightPlanner, SLOT(slotVisualize()));
    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWidget, SLOT(updateGL()));

    menuBar()->addAction("Screenshot", mGlWidget, SLOT(slotSaveImage()));

    menuBar()->addAction("ViewFromSide", mGlWidget, SLOT(slotViewFromSide()));
    menuBar()->addAction("ViewFromTop", mGlWidget, SLOT(slotViewFromTop()));

    mPlotWidget = new PlotWidget(this);
    mPlotWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
    addDockWidget(Qt::LeftDockWidgetArea, mPlotWidget);

    mPlotWidget->createCurve("cPitch");
    mPlotWidget->createCurve("cRoll");
    mPlotWidget->createCurve("cThrust");
    mPlotWidget->createCurve("cYaw");

    mPlotWidget->createCurve("pPitch");
    mPlotWidget->createCurve("pRoll");
    mPlotWidget->createCurve("pYaw");

    mLogWidget->log(Information, "BaseStation::BaseStation()", "startup finished, ready.");

    slotConnectToRover();
}

BaseStation::~BaseStation()
{
}

void BaseStation::slotSocketDisconnected()
{
    qDebug() << "BaseStation::slotConnectionEnded()";

    mLogWidget->log(Warning, "BaseStation::slotSocketDisconnected()", "connection closed, retrying...");

    mControlWidget->slotUpdateRoverConnection(false);

    slotConnectToRover();
}

void BaseStation::slotSocketConnected()
{
    qDebug() << "BaseStation::slotSocketConnected()";

    mLogWidget->log(Information, "BaseStation::slotSocketConnected()", "connection established.");

    mControlWidget->slotUpdateRoverConnection(true);
}

void BaseStation::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(CloudExporter::savePly(this, mOctree, fileName))
    {
        mLogWidget->log(Information, "BaseStation::slotExportCloud()", "Successfully wrote cloud to " + fileName);
        QMessageBox::information(this, "Cloud export", "Successfully wrote cloud to\n" + fileName, "OK");
    }
    else
    {
        mLogWidget->log(Error, "BaseStation::slotExportCloud()", "Failed saving cloud to file " + fileName);
        QMessageBox::information(this, "Cloud export", "Failed saving cloud to file\n\n" + fileName, "OK");
    }
}

void BaseStation::addRandomPoint()
{
    QVector3D scannerPosition(0, 0, 0);
    QVector3D point(rand()%199 - 100, rand()%199 - 100, rand()%199 - 100);
    mOctree->insertPoint(
            new LidarPoint(
                    point,
                    scannerPosition-point,
                    (rand()%20+1)
                    )
            );

    mLogWidget->log(Information, "BaseStation::addRandomPoint()", "added random point.");
}

void BaseStation::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
        addRandomPoint();
}

void BaseStation::slotReadSocket()
{
//    qDebug() << "BaseStation::slotReadSocket()";

    mIncomingDataBuffer.append(mTcpSocket->readAll());

    if(mIncomingDataBuffer.size() < 8) return;

//    mIncomingDataBuffer.remove(mIncomingDataBuffer.indexOf(QString("$KPROT").toAscii()), 6);

    QDataStream stream(mIncomingDataBuffer); // byteArray is const!
    quint32 packetLength;
    stream >> packetLength;

//    qDebug() << "BaseStation::slotReadSocket(): packetLength is" << packetLength << "buffersize is" << mIncomingDataBuffer.size();

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


void BaseStation::processPacket(QByteArray data)
{
//    qDebug() << "BaseStation::processPacket(): processing bytes:" << data.size();

    QDataStream stream(data);

    QString packetType;
    stream >> packetType;

    if(packetType == "lidarpoints")
    {
        QVector3D scannerPosition;
        QList<QVector3D> pointList;
//        quint32 numberOfHitsToReceive;
//        stream >> numberOfHitsToReceive;
        stream >> scannerPosition;
        stream >> pointList;

//        mLogWidget->log(QString("received %1 points, inserting...").arg(pointList.size()));

        int i=0;
        foreach(const QVector3D &p, pointList)
        {
//            qDebug() << p;
            mOctree->insertPoint(new LidarPoint(p, (p-scannerPosition).normalized(), (p-scannerPosition).lengthSquared()));
            if(i%10 == 0)
                mFlightPlanner->insertPoint(new LidarPoint(p, (p-scannerPosition).normalized(), (p-scannerPosition).lengthSquared()));
            i++;
        }

        mGlWidget->update();

        mLogWidget->log(Information, "BaseStation::processPacket()", QString("%1 points using %2 MB, %3 nodes, %4 points added.").arg(mOctree->getNumberOfItems()).arg((mOctree->getNumberOfItems()*sizeof(LidarPoint))/1000000.0, 2, 'g').arg(mOctree->getNumberOfNodes()).arg(pointList.size()));

//        qDebug() << "appended" << pointList.size() << "points to octree.";
    }
    else if(packetType == "images")
    {
        QString cameraName;
        QVector3D cameraPosition;
        QQuaternion cameraOrientation;
        quint32 imageSize;
        QByteArray image;

        stream >> cameraName;
        stream >> cameraPosition;
        stream >> cameraOrientation;
        stream >> imageSize;

        image.resize(imageSize);

        qDebug() << "imageSize is" << imageSize << "actually read" << stream.readRawData(image.data(), imageSize) << "ba size" << image.size();

        qDebug() << "camera" << cameraName << "pos" << cameraPosition << "orientation" << cameraOrientation << "image bytearray size is" << image.size();

        if(!mCameraWindows.contains(cameraName))
            mCameraWindows.insert(cameraName, new CameraWindow(this, cameraName));

        CameraWindow* win = mCameraWindows.value(cameraName);

        win->slotSetPixmapData(image);
        win->show();
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

        mControlWidget->slotUpdateMissionRunTime(missionRunTime);
        mControlWidget->slotUpdateBattery(batteryVoltage);
        mControlWidget->slotUpdateBarometricHeight(barometricHeight);

        // It seems impossible to read the RSSI of the basestation's signal if the rover
        // is the access-point. At least hostapd seems to have no feature for querying
        // an STAs RSSI. Thus, we read the RSSI of the rover's signal at the base...
//        mControlWidget->slotUpdateWirelessRssi(wirelessRssi);
    }
    else if(packetType == "gpsstatus")
    {
        quint8 gnssMode, integrationMode, error, numSatellitesTracked, lastPvtAge;
        quint16 info;
        QString status;

        stream >> gnssMode;
        stream >> integrationMode;
        stream >> info;
        stream >> error;
        stream >> numSatellitesTracked;
        stream >> lastPvtAge;
        stream >> status;

        mLogWidget->log(
                    error == 0 ? Information : Error,
                    QString("GpsDevice"),
                    QString("GnssMode %1, IntgrationMode %2, Info %3, Error %4, NumSats %5, PvtAge %6, Status %7")
                    .arg(gnssMode).arg(integrationMode).arg(info).arg(error).arg(numSatellitesTracked).arg(lastPvtAge).arg(status)
                    );

        mControlWidget->slotUpdateGpsStatus(gnssMode, integrationMode, info, error, numSatellitesTracked, lastPvtAge, status);
    }
    else if(packetType == "posechanged")
    {
        stream >> mVehiclePose;

        mControlWidget->slotUpdatePose(mVehiclePose);
    }
    else if(packetType == "currentwaypointshash")
    {
        QString wayPointsHashFromRover;
        stream >> wayPointsHashFromRover;

        if(hash(mFlightPlanner->getWayPoints()) != wayPointsHashFromRover)
        {
            mLogWidget->log(Warning, "BaseStation::processPacket()", QString("waypoints hash from rover does not match our hash, resending list"));
            slotSetWayPoints(mFlightPlanner->getWayPoints());
        }
    }
    else if(packetType == "waypointreached")
    {
        WayPoint wpt;
        stream >> wpt;

        mFlightPlanner->slotWayPointReached(wpt);

        mLogWidget->log(Information, "BaseStation::processPacket()", QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));
    }
    else if(packetType == "logmessage")
    {
        quint8 importance;
        QString source, text;

        stream >> source;
        stream >> importance;
        stream >> text;

        mLogWidget->log((LogImportance)importance, ">" + source, text);
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

        QVector<float> values;
        values << pitch << roll << thrust << yaw;
        values << pose.getPitchDegrees() << pose.getRollDegrees() << pose.getYawDegrees();

        mPlotWidget->slotAppendData(values);
    }
    else
    {
        qDebug() << "BaseStation::processPacket(): unknown packetType" << packetType;
        mLogWidget->log(Error, "BaseStation::processPacket()", "unknown packetType: " + packetType);
        mIncomingDataBuffer.clear();
    }
}

void BaseStation::slotSetWayPoints(const QList<WayPoint>& wayPoints)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypoints");
    stream << wayPoints;

    mLogWidget->log(Information, "BaseStation::slotSetWayPoints()", QString("transmitting %1 waypoints to rover").arg(wayPoints.size()));

    slotSendData(data);
}

void BaseStation::slotWayPointInserted(const quint16& index, const WayPoint& wayPoint)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinsert");
//    stream << hash;
    stream << index;
    stream << wayPoint;

    mLogWidget->log(Information, "BaseStation::slotWayPointInsert()", QString("inserting 1 waypoint at index %2").arg(index));

    slotSendData(data);
}

void BaseStation::slotWayPointDeleted(const quint16& index)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointdelete");
//    stream << hash;
    stream << index;

    mLogWidget->log(Information, "BaseStation::slotWayPointDelete()", QString("deleting waypoint at index %1").arg(index));

    slotSendData(data);
}

void BaseStation::slotSendRtkDataToRover(const QByteArray& rtkData)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("rtkdata");
    stream << rtkData;

    mLogWidget->log(Information, "BaseStation::slotSendRtkDataToRover()", QString("sending %1 bytes of rtk data to rover").arg(rtkData.size()));

    slotSendData(data);

    mControlWidget->slotUpdateRtkStatus(true);
}

void BaseStation::slotSendData(const QByteArray &data)
{
//    qDebug() << "BaseStation::slotSendData():" << data << mTcpSocket->state();
//    qDebug() << "BaseStation::slotSendData():" << mTcpSocket->errorString();

    QByteArray dataToSend;
    QDataStream streamDataToSend(&dataToSend, QIODevice::WriteOnly);
//    streamDataToSend << QString("$KPROT").toAscii();
    streamDataToSend << (quint32)(data.length() + sizeof(quint32));

    dataToSend.append(data);

    mTcpSocket->write(dataToSend);
}

void BaseStation::slotAskForConnectionHostNames()
{
    mConnectionDialog->show();
    mConnectionDialog->exec();
}

void BaseStation::slotConnectToRover()
{
    while(mConnectionDialog->getHostNameRover().isEmpty())
        slotAskForConnectionHostNames();

    mTcpSocket->abort();
    mTcpSocket->connectToHost(mConnectionDialog->getHostNameRover(), 12345);
}

void BaseStation::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "BaseStation::slotSocketError():" << socketError;

    mControlWidget->slotUpdateRoverConnection(false);

    if(socketError == QAbstractSocket::ConnectionRefusedError)
        QTimer::singleShot(2000, this, SLOT(slotConnectToRover()));
}

const WayPoint BaseStation::getNextWayPoint(void) const
{
    QList<WayPoint> waypoints = mFlightPlanner->getWayPoints();

    if(waypoints.size())
        return waypoints.first();
    else
        return WayPoint();
}

/*void BaseStation::slotFlightPlannerProcessing(const QString& text, const quint8& progress)
{
    if(!mProgress)
    {
        mProgress = new QProgressDialog(text, "Abort", 0, 100, this);
//        mProgress->setWindowModality(Qt::WindowModal);
    }

    mProgress->setValue(progress);

    if(progress == 100)
    {
        mProgress->hide();
        mProgress->deleteLater();
        mProgress = 0;
    }
}*/
