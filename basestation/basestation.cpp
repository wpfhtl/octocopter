#include "basestation.h"

#include "flightplannerbasic.h"
#include "flightplannerphysics.h"

BaseStation::BaseStation() : QMainWindow()
{
    qDebug() << "BaseStation::BaseStation()";

    // We need a small font.
    QFont widgetFont = QApplication::font();
    widgetFont.setPointSize(7);
    QApplication::setFont(widgetFont);

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



    mTimerStats = new QTimer(this);
    mTimerStats->setInterval(1000);
    mTimerStats->start();

    //mDateTimeProgramStart = QDateTime::currentDateTime();

    mStatsFile = new QFile(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz").prepend("stats-").append(".txt"));
    if(!mStatsFile->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::critical(this, "File Error", QString("Couldn't open stats-file %1 for writing!").arg(mStatsFile->fileName()));
    }
    QTextStream out(mStatsFile);
    out << "# FlightTime;ItemsInOctreeFine;ItemsInOctreeCoarse\n";
    out.flush();
    connect(mTimerStats, SIGNAL(timeout()), SLOT(slotWriteStats()));

    mControlWidget = new ControlWidget(this);
    connect(this, SIGNAL(vehiclePoseChanged(Pose)), mControlWidget, SLOT(slotUpdatePose(Pose)));
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);

    mWirelessDevice = new WirelessDevice("wlan0");
    connect(mWirelessDevice, SIGNAL(rssi(qint8)), mControlWidget, SLOT(slotUpdateWirelessRssi(qint8)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    menuBar()->addAction("Save Log", mLogWidget, SLOT(save()));

    mRtkFetcher = new RtkFetcher(mConnectionDialog->getHostNameRtkBase(), 4001, this);
    connect(mRtkFetcher, SIGNAL(rtkData(QByteArray)), SLOT(slotSendRtkDataToRover(QByteArray)));

    mFlightPlanner = new FlightPlannerPhysics(this, mOctree);
    mFlightPlanner->slotSetScanVolume(QVector3D(140, 70, 80), QVector3D(240, 120, 150));
    connect(this, SIGNAL(vehiclePoseChanged(Pose)), mFlightPlanner, SLOT(slotVehiclePoseChanged(Pose)));
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    connect(mControlWidget, SIGNAL(wayPointInsert(const quint16&, const WayPoint&)), mFlightPlanner, SLOT(slotWayPointInsert(const quint16&, const WayPoint&)));
    connect(mControlWidget, SIGNAL(wayPointDelete(const quint16&)), mFlightPlanner, SLOT(slotWayPointDelete(const quint16&)));
    connect(mControlWidget, SIGNAL(wayPointSwap(quint16,quint16)), mFlightPlanner, SLOT(slotWayPointSwap(quint16,quint16)));

    connect(mFlightPlanner, SIGNAL(wayPointDeleted(quint16)), mControlWidget, SLOT(slotWayPointDeleted(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>)), mControlWidget, SLOT(slotSetWayPoints(QList<WayPoint>)));
    connect(mFlightPlanner, SIGNAL(wayPointInserted(quint16,WayPoint)), mControlWidget, SLOT(slotWayPointInserted(quint16,WayPoint)));

    // When FlightPlanner wants us to send waypoint updates to the rover...
    connect(mFlightPlanner, SIGNAL(wayPointInsertOnRover(quint16,WayPoint)), SLOT(slotRoverWayPointInsert(quint16,WayPoint)));
    connect(mFlightPlanner, SIGNAL(wayPointDeleteOnRover(quint16)), SLOT(slotRoverWayPointDelete(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPointsSetOnRover(QList<WayPoint>)), SLOT(slotRoverWayPointsSet(QList<WayPoint>)));

    // Just for adding a line to the log file for marking a new waypoint generation iteration
    connect(mFlightPlanner, SIGNAL(wayPointsSetOnRover(QList<WayPoint>)), SLOT(slotAddLogFileMarkForPaper(QList<WayPoint>)));

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    menuBar()->addAction("Load Cloud", this, SLOT(slotImportCloud()));

    mGlWidget = new GlWidget(this, mOctree, mFlightPlanner);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(update()));
    connect(mGlWidget, SIGNAL(mouseClickedAtWorldPos(Qt::MouseButton, QVector3D)), mControlWidget, SLOT(slotSetWayPointCoordinateFields(Qt::MouseButton, QVector3D)));
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
    mStatsFile->close();
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

    if(PlyManager::savePly(this, mOctree, fileName))
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

void BaseStation::slotImportCloud()
{
    const QString fileName = QFileDialog::getLoadFileName(this, "Load cloud from", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(PlyManager::savePly(this, mOctree, fileName))
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

        // We only run/stats/logs for efficiency (paper) when scanning is in progress
        mDateTimeLastLidarInput = QDateTime::currentDateTime();

        mGlWidget->update();

        mLogWidget->log(Information, "BaseStation::processPacket()", QString("%1 points using %2 MB, %3 nodes, %4 points added.").arg(mOctree->getNumberOfItems()).arg((mOctree->getNumberOfItems()*sizeof(LidarPoint))/1000000.0, 2, 'g').arg(mOctree->getNumberOfNodes()).arg(pointList.size()));

//        qDebug() << "appended" << pointList.size() << "points to octree.";
    }
    else if(packetType == "image")
    {
        QString cameraName;
        QSize imageSize;
        QVector3D cameraPosition;
        QQuaternion cameraOrientation;
        QByteArray imageData;

        stream >> cameraName;
        stream >> imageSize;
        stream >> cameraPosition;
        stream >> cameraOrientation;
        stream >> imageData;

//        qDebug() << QDateTime::currentDateTime().toString("hh:mm:ss:zzz") << "camera" << cameraName << "pos" << cameraPosition << "orientation" << cameraOrientation << "image bytearray size is" << imageData.size();

        CameraWidget *widget;

        if(!mCameraWidgets.contains(cameraName))
        {
            widget = new CameraWidget(this, cameraName);
            mCameraWidgets.insert(cameraName, widget);
            addDockWidget(Qt::RightDockWidgetArea, widget);
        }
        else
        {
            widget = mCameraWidgets.value(cameraName);
        }

        widget->slotSetPixmapData(imageData);
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
        quint8 gnssMode, integrationMode, error, numSatellitesTracked, lastPvtAge, meanCorrAge;
        quint16 info;
        QString status;

        stream >> gnssMode;
        stream >> integrationMode;
        stream >> info;
        stream >> error;
        stream >> numSatellitesTracked;
        stream >> lastPvtAge;
        stream >> meanCorrAge;
        stream >> status;

        mLogWidget->log(
                    error == 0 ? Information : Error,
                    QString("GpsDevice"),
                    QString("GnssMode %1, IntgrationMode %2, Info %3, Error %4, NumSats %5, PvtAge %6, MeanCorrAge %7, Status %8")
                    .arg(gnssMode).arg(integrationMode).arg(info).arg(error).arg(numSatellitesTracked).arg(lastPvtAge).arg(meanCorrAge).arg(status)
                    );

        mControlWidget->slotUpdateGpsStatus(gnssMode, integrationMode, info, error, numSatellitesTracked, lastPvtAge, meanCorrAge, status);
    }
    else if(packetType == "posechanged")
    {
        Pose vehiclePose;
        stream >> vehiclePose;

        emit vehiclePoseChanged(vehiclePose);
    }
    else if(packetType == "currentwaypointshash")
    {
        QString wayPointsHashFromRover;
        stream >> wayPointsHashFromRover;

        if(hash(mFlightPlanner->getWayPoints()) != wayPointsHashFromRover)
        {
            mLogWidget->log(Warning, "BaseStation::processPacket()", QString("waypoints hash from rover does not match our hash, resending list"));
            slotRoverWayPointsSet(mFlightPlanner->getWayPoints());
        }
    }
    else if(packetType == "waypointreached")
    {
        WayPoint wpt;
        stream >> wpt;

        mFlightPlanner->slotWayPointReached(wpt);

        qDebug() << "process packet: wpt reached:" << QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z());

        mLogWidget->log(Information, "BaseStation::processPacket()", QString("reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));
    }
    else if(packetType == "waypointinserted")
    {
        // This happens when the rover reached the last waypoint and appended its own landing waypoint (probably just below the last one)
        quint16 index;
        stream >> index;

        WayPoint wpt;
        stream >> wpt;

        mFlightPlanner->slotWayPointInsertedByRover(index, wpt);

        qDebug() << "process packet: wpt appended by rover:" << QString("waypoint appended by rover: %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z());

        mLogWidget->log(Information, "BaseStation::processPacket()", QString("waypoint appended by rover: %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));
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

        // Normalize poseYaw between -180 and 180 for better graphing
        float poseYaw = pose.getYawDegrees() <= 180.0 ? pose.getYawDegrees() : pose.getYawDegrees() - 360.0;

        QVector<float> values;
        values << pitch << roll << thrust << yaw;
        values << pose.getPitchDegrees() << pose.getRollDegrees() << poseYaw;

        mPlotWidget->slotAppendData(values);
    }
    else
    {
        qDebug() << "BaseStation::processPacket(): unknown packetType" << packetType;
        mLogWidget->log(Error, "BaseStation::processPacket()", "unknown packetType: " + packetType);
        mIncomingDataBuffer.clear();
    }
}

void BaseStation::slotRoverWayPointsSet(const QList<WayPoint>& wayPoints)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypoints");
    stream << wayPoints;

    mLogWidget->log(Information, "BaseStation::slotSetWayPoints()", QString("transmitting %1 waypoints to rover").arg(wayPoints.size()));

    slotSendData(data);
}

void BaseStation::slotRoverWayPointInsert(const quint16& index, const WayPoint& wayPoint)
{
    qDebug() << "BaseStation::slotRoverWayPointInsert(): telling rover to insert wpt" << wayPoint << "before index" << index;

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    stream << QString("waypointinsert");
//    stream << hash;
    stream << index;
    stream << wayPoint;

    mLogWidget->log(Information, "BaseStation::slotRoverWayPointInsert()", QString("inserting 1 waypoint at index %2").arg(index));

    slotSendData(data);
}

void BaseStation::slotRoverWayPointDelete(const quint16& index)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    qDebug() << "BaseStation::slotRoverWayPointDelete(): telling rover to delete wpt" << index;

    stream << QString("waypointdelete");
//    stream << hash;
    stream << index;

    mLogWidget->log(Information, "BaseStation::slotRoverWayPointDelete()", QString("deleting waypoint at index %1").arg(index));

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

    qDebug() << "BaseStation::slotConnectToRover(): aborting and reconnecting...";

    mTcpSocket->abort();
    mTcpSocket->connectToHost(mConnectionDialog->getHostNameRover(), 12345);

    qDebug() << "BaseStation::slotConnectToRover(): aborting and reconnecting done.";
}

void BaseStation::slotSocketError(QAbstractSocket::SocketError socketError)
{
    qDebug() << "BaseStation::slotSocketError():" << socketError;

    mControlWidget->slotUpdateRoverConnection(false);

    if(socketError == QAbstractSocket::ConnectionRefusedError)
        QTimer::singleShot(2000, this, SLOT(slotConnectToRover()));
}

/*const WayPoint BaseStation::getNextWayPoint(void) const
{
    QList<WayPoint> waypoints = mFlightPlanner->getWayPoints();

    if(waypoints.size())
        return waypoints.first();
    else
        return WayPoint();
}*/

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

void BaseStation::slotWriteStats()
{
    static int second = 0;
    if(mOctree->getNumberOfItems() == 0 || mDateTimeLastLidarInput.secsTo(QDateTime::currentDateTime()) > 1)
    {
//        mDateTimeProgramStart = QDateTime::currentDateTime();
        return;
    }

//    if(!mDateTimeLastLidarInput.isValid()) mDateTimeLastLidarInput = QDateTime::currentDateTime();

    QTextStream out(mStatsFile);
//    out << mDateTimeProgramStart.secsTo(QDateTime::currentDateTime()) << "\t";
    out << second << "\t";
    out << mOctree->getNumberOfItems() << "\t";
    out << ((FlightPlannerPhysics*)mFlightPlanner)->getNumberOfPointsInCollisionOctree() << "\n";
    second++;
}

void BaseStation::slotAddLogFileMarkForPaper(QList<WayPoint> wptList)
{
    QTextStream out(mStatsFile);
    out << "# Generated" << wptList.size() << "waypoints\n";
}
