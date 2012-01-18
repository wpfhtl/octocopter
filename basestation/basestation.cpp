#include "basestation.h"

BaseStation::BaseStation() : QMainWindow()
{
    qDebug() << "BaseStation::BaseStation()";

//    // We need a small font.
//    QFont widgetFont = QApplication::font();
//    widgetFont.setPointSize(7);
//    QApplication::setFont(widgetFont);

    mOctree = new Octree(
            QVector3D(-100, -100, -100), // min
            QVector3D(100, 100, 100),  // max
            1000);

    mProgress = 0;

    mConnectionDialog = new ConnectionDialog(this);
    mConnectionDialog->exec();

    mOctree->setMinimumPointDistance(0.0001f);
    mOctree->setPointHandler(OpenGlUtilities::drawPoint);

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
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);

    mWirelessDevice = new WirelessDevice("wlan0");
    connect(mWirelessDevice, SIGNAL(rssi(qint8)), mControlWidget, SLOT(slotUpdateWirelessRssi(qint8)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    menuBar()->addAction("Save Log", mLogWidget, SLOT(save()));

    mFlightPlanner = new FlightPlannerPhysics(this, mOctree);
    // GlWidget and CUDA-based FlightPlanners have an intimate relationship because
    // cudaGlSetGlDevice() needs to be called in GL context and before any other CUDA calls.
//    mFlightPlanner = new FlightPlannerCuda(this, mOctree);

    mFlightPlanner->slotSetScanVolume(QVector3D(-50, -10, -35), QVector3D(50, 40, 35));

    // Just for adding a line to the log file for marking a new waypoint generation iteration
    connect(mFlightPlanner, SIGNAL(wayPointsSetOnRover(QList<WayPoint>)), SLOT(slotAddLogFileMarkForPaper(QList<WayPoint>)));
    connect(mFlightPlanner, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));

    mGlWidget = new GlWidget(this, mOctree, mFlightPlanner);
    connect(mGlWidget, SIGNAL(initializingInGlContext()), mFlightPlanner, SLOT(slotInitialize())); // init CUDA when GlWidget inits
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(update()));
    connect(mGlWidget, SIGNAL(mouseClickedAtWorldPos(Qt::MouseButton, QVector3D)), mControlWidget, SLOT(slotSetWayPointCoordinateFields(Qt::MouseButton, QVector3D)));
    setCentralWidget(mGlWidget);

    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    connect(mControlWidget, SIGNAL(wayPointInsert(const quint16&, const WayPoint&)), mFlightPlanner, SLOT(slotWayPointInsert(const quint16&, const WayPoint&)));
    connect(mControlWidget, SIGNAL(wayPointDelete(const quint16&)), mFlightPlanner, SLOT(slotWayPointDelete(const quint16&)));
    connect(mControlWidget, SIGNAL(wayPointSwap(quint16,quint16)), mFlightPlanner, SLOT(slotWayPointSwap(quint16,quint16)));

    connect(mFlightPlanner, SIGNAL(wayPointDeleted(quint16)), mControlWidget, SLOT(slotWayPointDeleted(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>)), mControlWidget, SLOT(slotSetWayPoints(QList<WayPoint>)));
    connect(mFlightPlanner, SIGNAL(wayPointInserted(quint16,WayPoint)), mControlWidget, SLOT(slotWayPointInserted(quint16,WayPoint)));

    menuBar()->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    menuBar()->addAction("Load Cloud", this, SLOT(slotImportCloud()));

    connect(mGlWidget, SIGNAL(visualizeNow()), mFlightPlanner, SLOT(slotVisualize()));
    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWidget, SLOT(updateGL()));

    menuBar()->addAction("Screenshot", mGlWidget, SLOT(slotSaveImage()));

    menuBar()->addAction("ViewFromSide", mGlWidget, SLOT(slotViewFromSide()));
    menuBar()->addAction("ViewFromTop", mGlWidget, SLOT(slotViewFromTop()));

    menuBar()->addAction("TogglePlot", this, SLOT(slotTogglePlot()));

    mLogPlayer = new LogPlayer(this);
    mLogPlayer->setAllowedAreas(Qt::AllDockWidgetAreas);
    addDockWidget(Qt::BottomDockWidgetArea, mLogPlayer);
    connect(mLogPlayer, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));
    connect(mLogPlayer, SIGNAL(vehiclePose(Pose)), mControlWidget, SLOT(slotUpdatePose(Pose)));
    connect(mLogPlayer, SIGNAL(vehiclePose(Pose)), mFlightPlanner, SLOT(slotVehiclePoseChanged(Pose)));
    connect(mLogPlayer, SIGNAL(scanData(QList<QVector3D>,QVector3D)), this, SLOT(slotNewScanData(QList<QVector3D>,QVector3D)));
//    connect(mLogPlayer, SIGNAL(vehicleStatus(quint32,float,qint16,qint8)), this, SLOT(slotNewVehicleStatus(quint32,float,qint16,qint8)));
    connect(mLogPlayer, SIGNAL(gpsStatus(GpsStatusInformation::GpsStatus)), mControlWidget, SLOT(slotUpdateGpsStatus(GpsStatusInformation::GpsStatus)));
//    connect(mLogPlayer, SIGNAL(controllerValues(QVector<float>)), mPlotWidget, SLOT(slotAppendData(QVector<float>)));

    mPlotWidget = new PlotWidget(this);
    mPlotWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
    mPlotWidget->setVisible(false);
    addDockWidget(Qt::LeftDockWidgetArea, mPlotWidget);

    mPlotWidget->createCurve("cPitch");
    mPlotWidget->createCurve("cRoll");
    mPlotWidget->createCurve("cThrust");
    mPlotWidget->createCurve("cYaw");

    mPlotWidget->createCurve("pPitch");
    mPlotWidget->createCurve("pRoll");
    mPlotWidget->createCurve("pYaw");


    // Only start RTK fetcher and RoverConnection if we're not working offline
    if(mConnectionDialog->result() == QDialog::Accepted)
    {
        mRoverConnection = new RoverConnection(mConnectionDialog->getRoverHostName(), mConnectionDialog->getRoverPort(), this);
        connect(mRoverConnection, SIGNAL(vehiclePose(Pose)), mControlWidget, SLOT(slotUpdatePose(Pose)));
        connect(mRoverConnection, SIGNAL(vehiclePose(Pose)), mFlightPlanner, SLOT(slotVehiclePoseChanged(Pose)));
        connect(mRoverConnection, SIGNAL(connectionStatusRover(bool)), mControlWidget, SLOT(slotUpdateConnectionRover(bool)));

        connect(mRoverConnection, SIGNAL(scanData(QList<QVector3D>,QVector3D)), this, SLOT(slotNewScanData(QList<QVector3D>,QVector3D)));
        connect(mRoverConnection, SIGNAL(vehicleStatus(quint32,float,qint16,qint8)), this, SLOT(slotNewVehicleStatus(quint32,float,qint16,qint8)));
        connect(mRoverConnection, SIGNAL(gpsStatus(GpsStatusInformation::GpsStatus)), mControlWidget, SLOT(slotUpdateGpsStatus(GpsStatusInformation::GpsStatus)));
        connect(mRoverConnection, SIGNAL(controllerValues(QVector<float>)), mPlotWidget, SLOT(slotAppendData(QVector<float>)));

        connect(mRoverConnection, SIGNAL(wayPointsHashFromRover(QString)), mFlightPlanner, SLOT(slotCheckWayPointsHashFromRover(QString)));
        connect(mRoverConnection, SIGNAL(wayPointReachedByRover(WayPoint)), mFlightPlanner, SLOT(slotWayPointReached(WayPoint)));
        connect(mRoverConnection, SIGNAL(wayPointInsertedByRover(quint16,WayPoint)), mFlightPlanner, SLOT(slotWayPointInsertedByRover(quint16,WayPoint)));

        // When FlightPlanner wants us to send waypoint updates to the rover...
        connect(mFlightPlanner, SIGNAL(wayPointInsertOnRover(quint16,WayPoint)), mRoverConnection, SLOT(slotRoverWayPointInsert(quint16,WayPoint)));
        connect(mFlightPlanner, SIGNAL(wayPointDeleteOnRover(quint16)), mRoverConnection, SLOT(slotRoverWayPointDelete(quint16)));
        connect(mFlightPlanner, SIGNAL(wayPointsSetOnRover(QList<WayPoint>)), mRoverConnection, SLOT(slotRoverWayPointsSet(QList<WayPoint>)));

        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working online, initialized RoverConnection and RtkFetcher, now connecting...");

        mRtkFetcher = new RtkFetcher(mConnectionDialog->getRtkBaseHostName(), mConnectionDialog->getRtkBasePort(), this);
        connect(mRtkFetcher, SIGNAL(rtkData(QByteArray)), mRoverConnection, SLOT(slotSendRtkDataToRover(QByteArray)));
        connect(mRtkFetcher, SIGNAL(connectionStatus(bool)), mControlWidget, SLOT(slotUpdateConnectionRtk(bool)));

        menuBar()->addAction("Connect", mRoverConnection, SLOT(slotConnectToRover()));

        mRoverConnection->slotConnectToRover();
    }
    else
    {
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working offline, RoverConnection and RtkFetcher not available.");
    }

    mLogWidget->log(Information, "BaseStation::BaseStation()", "Startup finished, ready.");
}

BaseStation::~BaseStation()
{
    mStatsFile->close();
}

void BaseStation::slotNewVehicleStatus(const quint32& missionRunTime, const float& batteryVoltage, const qint16& barometricHeight, const qint8& wirelessRssi)
{
    mControlWidget->slotUpdateMissionRunTime(missionRunTime);
    mControlWidget->slotUpdateBattery(batteryVoltage);
    mControlWidget->slotUpdateBarometricHeight(barometricHeight);
    // Really, or use our local RSSI?
    mControlWidget->slotUpdateWirelessRssi(wirelessRssi);
}

void BaseStation::slotNewImage(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData)
{
    //qDebug() << QDateTime::currentDateTime().toString("hh:mm:ss:zzz") << "camera" << cameraName << "pos" << cameraPosition << "orientation" << cameraOrientation << "image bytearray size is" << imageData.size();

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

void BaseStation::slotNewScanData(const QList<QVector3D>& pointList, const QVector3D& scannerPosition)
{
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

    mLogWidget->log(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("%1 points using %2 MB, %3 nodes, %4 points added.")
                .arg(mOctree->getNumberOfItems())
                .arg((mOctree->getNumberOfItems()*sizeof(LidarPoint))/1000000.0, 2, 'g')
                .arg(mOctree->getNumberOfNodes()).arg(pointList.size()));

    qDebug() << "RoverConnection::processPacket(): appended" << pointList.size() << "points to octree.";
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
    const QString fileName = QFileDialog::getOpenFileName(this, "Load cloud from", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    QList<Octree*> octreesToFill;
    octreesToFill.append(mOctree);

    QList<FlightPlannerInterface*> flightPlannersToFill;
    flightPlannersToFill.append(mFlightPlanner);

    if(PlyManager::loadPly(this, octreesToFill, flightPlannersToFill, fileName))
    {
        mLogWidget->log(Information, "BaseStation::slotImportCloud()", "Successfully loaded cloud from " + fileName);
        QMessageBox::information(this, "Cloud import", "Successfully loaded cloud from\n" + fileName, "OK");
    }
    else
    {
        mLogWidget->log(Error, "BaseStation::slotImportCloud()", "Failed saving cloud to file " + fileName);
        QMessageBox::information(this, "Cloud import", "Failed loading cloud from file\n\n" + fileName, "OK");
    }
}

void BaseStation::slotTogglePlot()
{
    mPlotWidget->setVisible(!mPlotWidget->isVisible());
}

void BaseStation::keyPressEvent(QKeyEvent* event)
{
//    if(event->key() == Qt::Key_Space)
//        addRandomPoint();
}

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
