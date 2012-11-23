#include "basestation.h"

BaseStation::BaseStation() : QMainWindow()
{
    qDebug() << "BaseStation::BaseStation()";

//    // We need a small font.
//    QFont widgetFont = QApplication::font();
//    widgetFont.setPointSize(7);
//    QApplication::setFont(widgetFont);

//    mOctree = new Octree(
//                QVector3D(-100, -100, -100), // min
//                QVector3D(100, 100, 100),  // max
//                1000, // maxItemsPerLeaf
//                2000000 // maxExpectedSize
//                );
//    mOctree->mPointColor = QColor(128,128,128, 128);

    mProgress = 0;

    mPlotWidget = 0;
    mTimerJoystick = 0;
    mLogPlayer = 0;
    mPtuController = 0;
    mAudioPlayer = 0;
    mDiffCorrFetcher = 0;

    mMenuFile = menuBar()->addMenu("File");
    mMenuView = menuBar()->addMenu("View");
    mMenuWindowList = menuBar()->addMenu("Windows");

    mConnectionDialog = new ConnectionDialog(this);
    mConnectionDialog->exec();

//    mOctree->setMinimumPointDistance(.02f);

    mControlWidget = new ControlWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);
    mMenuWindowList->addAction("Control Widget", this, SLOT(slotToggleControlWidget()));

    mWirelessDevice = new WirelessDevice("wlan0");
//    connect(mWirelessDevice, SIGNAL(rssi(qint8)), mControlWidget, SLOT(slotUpdateWirelessRssi(qint8)));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    mMenuFile->addAction("Save Log", mLogWidget, SLOT(save()));
    mMenuWindowList->addAction("Log Viewer", this, SLOT(slotToggleLogWidget()));

    mGlWidget = new GlWidget(this);
    setCentralWidget(mGlWidget);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWidget, SLOT(slotUpdateView()));

    mPointCloud = new PointCloudCuda(QVector3D(-50, 0, -50), QVector3D(50, 50, 50));
    connect(mGlWidget, SIGNAL(initializingInGlContext()), mPointCloud, SLOT(slotInitialize()));

    // register for rendering
    mGlWidget->slotPointCloudRegister(mPointCloud);

    // Choose your weapon!
//    mFlightPlanner = new FlightPlannerCuda(this, mOctree);
//    mFlightPlanner = new FlightPlannerPhysics(this, mOctree);
//    mFlightPlanner->slotSetScanVolume(QVector3D(-10, -10, -10), QVector3D(10, 10, 10));
    mFlightPlanner = new FlightPlannerParticles(this, mPointCloud);
    mFlightPlanner->setGlWidget(mGlWidget);

    mPtuController = new PtuController("/dev/serial/by-id/usb-Hjelmslund_Electronics_USB485_ISO4W_HEVGI92A-if00-port0", this);
    addDockWidget(Qt::BottomDockWidgetArea, mPtuController);
    mPtuController->setVisible(false);
    mMenuWindowList->addAction("PTU Controller", this, SLOT(slotTogglePtuControllerWidget()));
    connect(mPtuController, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));

    if(mPtuController->isOpened())
    {
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with real PTU.");
    }
    else
    {
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with dummy PTU.");
    }

    connect(mFlightPlanner, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));

    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mFlightPlanner, SLOT(slotSetScanVolume(QVector3D, QVector3D)));
    connect(mControlWidget, SIGNAL(generateWaypoints()), mFlightPlanner, SLOT(slotGenerateWaypoints()));

    // When the controlwidget wants waypoints to be changed on the rover, tell flightplanner, which will take care of it.
    connect(mControlWidget, SIGNAL(wayPointInsert(const quint16&, const WayPoint&)), mFlightPlanner, SLOT(slotWayPointInsert(const quint16&, const WayPoint&)));
    connect(mControlWidget, SIGNAL(wayPointDelete(const quint16&)), mFlightPlanner, SLOT(slotWayPointDelete(const quint16&)));
    connect(mControlWidget, SIGNAL(wayPointSwap(quint16,quint16)), mFlightPlanner, SLOT(slotWayPointSwap(quint16,quint16)));

    // When the flightplanner changed waypoints, tell controlwidget, so changes are reflected inthe UI
    connect(mFlightPlanner, SIGNAL(wayPointDeleted(quint16)), mControlWidget, SLOT(slotWayPointDeleted(quint16)));
    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>*const)), mControlWidget, SLOT(slotSetWayPoints(QList<WayPoint>*const)));
    connect(mFlightPlanner, SIGNAL(wayPointInserted(quint16,WayPoint)), mControlWidget, SLOT(slotWayPointInserted(quint16,WayPoint)));

    mMenuFile->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    mMenuFile->addAction("Load Cloud", this, SLOT(slotImportCloud()));
    mMenuFile->addAction("Clear Cloud", this, SLOT(slotClearOctree()));

    connect(mGlWidget, SIGNAL(visualizeNow()), mFlightPlanner, SLOT(slotVisualize()));
    connect(mGlWidget, SIGNAL(visualizeNow()), mPtuController, SLOT(slotVisualize()));
    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWidget, SLOT(slotUpdateView()));

    mMenuFile->addAction("Screenshot", mGlWidget, SLOT(slotSaveImage()));
    mMenuView->addAction("ViewFromSide", mGlWidget, SLOT(slotViewFromSide()));
    mMenuView->addAction("ViewFromTop", mGlWidget, SLOT(slotViewFromTop()));
    mMenuView->addAction("Clear Trajectory", mFlightPlanner, SLOT(slotClearVehicleTrajectory()));

    QAction* actionRotateView = new QAction("Rotate View", this);
    actionRotateView->setCheckable(true);
    connect(actionRotateView, SIGNAL(triggered(bool)), mGlWidget, SLOT(slotEnableTimerRotation(bool)));
    mMenuView->addAction(actionRotateView);

    mActionEnableAudio = new QAction("AudioOut", this);
    mActionEnableAudio->setCheckable(true);
    mActionEnableAudio->setChecked(true);
    menuBar()->addAction(mActionEnableAudio);

    mPidControllerWidget = new PidControllerWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mPidControllerWidget);
    mMenuWindowList->addAction("PID Controllers", this, SLOT(slotTogglePidControllerWidget()));

/*
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
    menuBar()->addAction("TogglePlot", this, SLOT(slotTogglePlotWidget()));
*/

    // Only start RTK fetcher, RoverConnection, PtuController etc. if we're working online
    if(mConnectionDialog->result() == QDialog::Accepted)
    {
        mRoverConnection = new RoverConnection(mConnectionDialog->getRoverHostName(), mConnectionDialog->getRoverPort(), this);

        mPidControllerWidget->setControllers(mRoverConnection->getFlightControllerValues()); // set this once. Table is rebuilt when signal from roverconnection comes in

        mJoystick = new Joystick;
        if(!mJoystick->isValid())
        {
            QMessageBox::warning(this, "Joystick not found", "Joystick initialization failed, using manual control will be impossible.");
        }
        else
        {
            connect(mJoystick, SIGNAL(motion(const MotionCommand* const)), mRoverConnection, SLOT(slotSendMotionToKopter(const MotionCommand* const)));
            connect(mJoystick, SIGNAL(buttonStateChanged(quint8,bool)), SLOT(slotManageJoystick(quint8,bool)));
            mTimerJoystick = new QTimer(this);
            connect(mTimerJoystick, SIGNAL(timeout()), mJoystick, SLOT(slotEmitMotionCommands()));
        }

        connect(mPidControllerWidget, SIGNAL(controllerWeight(QString, QMap<QChar,float>)), mRoverConnection, SLOT(slotSendControllerWeights(QString, QMap<QChar,float>)));
//        connect(mRoverConnection, SIGNAL(flightControllerWeightsChanged()), mPidControllerWidget, SLOT(slotRebuild()));

        connect(mRoverConnection, SIGNAL(flightState(FlightState*const)), mControlWidget, SLOT(slotFlightStateChanged(FlightState*const)));

        connect(mRoverConnection, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));

        connect(mRoverConnection, SIGNAL(vehiclePose(const Pose* const)), mControlWidget, SLOT(slotUpdatePose(const Pose* const)));
        connect(mRoverConnection, SIGNAL(vehiclePose(const Pose* const)), mFlightPlanner, SLOT(slotVehiclePoseChanged(const Pose* const)));

        connect(mRoverConnection, SIGNAL(vehiclePose(const Pose* const)), mPtuController, SLOT(slotVehiclePoseChanged(const Pose* const)));

        connect(mRoverConnection, SIGNAL(vehiclePose(const Pose* const)), mGlWidget, SLOT(slotNewVehiclePose(const Pose* const)));

        connect(mRoverConnection, SIGNAL(connectionStatusRover(const bool)), mControlWidget, SLOT(slotUpdateConnectionRover(const bool)));

        connect(mRoverConnection, SIGNAL(scanData(const QVector<QVector3D>* const,const QVector3D* const)), this, SLOT(slotNewScanData(const QVector<QVector3D>* const, const QVector3D* const)));
        connect(mRoverConnection, SIGNAL(vehicleStatus(const VehicleStatus* const)), mControlWidget, SLOT(slotUpdateVehicleStatus(const VehicleStatus* const)));
        connect(mRoverConnection, SIGNAL(gnssStatus(const GnssStatus* const)), mControlWidget, SLOT(slotUpdateGnssStatus(const GnssStatus* const)));
        connect(mRoverConnection, SIGNAL(flightControllerValues(const FlightControllerValues* const)), SLOT(slotSetFlightControllerValues(const FlightControllerValues* const)));

        connect(mRoverConnection, SIGNAL(wayPointsHashFromRover(QString)), mFlightPlanner, SLOT(slotCheckWayPointsHashFromRover(QString)));
        connect(mRoverConnection, SIGNAL(wayPointReachedByRover(WayPoint)), mFlightPlanner, SLOT(slotWayPointReached(WayPoint)));
        connect(mRoverConnection, SIGNAL(wayPointInsertedByRover(quint16,WayPoint)), mFlightPlanner, SLOT(slotWayPointInsertedByRover(quint16,WayPoint)));


        // When FlightPlanner wants us to send waypoint updates to the rover...
        connect(mFlightPlanner, SIGNAL(wayPointInsertOnRover(quint16,WayPoint)), mRoverConnection, SLOT(slotRoverWayPointInsert(quint16,WayPoint)));
        connect(mFlightPlanner, SIGNAL(wayPointDeleteOnRover(quint16)), mRoverConnection, SLOT(slotRoverWayPointDelete(quint16)));
        connect(mFlightPlanner, SIGNAL(wayPointsSetOnRover(QList<WayPoint>*const)), mRoverConnection, SLOT(slotRoverWayPointsSet(const QList<WayPoint>* const)));

        mDiffCorrFetcher = new DiffCorrFetcher(mConnectionDialog->getRtkBaseHostName(), mConnectionDialog->getRtkBasePort(), this);
        connect(mDiffCorrFetcher, SIGNAL(differentialCorrections(QByteArray)), mRoverConnection, SLOT(slotSendDiffCorrToRover(QByteArray)));
        connect(mDiffCorrFetcher, SIGNAL(connectionStatus(bool)), mControlWidget, SLOT(slotUpdateConnectionRtk(bool)));

        menuBar()->addAction("Connect", mRoverConnection, SLOT(slotConnectToRover()));

        mAudioPlayer = new AudioPlayer;
        connect(mRoverConnection, SIGNAL(gnssStatus(const GnssStatus* const)), this, SLOT(slotSpeakGnssStatus(const GnssStatus* const)));
        mActionEnableAudio->setChecked(true); // enable audio out by default when in the field

        mRoverConnection->slotConnectToRover();
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working online, enabling RoverConnection+RtkFetcher+PtuController, disabling LogPlayer.");
    }
    else
    {
        mLogPlayer = new LogPlayer(this);
        mLogPlayer->setAllowedAreas(Qt::AllDockWidgetAreas);
        addDockWidget(Qt::BottomDockWidgetArea, mLogPlayer);
        connect(mLogPlayer, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mControlWidget, SLOT(slotUpdatePose(const Pose* const)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mFlightPlanner, SLOT(slotVehiclePoseChanged(const Pose* const)));

        //connect(mLogPlayer, SIGNAL(vehiclePoseLowFreq(Pose)), mPtuController, SLOT(slotVehiclePoseChanged(Pose)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mPtuController, SLOT(slotVehiclePoseChanged(const Pose* const)));

        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mGlWidget, SLOT(slotNewVehiclePose(const Pose* const)));
        connect(mLogPlayer, SIGNAL(scanData(const QVector<QVector3D>* const, const QVector3D* const)), this, SLOT(slotNewScanData(const QVector<QVector3D>* const, const QVector3D* const)));
    //    connect(mLogPlayer, SIGNAL(vehicleStatus(quint32,float,qint16,qint8)), this, SLOT(slotNewVehicleStatus(quint32,float,qint16,qint8)));
        connect(mLogPlayer, SIGNAL(gnssStatus(const GnssStatus* const)), mControlWidget, SLOT(slotUpdateGnssStatus(const GnssStatus* const)));

        mAudioPlayer = new AudioPlayer;
        connect(mLogPlayer, SIGNAL(gnssStatus(const GnssStatus* const)), this, SLOT(slotSpeakGnssStatus(const GnssStatus* const)));

        mPidControllerWidget->setEnabled(false); // do not allow changes to PID values while playing a recorded flight.
        mPidControllerWidget->setControllers(mLogPlayer->getFlightControllerValues()); // set this once. Table is rebuilt when signal from roverconnection comes in
//        connect(mLogPlayer, SIGNAL(flightControllerWeightsChanged()), mPidControllerWidget, SLOT(slotRebuild()));


        connect(mLogPlayer, SIGNAL(flightControllerValues(const FlightControllerValues* const)), SLOT(slotSetFlightControllerValues(const FlightControllerValues* const)));

        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working offline, disabling RoverConnection+RtkFetcher, enabling LogPlayer.");
    }

    mLogWidget->log(Information, "BaseStation::BaseStation()", "Startup finished, ready.");
}

BaseStation::~BaseStation()
{
    delete mPtuController;
    delete mPlotWidget;
    delete mPidControllerWidget;
    delete mLogPlayer;
    delete mFlightPlanner;
    delete mDiffCorrFetcher;
    delete mTimerJoystick;
    delete mGlWidget;
    delete mWirelessDevice;
    delete mAudioPlayer;
    delete mPointCloud;
}

void BaseStation::slotManageJoystick(quint8 button, bool pressed)
{
    if(button == 0)
    {
        if(pressed && !mTimerJoystick->isActive())
            mTimerJoystick->start(100);
        else if(!pressed && mTimerJoystick->isActive())
            mTimerJoystick->stop();
    }
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

void BaseStation::slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition)
{
    mPointCloud->slotInsertPoints(pointList);

    mGlWidget->slotUpdateView();

    mLogWidget->log(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("%1 points using %2 MB, %3 points processed.")
                .arg(mPointCloud->getNumberOfPoints())
                .arg((mPointCloud->getNumberOfPoints()*sizeof(QVector4D))/(1024.0f*1024.0f), 2, 'g')
                .arg(pointList->size()));

    //qDebug() << "BaseStation::slotNewScanData(): appended" << pointList->size() << "points to octree.";
}

void BaseStation::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(mPointCloud->exportToPly(fileName, this))
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

    if(mPointCloud->importFromPly(fileName, this))
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

void BaseStation::keyPressEvent(QKeyEvent* event)
{
//    if(event->key() == Qt::Key_Space)
//        addRandomPoint();
}

void BaseStation::slotClearOctree()
{
    mPointCloud->slotReset();
    mGlWidget->update();
}

void BaseStation::slotSpeakGnssStatus(const GnssStatus* const status)
{
    if(mAudioPlayer && mActionEnableAudio->isChecked())
    {
        if(status->error != GnssStatus::Error::NoError)
            mAudioPlayer->setSound(QString("../media/ins_error_%1.ogg").arg(status->getError().toLower().replace(' ', '_')));
        else if(!testBit(status->info, 11))
            mAudioPlayer->setSound(QString("../media/ins_error_heading_ambiguous.ogg"));
        else if(status->pvtMode != GnssStatus::PvtMode::RtkFixed)
            mAudioPlayer->setSound(QString("../media/gnss_mode_%1.ogg").arg(status->getPvtMode().toLower().replace(' ', '_')));
        else if(status->covariances > Pose::maximumUsableCovariance)
            mAudioPlayer->setSound(QString("../media/ins_covariances_too_high.ogg"));
        else
            mAudioPlayer->setSound(QString("../media/ins_nominal.ogg"));
    }
}

void BaseStation::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    // visualize pose and controller values
    mGlWidget->slotSetFlightControllerValues(fcv);

    mControlWidget->slotFlightStateChanged(&fcv->flightState);

//    if(!mPidControllerWidget->isPopulated()) mPidControllerWidget->slotUpdateWeights();

    mPidControllerWidget->slotUpdateValues();
}
