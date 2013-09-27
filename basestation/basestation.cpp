#include "basestation.h"
#include <QMenu>
#include <QMenuBar>

BaseStation::BaseStation() : QMainWindow()
{
    qDebug() << __PRETTY_FUNCTION__;

    mTimerJoystick = 0;
    mLogPlayer = 0;
//    mPtuController = 0;
    mAudioPlayer = 0;
    mDiffCorrFetcher = 0;

    mMenuFile = menuBar()->addMenu("File");
    mMenuView = menuBar()->addMenu("View");
    mMenuWindowList = menuBar()->addMenu("Windows");

    mConnectionDialog = new ConnectionDialog(this);
    mConnectionDialog->exec();

    mControlWidget = new ControlWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);
    mMenuWindowList->addAction("Control Widget", this, SLOT(slotToggleControlWidget()));

    mSatelliteWidget = new SatelliteWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mSatelliteWidget);
    mMenuWindowList->addAction("Satellite Manager", this, SLOT(slotToggleSatelliteWidget()));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    mMenuWindowList->addAction("Log Viewer", this, SLOT(slotToggleLogWidget()));

    mGlScene = new GlScene;
    mGlWindow = new GlWindow(mGlScene);
    QWidget* glContainer = QWidget::createWindowContainer(mGlWindow, this);
    glContainer->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setCentralWidget(glContainer);

    // Create a large cloud. Les large for notebook.
    const quint32 numberOfPoints = CudaHelper::isDeviceSupported ? 8*1024*1024 : 1 * 1024*1024;
    mPointCloud = new PointCloudCuda(Box3D(QVector3D(-512, 0, -512), QVector3D(512, 32, 512)), numberOfPoints, "DenseCloud");
    connect(mGlWindow, &GlWindow::message, mLogWidget, &LogWidget::log);

    // register dense pointcloud for rendering.
    mGlScene->slotPointCloudRegister(mPointCloud);

    mFlightPlanner = new FlightPlannerParticles(this, mGlWindow, mPointCloud);
    connect(mFlightPlanner, &FlightPlannerParticles::volumeLocal, mGlScene, &GlScene::slotSetVolumeLocal);
    connect(mFlightPlanner, &FlightPlannerParticles::volumeGlobal, mGlScene, &GlScene::slotSetVolumeGlobal);

    connect(mFlightPlanner, &FlightPlannerParticles::vboInfoParticles, mGlScene, &GlScene::slotSetVboInfoParticles);
    connect(mFlightPlanner, &FlightPlannerParticles::vboInfoGridOccupancy, mGlScene, &GlScene::slotSetVboInfoGridOccupancy);
    connect(mFlightPlanner, &FlightPlannerParticles::vboInfoGridPathPlanner, mGlScene, &GlScene::slotSetVboInfoGridPathPlanner);
    connect(mFlightPlanner, &FlightPlannerParticles::vboInfoGridInformationGain, mGlScene, &GlScene::slotSetVboInfoGridInformationGain);
    connect(mFlightPlanner, &FlightPlannerParticles::wayPointListAhead, mGlScene, &GlScene::slotSetWayPointListAhead);
    connect(mFlightPlanner, &FlightPlannerParticles::wayPointListPassed, mGlScene, &GlScene::slotSetWayPointListPassed);

    connect(mFlightPlanner, &FlightPlannerParticles::renderInformationGain, [=](const bool value) {mGlScene->mRenderInformationGain = value; mGlWindow->slotRenderLater();});
    connect(mFlightPlanner, &FlightPlannerParticles::renderOccupancyGrid, [=](const bool value) {mGlScene->mRenderOccupancyGrid = value; mGlWindow->slotRenderLater();});
    connect(mFlightPlanner, &FlightPlannerParticles::renderParticles, [=](const bool value) {mGlScene->mRenderParticles = value; mGlWindow->slotRenderLater();});
    connect(mFlightPlanner, &FlightPlannerParticles::renderPathPlannerGrid, [=](const bool value) {mGlScene->mRenderPathPlannerGrid = value; mGlWindow->slotRenderLater();});
    connect(mFlightPlanner, &FlightPlannerParticles::message, mLogWidget, &LogWidget::log);
    connect(mFlightPlanner, &FlightPlannerParticles::wayPoints, mControlWidget, &ControlWidget::slotSetWayPoints);
    connect(mFlightPlanner, &FlightPlannerParticles::suggestVisualization, mGlWindow, &GlWindow::slotRenderLater);

    connect(mGlWindow, &GlWindow::initializingInGlContext, mFlightPlanner, &FlightPlannerParticles::slotInitialize);


//    mPtuController = new PtuController("/dev/serial/by-id/usb-Hjelmslund_Electronics_USB485_ISO4W_HEVGI92A-if00-port0", this);
//    addDockWidget(Qt::BottomDockWidgetArea, mPtuController);
//    mPtuController->setVisible(false);
//    mMenuWindowList->addAction("PTU Controller", this, SLOT(slotTogglePtuControllerWidget()));
//    connect(mPtuController, &PtuController::message, mLogWidget, &LogWidget::log);

//    if(mPtuController->isOpened())
//        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with real PTU.");
//    else
//        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with dummy PTU.");

    connect(mControlWidget, &ControlWidget::wayPoints, mFlightPlanner, &FlightPlannerParticles::slotSetWayPoints);
    connect(mControlWidget, &ControlWidget::volumeGlobal, mFlightPlanner, &FlightPlannerParticles::slotSetVolumeGlobal);
    connect(mControlWidget, &ControlWidget::volumeLocal, mFlightPlanner, &FlightPlannerParticles::slotSetVolumeLocal);
    connect(mControlWidget, &ControlWidget::showUserInterface, mFlightPlanner, &FlightPlannerParticles::slotShowUserInterface);
    connect(mControlWidget, &ControlWidget::wayPointSelected, mGlScene, &GlScene::slotSetActiveWayPoint); // this should be in GlWindow...

    // Connect ControlWidget and FlightPlanner

    mMenuFile->addAction("Load Cloud", this, SLOT(slotImportCloud()));
    mMenuFile->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    mMenuFile->addAction("Save Log", mLogWidget, SLOT(save()));

    // Setup actions in menus
    QAction* action;

    action = new QAction("Reload Shaders", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mGlScene->reloadShaders(); mGlWindow->slotRenderLater();});

    action = new QAction("Clear Dense Cloud", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mPointCloud->slotReset(); mGlWindow->slotRenderLater();});

    mMenuView->insertSeparator(action);

    action = new QAction("Clear Trajectory", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mGlScene->slotClearVehicleTrajectory(); mGlWindow->slotRenderLater();});

    action = new QAction("Clear Passed Waypoints", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mFlightPlanner->slotClearWayPointsPassed(); mGlWindow->slotRenderLater();});

    action = new QAction("Show Dense Cloud", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {
        if(checked) mGlScene->slotPointCloudRegister(mPointCloud);
        else mGlScene->slotPointCloudUnregister(mPointCloud);
        mGlWindow->slotRenderLater();
    });

    mMenuView->insertSeparator(action);

    action = new QAction("Show Sparse Cloud", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {
        if(checked) mGlScene->slotPointCloudRegister(mFlightPlanner->getPointCloudColliders());
        else mGlScene->slotPointCloudUnregister(mFlightPlanner->getPointCloudColliders());
        mGlWindow->slotRenderLater();
    });

    action = new QAction("Show WayPoints Ahead", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderWayPointsAhead = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show WayPoints Passed", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderWayPointsPassed = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Axes Base", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderAxisBase = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Axes Vehicle", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderAxisVehicle = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Vehicle", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderVehicle = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Global BBox", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderBoundingBoxGlobal = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Local BBox", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderBoundingBoxLocal = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Raw Scan", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderRawScanRays = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Trajectory", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderTrajectory = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Satellite Signals", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlScene->mRenderSatelliteSignals = checked; mGlWindow->slotRenderLater();});

    MenuSlider* menuSlider;

    menuSlider = new MenuSlider("Rotate View", -1.0, 0.0, 1.0, this, 0.1f);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->slotSetCameraRotation(-value);});
    mMenuView->addAction(menuSlider);

    mMenuView->insertSeparator(menuSlider);

    menuSlider = new MenuSlider("BG Brightness", 0, 0.2, 1.0, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mBackgroundBrightness = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Particle VisSize", 0, 0.5, 2.0, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mParticleRadius = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Distance Threshold", 0, 30, 30, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mMaxPointVisualizationDistance = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Point Size", 0.1f, 1.0f, 10.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mPointCloudPointSize = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Point Alpha", 0.01f, 0.3f, 1.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mPointCloudPointAlpha = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Color Low", -5.0f, 0.0f, 20.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mPointCloudColorLow = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Color High", -5.0f, 10.0f, 20.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlScene->mPointCloudColorHigh = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    mActionEnableAudio = new QAction("Speech", this);
    mActionEnableAudio->setCheckable(true);
    mActionEnableAudio->setChecked(true);
    menuBar()->addAction(mActionEnableAudio);

    mPidControllerWidget = new PidControllerWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mPidControllerWidget);
    mMenuWindowList->addAction("PID Controllers", this, SLOT(slotTogglePidControllerWidget()));

    // Only start RTK fetcher, RoverConnection, PtuController etc. if we're working online
    if(mConnectionDialog->result() == QDialog::Accepted)
    {
        mOperatingMode = OperatingMode::OperatingOnline;
        mRoverConnection = new RoverConnection(mConnectionDialog->getRoverHostName(), mConnectionDialog->getRoverPort(), this);
        connect(mControlWidget, &ControlWidget::setScannerState, mRoverConnection, &RoverConnection::slotSendScannerState);

        mPidControllerWidget->setControllers(mRoverConnection->getFlightControllerValues()); // set this once. Table is rebuilt when signal from roverconnection comes in

        mJoystick = new Joystick;
        if(!mJoystick->isValid())
        {
            QMessageBox::warning(this, "Joystick not found", "Joystick initialization failed, using manual control will be impossible.");
        }
        else
        {
            connect(mJoystick, &Joystick::motion, mRoverConnection, &RoverConnection::slotSendMotionToKopter);
            connect(mJoystick, &Joystick::buttonStateChanged, this, &BaseStation::slotManageJoystick);
            mTimerJoystick = new QTimer(this);
            connect(mTimerJoystick, &QTimer::timeout, mJoystick, &Joystick::slotEmitMotionCommands);
        }

        connect(mPidControllerWidget, &PidControllerWidget::controllerWeight, mRoverConnection, &RoverConnection::slotSendControllerWeights);
        connect(mRoverConnection, &RoverConnection::flightState, mControlWidget, &ControlWidget::slotSetFlightState);
        connect(mRoverConnection, &RoverConnection::flightStateRestriction, mControlWidget, &ControlWidget::slotSetFlightStateRestriction);

        connect(mRoverConnection, &RoverConnection::message, mLogWidget, &LogWidget::log);

        connect(mRoverConnection, &RoverConnection::vehiclePose, mControlWidget, &ControlWidget::slotUpdatePose);
        connect(mRoverConnection, &RoverConnection::vehiclePose, mFlightPlanner, &FlightPlannerParticles::slotVehiclePoseChanged);

//        connect(mRoverConnection, &RoverConnection::vehiclePose, mPtuController, &PtuController::slotVehiclePoseChanged);

        connect(mRoverConnection, &RoverConnection::vehiclePose, mGlScene, &GlScene::slotNewVehiclePose);

        connect(mRoverConnection, &RoverConnection::connectionStatusRover, mControlWidget, &ControlWidget::slotUpdateConnectionRover);

        connect(mRoverConnection, &RoverConnection::scanData, mFlightPlanner, &FlightPlannerParticles::slotNewScanFused);
        connect(mRoverConnection, &RoverConnection::vehicleStatus, mControlWidget, &ControlWidget::slotUpdateVehicleStatus);
        connect(mRoverConnection, &RoverConnection::gnssStatus, mControlWidget, &ControlWidget::slotUpdateInsStatus);
        connect(mRoverConnection, &RoverConnection::gnssStatus, mGlScene, &GlScene::slotSetInsStatus);
        connect(mRoverConnection, &RoverConnection::flightControllerValues, this, &BaseStation::slotSetFlightControllerValues);

        connect(mRoverConnection, &RoverConnection::wayPointReachedByRover, mFlightPlanner, &FlightPlannerParticles::slotWayPointReached);

        connect(mRoverConnection, &RoverConnection::wayPoints, mFlightPlanner, &FlightPlannerParticles::slotSetWayPoints);
        connect(mFlightPlanner, &FlightPlannerParticles::wayPoints, mRoverConnection, &RoverConnection::slotSetWayPoints);

        mDiffCorrFetcher = new DiffCorrFetcher(mConnectionDialog->getRtkBaseHostName(), mConnectionDialog->getRtkBasePort(), this);
        connect(mDiffCorrFetcher, &DiffCorrFetcher::differentialCorrections, mRoverConnection, &RoverConnection::slotSendDiffCorrToRover);
        connect(mDiffCorrFetcher, &DiffCorrFetcher::connectionStatus, mControlWidget, &ControlWidget::slotUpdateConnectionDiffCorr);

        menuBar()->addAction("Connect", mRoverConnection, SLOT(slotConnectToRover()));

        mAudioPlayer = new AudioPlayer;
        connect(mRoverConnection, &RoverConnection::gnssStatus, this, &BaseStation::slotSpeakGnssStatus);
        mActionEnableAudio->setChecked(true); // enable audio out by default when in the field

        mRoverConnection->slotConnectToRover();
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working online, enabling RoverConnection+RtkFetcher+PtuController, disabling LogPlayer.");
    }
    else
    {
        mOperatingMode = OperatingMode::OperatingOffline;
        mLogPlayer = new LogPlayer(this);
        mLogPlayer->setAllowedAreas(Qt::AllDockWidgetAreas);
        addDockWidget(Qt::BottomDockWidgetArea, mLogPlayer);
        mMenuWindowList->addAction("Log Player", this, SLOT(slotToggleLogPlayer()));
        connect(mLogPlayer, &LogPlayer::message, mLogWidget, &LogWidget::log);
        connect(mLogPlayer, &LogPlayer::vehiclePose, mControlWidget, &ControlWidget::slotUpdatePose);
        connect(mLogPlayer, &LogPlayer::vehiclePose, mFlightPlanner, &FlightPlannerParticles::slotVehiclePoseChanged);
        connect(mLogPlayer, &LogPlayer::scanRaw, mGlScene, &GlScene::slotNewRawScan);

//        connect(mLogPlayer, &LogPlayer::vehiclePose, mPtuController, &PtuController::slotVehiclePoseChanged);

        connect(mLogPlayer, &LogPlayer::vehiclePose, mGlScene, &GlScene::slotNewVehiclePose);
        connect(mLogPlayer, &LogPlayer::scanFused, mFlightPlanner, &FlightPlannerParticles::slotNewScanFused);
        connect(mLogPlayer, &LogPlayer::gnssStatus, mControlWidget, &ControlWidget::slotUpdateInsStatus);
        connect(mLogPlayer, &LogPlayer::gnssStatus, mGlScene, &GlScene::slotSetInsStatus);
        connect(mLogPlayer, &LogPlayer::flightState, mControlWidget, &ControlWidget::slotSetFlightState);
        connect(mLogPlayer, &LogPlayer::flightStateRestriction, mControlWidget, &ControlWidget::slotSetFlightStateRestriction);

        mAudioPlayer = new AudioPlayer;
        connect(mLogPlayer, &LogPlayer::gnssStatus, this, &BaseStation::slotSpeakGnssStatus);

        mPidControllerWidget->setEnabled(false); // do not allow changes to PID values while playing a recorded flight.
        mPidControllerWidget->setControllers(mLogPlayer->getFlightControllerValues()); // set this once. Table is rebuilt when signal from roverconnection comes in

        connect(mLogPlayer, &LogPlayer::flightControllerValues, this, &BaseStation::slotSetFlightControllerValues);

        mLogWidget->log(Information, "BaseStation::BaseStation()", "Working offline, disabling RoverConnection+RtkFetcher, enabling LogPlayer.");
    }

    QSettings settings("BenAdler", "Basestation");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    // After connecting consumers of initializingInGlContext(), initialze glWindow, which will emit the signal.
    mGlWindow->slotInitialize();

    mLogWidget->log(Information, "BaseStation::BaseStation()", "Startup finished, ready.");
}

BaseStation::~BaseStation()
{
//    delete mPtuController;
    delete mPidControllerWidget;
    delete mLogPlayer;
    delete mFlightPlanner;
    delete mDiffCorrFetcher;
    delete mTimerJoystick;
    delete mGlWindow;
    delete mGlScene;
    delete mAudioPlayer;
    delete mPointCloud;
}

void BaseStation::closeEvent(QCloseEvent *event)
{
    qDebug() << "saving window state!";
    QSettings settings("BenAdler", "Basestation");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    QMainWindow::closeEvent(event);
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

void BaseStation::slotExportCloud()
{
    const QString fileName = QFileDialog::getSaveFileName(this, "Save cloud to", QString(), "PLY files (*.ply)");

    if(fileName.isNull()) return;

    if(mPointCloud->exportToFile(fileName, this))
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

    if(mPointCloud->importFromFile(fileName, this))
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

void BaseStation::slotSpeakGnssStatus(const GnssStatus* const status)
{
    if(mAudioPlayer && mActionEnableAudio->isChecked())
    {
        if(status->pvtMode != GnssStatus::PvtMode::RtkFixed)
            mAudioPlayer->setSound(QString("../media/gnss_mode_%1.ogg").arg(status->getPvtMode().toLower().replace(' ', '_')));
        else if(status->error != GnssStatus::Error::NoError)
            mAudioPlayer->setSound(QString("../media/ins_error_%1.ogg").arg(status->getError().toLower().replace(' ', '_')));
        else if(!testBit(status->info, 11))
            mAudioPlayer->setSound(QString("../media/ins_error_heading_ambiguous.ogg"));
        else if(status->covariances > Pose::maximumUsableCovariance)
            mAudioPlayer->setSound(QString("../media/ins_covariances_too_high.ogg"));
        else if(status->meanCorrAge > 50)
            mAudioPlayer->setSound(QString("../media/gnss_differential_corrections_missing.ogg"));
        else
            mAudioPlayer->setSound(QString("../media/ins_nominal.ogg"));
    }
}

void BaseStation::slotSetFlightControllerValues(const FlightControllerValues* const fcv)
{
    // visualize pose and controller values
    mGlScene->slotSetFlightControllerValues(fcv);
    mPidControllerWidget->slotUpdateValues();
}
