#include "basestation.h"
#include <QMenu>
#include <QMenuBar>


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

//    mPlotWidget = 0;
    mTimerJoystick = 0;
    mLogPlayer = 0;
    mPtuController = 0;
    mAudioPlayer = 0;
    mDiffCorrFetcher = 0;

    // Allow focusing this by tabbing and clicking
    setFocusPolicy(Qt::StrongFocus);

    mMenuFile = menuBar()->addMenu("File");
    mMenuView = menuBar()->addMenu("View");
    mMenuWindowList = menuBar()->addMenu("Windows");

    mConnectionDialog = new ConnectionDialog(this);
    mConnectionDialog->exec();

//    mOctree->setMinimumPointDistance(.02f);

    mControlWidget = new ControlWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mControlWidget);
    mMenuWindowList->addAction("Control Widget", this, SLOT(slotToggleControlWidget()));

//    mWirelessDevice = new WirelessDevice("wlan0");
//    connect(mWirelessDevice, &WirelessDevice::rssi, mControlWidget, &ControlWidget::slotUpdateWirelessRssi);

    mSatelliteWidget = new SatelliteWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mSatelliteWidget);
    mMenuWindowList->addAction("Satellite Manager", this, SLOT(slotToggleSatelliteWidget()));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    mMenuWindowList->addAction("Log Viewer", this, SLOT(slotToggleLogWidget()));

    mGlWindow = new GlWindow;
    QWidget* glContainer = QWidget::createWindowContainer(mGlWindow, this);
    glContainer->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setCentralWidget(glContainer);
    connect(mControlWidget, &ControlWidget::setScanVolume, mGlWindow, &GlWindow::slotRenderLater);

//    mPointCloud = new PointCloudCuda(QVector3D(-64, -2, -64), QVector3D(64, 30, 64)/*, 250000*/);
    mPointCloud = new PointCloudCuda(QVector3D(-50, 0, -50), QVector3D(50, 30, 50)/*, 250000*/);
    connect(mGlWindow, &GlWindow::message, mLogWidget, &LogWidget::log);
    connect(mGlWindow, &GlWindow::initializingInGlContext, mPointCloud, &PointCloudCuda::slotInitialize);
    // After connecting consumers of initializingInGlContext(), initialze glWindow, which will emit the signal.
    mGlWindow->slotInitialize();

    // register dense pointcloud for rendering.
    mGlWindow->slotPointCloudRegister(mPointCloud);

    // Choose your weapon!
//    mFlightPlanner = new FlightPlannerCuda(this, mGlWidget, mPointCloud);
//    mFlightPlanner = new FlightPlannerPhysics(this, mGlWidget, mPointCloud);
//    mFlightPlanner->slotSetScanVolume(QVector3D(-10, -10, -10), QVector3D(10, 10, 10));
    mFlightPlanner = new FlightPlannerParticles(this, mGlWindow, mPointCloud);

    mPtuController = new PtuController("/dev/serial/by-id/usb-Hjelmslund_Electronics_USB485_ISO4W_HEVGI92A-if00-port0", this);
    addDockWidget(Qt::BottomDockWidgetArea, mPtuController);
    mPtuController->setVisible(false);
    mMenuWindowList->addAction("PTU Controller", this, SLOT(slotTogglePtuControllerWidget()));
    connect(mPtuController, &PtuController::message, mLogWidget, &LogWidget::log);

    if(mPtuController->isOpened())
    {
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with real PTU.");
    }
    else
    {
        mLogWidget->log(Information, "BaseStation::BaseStation()", "Enabling PtuController with dummy PTU.");
    }

    connect(mFlightPlanner, &FlightPlannerInterface::message, mLogWidget, &LogWidget::log);

    connect(mControlWidget, &ControlWidget::setScanVolume, mFlightPlanner, &FlightPlannerInterface::slotSetScanVolume);
    connect(mControlWidget, &ControlWidget::showUserInterface, mFlightPlanner, &FlightPlannerInterface::slotShowUserInterface);

    // Connect ControlWidget and FlightPlanner
    connect(mControlWidget, &ControlWidget::wayPoints, mFlightPlanner, &FlightPlannerInterface::slotSetWayPoints);
    connect(mFlightPlanner, &FlightPlannerInterface::wayPoints, mControlWidget, &ControlWidget::slotSetWayPoints);

    connect(mGlWindow, &GlWindow::visualizeNow, mFlightPlanner, &FlightPlannerInterface::slotVisualize);
    connect(mGlWindow, &GlWindow::visualizeNow, mPtuController, &PtuController::slotVisualize);
    connect(mFlightPlanner, &FlightPlannerInterface::suggestVisualization, mGlWindow, &GlWindow::slotRenderLater);

    mMenuFile->addAction("Load Cloud", this, SLOT(slotImportCloud()));
    mMenuFile->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    mMenuFile->addAction("Save Log", mLogWidget, SLOT(save()));

    // Setup actions in menus
    QAction* action;

    action = new QAction("Reload Shaders", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mGlWindow->reloadShaders(); mGlWindow->slotRenderLater();});

    action = new QAction("Clear Dense Cloud", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mPointCloud->slotReset(); mGlWindow->slotRenderLater();});

    action = new QAction("Clear Trajectory", this);
    mMenuView->addAction(action);
    connect(action, &QAction::triggered, [=]() {mGlWindow->slotClearVehicleTrajectory(); mGlWindow->slotRenderLater();});


    action = new QAction("Rotate View", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(false);
    connect(mGlWindow, &GlWindow::rotating, action, &QAction::setChecked);
    connect(action, &QAction::triggered, [=](const bool &checked) {
        mGlWindow->slotEnableTimerRotation(checked);
    });

    mMenuView->insertSeparator(action);

    action = new QAction("Show Dense Cloud", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {
        if(checked) mGlWindow->slotPointCloudRegister(mPointCloud);
        else mGlWindow->slotPointCloudUnregister(mPointCloud);
        mGlWindow->slotRenderLater();
    });

    action = new QAction("Show Axes Base", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlWindow->mRenderAxisBase = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Axes Vehicle", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlWindow->mRenderAxisVehicle = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Vehicle", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlWindow->mRenderVehicle = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Global BBox", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mFlightPlanner->slotSetRenderGlobalBoundingBox(checked); mGlWindow->slotRenderLater();});

    action = new QAction("Show Local BBox", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mFlightPlanner->slotSetRenderLocalBoundingBox(checked); mGlWindow->slotRenderLater();});

    action = new QAction("Show Raw Scan", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlWindow->mRenderRawScanRays = checked; mGlWindow->slotRenderLater();});

    action = new QAction("Show Trajectory", this);
    mMenuView->addAction(action);
    action->setCheckable(true);
    action->setChecked(true);
    connect(action, &QAction::triggered, [=](const bool &checked) {mGlWindow->mRenderTrajectory = checked; mGlWindow->slotRenderLater();});


    MenuSlider* menuSlider;

    menuSlider = new MenuSlider("BG Brightness", 0, 0.2, 1.0, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mBackgroundBrightness = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    mMenuView->insertSeparator(action);

    menuSlider = new MenuSlider("Distance Threshold", 0, 30, 30, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mMaxPointVisualizationDistance = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Point Size", 0.1f, 1.0f, 10.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mPointCloudPointSize = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Point Alpha", 0.01f, 0.3f, 1.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mPointCloudPointAlpha = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Color Low", -5.0f, 0.0f, 10.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mPointCloudColorLow = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    menuSlider = new MenuSlider("Color High", 0.0f, 10.0f, 30.0f, this);
    connect(menuSlider, &MenuSlider::value, [=](const float &value) {mGlWindow->mPointCloudColorHigh = value; mGlWindow->slotRenderLater();});
    mMenuView->addAction(menuSlider);

    mActionEnableAudio = new QAction("Speech", this);
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
            connect(mJoystick, &Joystick::motion, mRoverConnection, &RoverConnection::slotSendMotionToKopter);
            connect(mJoystick, &Joystick::buttonStateChanged, this, &BaseStation::slotManageJoystick);
            mTimerJoystick = new QTimer(this);
            connect(mTimerJoystick, &QTimer::timeout, mJoystick, &Joystick::slotEmitMotionCommands);
        }

        connect(mPidControllerWidget, &PidControllerWidget::controllerWeight, mRoverConnection, &RoverConnection::slotSendControllerWeights);
        connect(mRoverConnection, &RoverConnection::flightState, mControlWidget, &ControlWidget::slotFlightStateChanged);

        connect(mRoverConnection, &RoverConnection::message, mLogWidget, &LogWidget::log);

        connect(mRoverConnection, &RoverConnection::vehiclePose, mControlWidget, &ControlWidget::slotUpdatePose);
        connect(mRoverConnection, &RoverConnection::vehiclePose, mFlightPlanner, &FlightPlannerInterface::slotVehiclePoseChanged);

        connect(mRoverConnection, &RoverConnection::vehiclePose, mPtuController, &PtuController::slotVehiclePoseChanged);

        connect(mRoverConnection, &RoverConnection::vehiclePose, mGlWindow, &GlWindow::slotNewVehiclePose);

        connect(mRoverConnection, &RoverConnection::connectionStatusRover, mControlWidget, &ControlWidget::slotUpdateConnectionRover);

        connect(mRoverConnection, &RoverConnection::scanData, mFlightPlanner, &FlightPlannerInterface::slotNewScanFused);
        connect(mRoverConnection, &RoverConnection::vehicleStatus, mControlWidget, &ControlWidget::slotUpdateVehicleStatus);
        connect(mRoverConnection, &RoverConnection::gnssStatus, mControlWidget, &ControlWidget::slotUpdateGnssStatus);
        connect(mRoverConnection, &RoverConnection::flightControllerValues, this, &BaseStation::slotSetFlightControllerValues);

        connect(mRoverConnection, &RoverConnection::wayPointReachedByRover, mFlightPlanner, &FlightPlannerInterface::slotWayPointReached);

        connect(mRoverConnection, &RoverConnection::wayPoints, mFlightPlanner, &FlightPlannerInterface::slotSetWayPoints);
        connect(mFlightPlanner, &FlightPlannerInterface::wayPoints, mRoverConnection, &RoverConnection::slotSetWayPoints);

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
        mLogPlayer = new LogPlayer(this);
        mLogPlayer->setAllowedAreas(Qt::AllDockWidgetAreas);
        addDockWidget(Qt::BottomDockWidgetArea, mLogPlayer);
        mMenuWindowList->addAction("Log Player", this, SLOT(slotToggleLogPlayer()));
        connect(mLogPlayer, &LogPlayer::message, mLogWidget, &LogWidget::log);
        connect(mLogPlayer, &LogPlayer::vehiclePose, mControlWidget, &ControlWidget::slotUpdatePose);
        connect(mLogPlayer, &LogPlayer::vehiclePose, mFlightPlanner, &FlightPlannerInterface::slotVehiclePoseChanged);
        connect(mLogPlayer, &LogPlayer::scanRaw, mGlWindow, &GlWindow::slotNewRawScan);

        connect(mLogPlayer, &LogPlayer::vehiclePose, mPtuController, &PtuController::slotVehiclePoseChanged);

        connect(mLogPlayer, &LogPlayer::vehiclePose, mGlWindow, &GlWindow::slotNewVehiclePose);
        connect(mLogPlayer, &LogPlayer::scanFused, mFlightPlanner, &FlightPlannerInterface::slotNewScanFused);
        connect(mLogPlayer, &LogPlayer::gnssStatus, mControlWidget, &ControlWidget::slotUpdateGnssStatus);

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

    mLogWidget->log(Information, "BaseStation::BaseStation()", "Startup finished, ready.");
}

BaseStation::~BaseStation()
{
    delete mPtuController;
//    delete mPlotWidget;
    delete mPidControllerWidget;
    delete mLogPlayer;
    delete mFlightPlanner;
    delete mDiffCorrFetcher;
    delete mTimerJoystick;
    delete mGlWindow;
    //delete mWirelessDevice;
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
    mFlightPlanner->keyPressEvent(event);
//    if(mLogPlayer) mLogPlayer->keyPressEvent(event);

      QMainWindow::keyPressEvent(event);

}

//void BaseStation::slotClearCloud()
//{
//    mPointCloud->slotReset();
//    mGlWindow->slotRenderNow();
//}

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
    mGlWindow->slotSetFlightControllerValues(fcv);

    mControlWidget->slotFlightStateChanged(&fcv->flightState);

//    if(!mPidControllerWidget->isPopulated()) mPidControllerWidget->slotUpdateWeights();

    mPidControllerWidget->slotUpdateValues();
}
