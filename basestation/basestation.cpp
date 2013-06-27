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

    mWirelessDevice = new WirelessDevice("wlan0");
//    connect(mWirelessDevice, SIGNAL(rssi(qint8)), mControlWidget, SLOT(slotUpdateWirelessRssi(qint8)));

    mSatelliteWidget = new SatelliteWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mSatelliteWidget);
    mMenuWindowList->addAction("Satellite Manager", this, SLOT(slotToggleSatelliteWidget()));

    mLogWidget = new LogWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea, mLogWidget);
    mMenuFile->addAction("Save Log", mLogWidget, SLOT(save()));
    mMenuWindowList->addAction("Log Viewer", this, SLOT(slotToggleLogWidget()));

    mGlWindow = new GlWindow;
    QWidget* glContainer = QWidget::createWindowContainer(mGlWindow, this);
    glContainer->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setCentralWidget(glContainer);
    connect(mControlWidget, SIGNAL(setScanVolume(QVector3D,QVector3D)), mGlWindow, SLOT(slotRenderLater()));

//    mPointCloud = new PointCloudCuda(QVector3D(-64, -2, -64), QVector3D(64, 30, 64)/*, 250000*/);
    mPointCloud = new PointCloudCuda(QVector3D(-50, 0, -50), QVector3D(50, 30, 50)/*, 250000*/);
    connect(mGlWindow, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));
    connect(mGlWindow, SIGNAL(initializingInGlContext()), mPointCloud, SLOT(slotInitialize()));
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
    connect(mControlWidget, SIGNAL(showUserInterface()), mFlightPlanner, SLOT(slotShowUserInterface()));

    // Connect ControlWidget and FlightPlanner
    connect(mControlWidget, SIGNAL(wayPoints(QList<WayPoint>*const,WayPointListSource)), mFlightPlanner, SLOT(slotSetWayPoints(QList<WayPoint>*const,WayPointListSource)));
    connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>*const,WayPointListSource)), mControlWidget, SLOT(slotSetWayPoints(QList<WayPoint>*const,WayPointListSource)));

    mMenuFile->addAction("Save Cloud", this, SLOT(slotExportCloud()));
    mMenuFile->addAction("Load Cloud", this, SLOT(slotImportCloud()));
    mMenuFile->addAction("Clear Cloud", this, SLOT(slotClearCloud()));

    connect(mGlWindow, SIGNAL(visualizeNow()), mFlightPlanner, SLOT(slotVisualize()));
    connect(mGlWindow, SIGNAL(visualizeNow()), mPtuController, SLOT(slotVisualize()));
    connect(mFlightPlanner, SIGNAL(suggestVisualization()), mGlWindow, SLOT(slotRenderLater()));

    mMenuView->addAction("Dense PointCloud", this, SLOT(slotToggleViewPointCloudDense()));
    mMenuView->addAction("ViewFromSide", mGlWindow, SLOT(slotViewFromSide()));
    mMenuView->addAction("ViewFromTop", mGlWindow, SLOT(slotViewFromTop()));
    mMenuView->addAction("Clear Trajectory", mFlightPlanner, SLOT(slotClearVehicleTrajectory()));

    QAction* actionRotateView = new QAction("Rotate View", this);
    actionRotateView->setCheckable(true);
    connect(actionRotateView, SIGNAL(triggered(bool)), mGlWindow, SLOT(slotEnableTimerRotation(bool)));
    connect(mGlWindow, SIGNAL(rotating(bool)), actionRotateView, SLOT(setChecked(bool)));
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

        connect(mRoverConnection, SIGNAL(vehiclePose(const Pose* const)), mGlWindow, SLOT(slotNewVehiclePose(const Pose* const)));

        connect(mRoverConnection, SIGNAL(connectionStatusRover(const bool)), mControlWidget, SLOT(slotUpdateConnectionRover(const bool)));

        connect(mRoverConnection, SIGNAL(scanData(float*const, quint32, QVector3D*const)), mFlightPlanner, SLOT(slotNewScanData(float*const,quint32,QVector3D*const)));
        connect(mRoverConnection, SIGNAL(vehicleStatus(const VehicleStatus* const)), mControlWidget, SLOT(slotUpdateVehicleStatus(const VehicleStatus* const)));
        connect(mRoverConnection, SIGNAL(gnssStatus(const GnssStatus* const)), mControlWidget, SLOT(slotUpdateGnssStatus(const GnssStatus* const)));
        connect(mRoverConnection, SIGNAL(flightControllerValues(const FlightControllerValues* const)), SLOT(slotSetFlightControllerValues(const FlightControllerValues* const)));

        connect(mRoverConnection, SIGNAL(wayPointReachedByRover(WayPoint)), mFlightPlanner, SLOT(slotWayPointReached(WayPoint)));

        connect(mRoverConnection, SIGNAL(wayPoints(QList<WayPoint>*const,WayPointListSource)), mFlightPlanner, SLOT(slotSetWayPoints(QList<WayPoint>*const,WayPointListSource)));
        connect(mFlightPlanner, SIGNAL(wayPoints(QList<WayPoint>*const,WayPointListSource)), mRoverConnection, SLOT(slotSetWayPoints(QList<WayPoint>*const,WayPointListSource)));

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
        mMenuWindowList->addAction("Log Player", this, SLOT(slotToggleLogPlayer()));
        connect(mLogPlayer, SIGNAL(message(LogImportance,QString,QString)), mLogWidget, SLOT(log(LogImportance,QString,QString)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mControlWidget, SLOT(slotUpdatePose(const Pose* const)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mFlightPlanner, SLOT(slotVehiclePoseChanged(const Pose* const)));
        connect(mLogPlayer, SIGNAL(rayData(Pose*const,qint32,std::vector<quint16>*const)), mGlWindow, SLOT(slotNewRayData(Pose*const,qint32,std::vector<quint16>*const)));

        //connect(mLogPlayer, SIGNAL(vehiclePoseLowFreq(Pose)), mPtuController, SLOT(slotVehiclePoseChanged(Pose)));
        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mPtuController, SLOT(slotVehiclePoseChanged(const Pose* const)));

        connect(mLogPlayer, SIGNAL(vehiclePose(const Pose* const)), mGlWindow, SLOT(slotNewVehiclePose(const Pose* const)));
        connect(mLogPlayer, SIGNAL(scanData(float*const, quint32, QVector3D*const)), mFlightPlanner, SLOT(slotNewScanData(float*const,quint32,QVector3D*const)));
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
    delete mWirelessDevice;
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
    if(event->key() == Qt::Key_P)
        slotToggleViewPointCloudDense();
    else if(event->key() == Qt::Key_C)
        slotClearCloud();

    mGlWindow->keyPressEvent(event);
    mFlightPlanner->keyPressEvent(event);
    //if(mLogPlayer) mLogPlayer->keyPressEvent(event);
}

void BaseStation::slotClearCloud()
{
    mPointCloud->slotReset();
    mGlWindow->slotRenderNow();
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
    mGlWindow->slotSetFlightControllerValues(fcv);

    mControlWidget->slotFlightStateChanged(&fcv->flightState);

//    if(!mPidControllerWidget->isPopulated()) mPidControllerWidget->slotUpdateWeights();

    mPidControllerWidget->slotUpdateValues();
}
