#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) :
        QMainWindow(),
        mPhysics(0),
        mMutex(QMutex::NonRecursive)
{
    qDebug() << "Simulator::Simulator()";

    // We need a small font.
    QFont widgetFont = QApplication::font();
    widgetFont.setPointSize(7);
    QApplication::setFont(widgetFont);

    mClockDivisorBaseConnectionUpdate = 0;

    resize(1024, 768);

    slotShowMessage("Starting up...");

    mTimeSimulationPause = QTime(); // set invalid;
    mTimeSimulationStart = QTime(); // set invalid;

    mCoordinateConverter = new CoordinateConverter();

    mLaserScanners = new QList<LaserScanner*>;
    mCameras = new QList<Camera*>;

    mOgreWidget = new OgreWidget(this);
    mOgreWidget->setFocus();
    setCentralWidget(mOgreWidget);
    mOgreWidget->show();

    mJoystick = new Joystick;
    if(!mJoystick->isValid()) QMessageBox::warning(this, "Joystick not found", "Joystick initialization failed, using manual control will be impossible.");
    connect(mJoystick, SIGNAL(buttonStateChanged(quint8,bool)), SLOT(slotJoystickButtonChanged(quint8, bool)));
    mJoystickEnabled = true;

    connect(mOgreWidget, SIGNAL(setupFinished()), SLOT(slotOgreInitialized()), Qt::QueuedConnection);

    mBattery = new Battery(this, 16.8, 5.0);
    mBattery->charge();

    mUpdateTimer = new QTimer(this);
    connect(mUpdateTimer, SIGNAL(timeout()), SLOT(slotUpdate()));

    // Make it create logfiles!
    mFlightController = new FlightController(QString("simulator-%1-").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd-HHmmsszzz")));

    mBaseConnection = new BaseConnection("eth0");
    connect(mBaseConnection, SIGNAL(wayPointInsert(quint16,WayPoint)), mFlightController, SLOT(slotWayPointInsert(quint16,WayPoint)));
    connect(mBaseConnection, SIGNAL(wayPointDelete(quint16)), mFlightController, SLOT(slotWayPointDelete(quint16)));
    connect(mBaseConnection, SIGNAL(wayPoints(QList<WayPoint>)), mFlightController, SLOT(slotSetWayPoints(QList<WayPoint>)));
    connect(mBaseConnection, SIGNAL(controllerWeights(const QString* const,const QMap<QString,float>* const)), mFlightController, SLOT(slotSetControllerWeights(const QString* const, const QMap<QString,float>* const)));
    connect(mBaseConnection, SIGNAL(newConnection()), SLOT(slotNewConnection()));

    connect(mFlightController, SIGNAL(wayPointReached(WayPoint)), mBaseConnection, SLOT(slotWayPointReached(WayPoint)));
    connect(mFlightController, SIGNAL(wayPointInserted(quint16,WayPoint)), mBaseConnection, SLOT(slotRoverWayPointInserted(quint16,WayPoint)));
    connect(mFlightController, SIGNAL(currentWayPoints(const QList<WayPoint>* const)), mBaseConnection, SLOT(slotFlightControllerWayPointsChanged(const QList<WayPoint>*const)));
    connect(mFlightController, SIGNAL(flightStateChanged(FlightState*const)), mBaseConnection, SLOT(slotFlightStateChanged(FlightState*const)));
    connect(mFlightController, SIGNAL(flightControllerValues(const FlightControllerValues* const)), mBaseConnection, SLOT(slotNewFlightControllerValues(const FlightControllerValues* const)));

    mStatusWidget = new StatusWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mStatusWidget, SIGNAL(simulationStart()), SLOT(slotSimulationStart()));
    connect(mStatusWidget, SIGNAL(simulationPause()), SLOT(slotSimulationPause()));
    connect(mStatusWidget, SIGNAL(timeFactorChanged(double)), SLOT(slotSetTimeFactor(double)));
    connect(mOgreWidget, SIGNAL(currentRenderStatistics(QSize,int,float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));

    mTimeFactor = mStatusWidget->getTimeFactor();
    qDebug() << "Simulator::Simulator(): setting timeFactor to" << mTimeFactor;
}

Simulator::~Simulator(void)
{
    delete mStatusWidget;

    // Delete all laserscanners
    while(mLaserScanners->size())
    {
        qDebug() << "Simulator::~Simulator(void): shutting down laserscanners, please wait.";
        LaserScanner* scanner = mLaserScanners->takeFirst();
        scanner->quit();
        scanner->wait();
        scanner->deleteLater();
    }
    delete mLaserScanners;

    qDeleteAll(*mCameras);
    delete mCameras;
}

void Simulator::slotShowMessage(const QString message)
{
    statusBar()->showMessage(message);
}

void Simulator::slotOgreInitialized(void)
{
    // Vehicle creates SceneNode and Entity in OgreWidget. But OgreWidget is not initialized after instatiation, but only when
    // the window is initialized. Thus, vehicle needs to be created after ogreWidget initialization.
    mPhysics = new Physics(this, mOgreWidget);
    connect(mPhysics, SIGNAL(newVehiclePose(const Pose* const)), mStatusWidget, SLOT(slotUpdatePose(const Pose* const)));
    connect(mPhysics, SIGNAL(newVehiclePose(const Pose* const)), mFlightController, SLOT(slotNewVehiclePose(const Pose* const)));

    connect(mStatusWidget, SIGNAL(windDetailChanged(bool,float)), mPhysics, SLOT(slotSetWindSetting(bool, float)));

    // Only one of the two objects will emit motion signals depending on mJoystickEnabled
    connect(mFlightController, SIGNAL(motion(const MotionCommand* const)), mPhysics, SLOT(slotSetMotion(const MotionCommand* const)));
    connect(mJoystick, SIGNAL(motion(const MotionCommand* const)), mPhysics, SLOT(slotSetMotion(const MotionCommand* const)));

    // Same thing for StatusWidget, which has the configuration window. Tell it to read the config only after ogre
    // is initialized, so it can create cameras and laserscanners.
    mStatusWidget->mDialogConfiguration->slotReadConfiguration();
}

void Simulator::slotSimulationStart(void)
{
    QMutexLocker locker(&mMutex);

    mStatusWidget->slotSetButtonPauseEnabled(true);
    mStatusWidget->slotSetButtonStartEnabled(false);

    mBattery->slotStart();

    if(mTimeSimulationPause.isValid())
    {
        // simulation was paused, resume now.
        mTimeSimulationStart = mTimeSimulationStart.addMSecs(mTimeSimulationPause.elapsed());
    }
    else
    {
        // simulation wasn't paused, start it.
        mTimeSimulationStart.start();
    }

    mTimeSimulationPause = QTime(); // invalidate;
//    mVehicle->start();

    // Notify all laserscanners
    for(int i=0; i < mLaserScanners->size(); i++)
    {
        qDebug() << "Simulator::slotSimulationStart(): queue-starting scanner from thread" << thread()->currentThreadId();
        QMetaObject::invokeMethod(mLaserScanners->at(i), "slotStart", Qt::QueuedConnection);
    }

    // Notify all cameras
    for(int i=0; i < mCameras->size(); i++)
    {
        // Cameras currently don't live in a separate thread.
        qDebug() << "Simulator::slotSimulationStart(): queue-starting camera from thread" << thread()->currentThreadId();
        QMetaObject::invokeMethod(mCameras->at(i), "slotStart", Qt::QueuedConnection);
    }

    if(mFlightController->getFlightState() == FlightState::Value::Hover || mFlightController->getFlightState() == FlightState::Value::ApproachWayPoint)
    mFlightController->slotSetPause(false);

    // Set the timer to update mOgreWidget every 25th of a second.
    mUpdateTimer->start(1000/30);
}

void Simulator::slotSimulationPause(void)
{
    QMutexLocker locker(&mMutex);

    mStatusWidget->slotSetButtonPauseEnabled(false);
    mStatusWidget->slotSetButtonStartEnabled(true);

    mBattery->slotPause();

    mTimeSimulationPause.start();
//    mVehicle->stop();

    // Notify all laserscanners
    for(int i=0; i < mLaserScanners->size(); i++)
        QMetaObject::invokeMethod(mLaserScanners->at(i), "slotPause", Qt::QueuedConnection);

    // Notify all cameras
    // Cameras currently don't live in a separate thread.
    for(int i=0; i < mCameras->size(); i++)
        QMetaObject::invokeMethod(mCameras->at(i), "slotPause", Qt::QueuedConnection);

    mFlightController->slotSetPause(true);

    // Stop the update timer for the GL view
    mUpdateTimer->stop();
}

bool Simulator::isPaused(void) const
{
    return mTimeSimulationPause.isValid() || !mTimeSimulationStart.isValid();
}

// returns scaled time since start in milliseconds
quint32 Simulator::getSimulationTime(void) const
{
    // We need no mutex-locking, as this method does only const operations. Hopefully.
//    QMutexLocker locker(&mMutex);

    if(mTimeSimulationPause.isValid())
    {
        // simulation is paused.
//        qDebug() << "Simulator::getSimulationTime(): paused, timeFactor " << mTimeFactor << "startElapsed" << mTimeSimulationStart.elapsed() << "pauseElapsed:" << mTimeSimulationPause.elapsed() << "result" << mTimeFactor * (mTimeSimulationStart.elapsed() - mTimeSimulationPause.elapsed());
        return mTimeFactor * (mTimeSimulationStart.elapsed() - mTimeSimulationPause.elapsed());
    }
    else if(mTimeSimulationStart.isValid())
    {
        // simulation is running.
//        qDebug() << "Simulator::getSimulationTime(): running, timeFactor " << mTimeFactor << "startElapsed" << mTimeSimulationStart.elapsed() << "result" << mTimeFactor * mTimeSimulationStart.elapsed();
        return mTimeFactor * mTimeSimulationStart.elapsed();
    }
    else
    {
        // both invalid, has never run before
//        qDebug() << "Simulator::getSimulationTime(): never started, returning 0";
        return 0;
    }
}

void Simulator::slotSetTimeFactor(double timeFactor)
{
    QMutexLocker locker(&mMutex);

    if(!mTimeSimulationStart.isValid())
    {
        // The simulation hasn't started yet. Simply adjust the factor and quit
        qDebug() << "Simulator::slotSetTimeFactor(): sim never started, setting timeFactor from" << mTimeFactor << "to" << timeFactor;
        mTimeFactor = timeFactor;
        slotNotifyDevicesOfNewTimeFactor();
        return;
    }

    if(fabs(timeFactor) < 0.00001)
    {
        // timeFactor is getting real close to zero, its hopefully just an intermediate state in the spinbox, ignore.
        qDebug() << "Simulator::slotSetTimeFactor(): ignoring value of" << timeFactor;
        return;
    }

    locker.unlock();
    const int scaledElapsedTime = getSimulationTime();
    locker.relock();

    // Setting a new timefactor means that we also have to change
    // mTimeSimulationStart (and -Pause, when simulation is currently paused)
    // to prevent discontinuities in time when changing the factor. After the
    // factor is changed, elapsed() time should be the same *AFTER* scaling
    // with mTimeFactor. Whew. And from then on, it'll just advance
    // quicker/slower.

    qDebug() << "Simulator::slotSetTimeFactor(): setting timeFactor from" << mTimeFactor << "to" << timeFactor << "simTime:" << scaledElapsedTime;

    if(mTimeSimulationPause.isValid())
    {
        // simulation is paused, so we need to scale both, too!
        qDebug() << "Simulator::slotSetTimeFactor(): resetting paused time";
        mTimeSimulationPause.start();
//        qDebug() << "Simulator::slotSetTimeFactor(): scaling paused time b" << mTimeSimulationPause.elapsed();
    }

    // move the starttime, so that the scaled elapsed value is the same as before the adjustment of timeFactor.
    qDebug() << "Simulator::slotSetTimeFactor(): scaling started time a" << mTimeSimulationStart.elapsed();
    mTimeSimulationStart = QTime::currentTime().addMSecs(-scaledElapsedTime / timeFactor);
    qDebug() << "Simulator::slotSetTimeFactor(): scaling started time b" << mTimeSimulationStart.elapsed();

    mTimeFactor = timeFactor;

    locker.unlock();
    qDebug() << "Simulator::slotSetTimeFactor(): done, simTime:" << getSimulationTime() << "diff" << getSimulationTime() - scaledElapsedTime;
    Q_ASSERT(abs(getSimulationTime() - scaledElapsedTime) < 20);
    locker.relock();

    slotNotifyDevicesOfNewTimeFactor();
}

void Simulator::slotNotifyDevicesOfNewTimeFactor()
{
    // Notify all laserscanners of the new timeFactor, so they can adjust their speed.
    for(int i=0; i < mLaserScanners->size(); i++)
        mLaserScanners->at(i)->slotSetTimeFactor(mTimeFactor);

    for(int i=0; i < mCameras->size(); i++)
        mCameras->at(i)->slotSetTimeFactor(mTimeFactor);
}

double Simulator::getTimeFactor(void) const
{
    return mTimeFactor;
}

void Simulator::slotJoystickButtonChanged(const quint8& button, const bool& enabled)
{
//    qDebug() << "Simulator::slotJoystickButtonChanged(): button" << button << ":" << enabled;
    // This slot is called when pressing and releasing a button, so only act once.
    if(!enabled) return;

    switch(button)
    {

    case 6:
    {
        // coolie->right, idle?! nothing yet.
        break;
    }

    case 7:
    {
        // coolie->left, Hover
        slotShowMessage("Disabling Joystick control, setting FlightController to Hover");
        if(mFlightController->getFlightState().state != FlightState::Value::Hover)
        {
            mFlightStateSwitch.value = FlightStateSwitch::Value::Hover;
            mFlightController->slotFlightStateSwitchValueChanged(&mFlightStateSwitch);
        }
        mJoystickEnabled = false;
        break;
    }

    case 8:
    {
        // coolie->down, UserControl
        slotShowMessage("Enabling Joystick control, setting FlightController to UserControl");
        if(mFlightController->getFlightState().state != FlightState::Value::UserControl)
        {
            mFlightStateSwitch.value = FlightStateSwitch::Value::UserControl;
            mFlightController->slotFlightStateSwitchValueChanged(&mFlightStateSwitch);
        }
        mJoystickEnabled = true;
        break;
    }

    case 9:
    {
        // coolie->up, ApproachWayPoint
        slotShowMessage("Disabling Joystick control, setting FlightController to ApproachWayPoint");
        if(mFlightController->getFlightState().state != FlightState::Value::ApproachWayPoint)
        {
            mFlightStateSwitch.value = FlightStateSwitch::Value::ApproachWayPoint;
            mFlightController->slotFlightStateSwitchValueChanged(&mFlightStateSwitch);
        }
        mJoystickEnabled = false;
        break;
    }

    default:
    {
        if(mUpdateTimer->isActive())
        {
            slotShowMessage("Pausing simulation due to Joystick request.");
            slotSimulationPause();
        }
        else
        {
            slotShowMessage("Starting simulation due to Joystick request.");
            slotSimulationStart();
        }
    }

    }
}

void Simulator::slotUpdate()
{
    if(mJoystickEnabled && mJoystick->isValid())
    {
        mJoystick->slotEmitMotionCommands();
    }
    else
    {
        // This will compute and emit motion commands, which are then used by vehicle.
//        mFlightController->slotComputeMotionCommands();
    }

    // Do the physics. This will move the vehicle, which will make the vehicle'smotion
    // state emit its new position, which will go into flightcontroller. Sweet!
    mPhysics->slotUpdatePhysics();

//    mFlightController->slotNewVehiclePose(mPhysics->getVehiclePose());

    mOgreWidget->slotVisualizeTrajectory(
                mFlightController->getLastKnownPose()->getPosition(),
                mFlightController->getWayPoints()
                );

    // Give baseconnection 4Hz updates about pose and vehicle status
    mClockDivisorBaseConnectionUpdate++;
    if(mClockDivisorBaseConnectionUpdate % ((1000/4) / mUpdateTimer->interval()))
    {
        mVehicleStatus.missionRunTime = getSimulationTime();
        mVehicleStatus.barometricHeight = mFlightController->getLastKnownPose()->getPosition().y();
        mVehicleStatus.batteryVoltage = mBattery->voltageCurrent();
        mBaseConnection->slotNewVehicleStatus(&mVehicleStatus);
        mBaseConnection->slotNewVehiclePose(mFlightController->getLastKnownPose());
    }

    // At last, re-render
    mOgreWidget->update();
}

QList<LaserScanner*>* Simulator::getLaserScannerList(void)
{
    return mLaserScanners;
}

void Simulator::slotNewConnection()
{
    // feed new data to basestation
    mBaseConnection->slotFlightStateChanged(&mFlightController->getFlightState());

    mBaseConnection->slotNewVehicleStatus(&mVehicleStatus);
    mBaseConnection->slotNewVehiclePose(mFlightController->getLastKnownPose());
}
