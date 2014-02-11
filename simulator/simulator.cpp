#include "simulator.h"
#include <QStatusBar>
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
    connect(mJoystick, &Joystick::buttonStateChanged, this, &Simulator::slotJoystickButtonChanged);
    mJoystickEnabled = true;

    connect(mOgreWidget, &OgreWidget::setupFinished, this, &Simulator::slotOgreInitialized);

    mBattery = new Battery(this, 16.8, 5.0);
    mBattery->charge();

    mUpdateTimer = new QTimer(this);
    connect(mUpdateTimer, &QTimer::timeout, this, &Simulator::slotUpdate);

    // Make it create logfiles!
    mFlightController = new FlightController(QString("simulator-%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd-HHmmsszzz")));
    QMap<QChar,float> weights;
    weights.insert('p', 15);
    weights.insert('i', 0);
    weights.insert('d', 1);
    QString controllerP("pitch");
    QString controllerR("roll");
    mFlightController->slotSetControllerWeights(&controllerP, &weights);
    mFlightController->slotSetControllerWeights(&controllerR, &weights);
    mFlightController->slotSetFlightSpeed(2);

    mBaseConnection = new BaseConnection("eth0", "simulator");
    connect(mBaseConnection, &BaseConnection::wayPoints, mFlightController, &FlightController::slotSetWayPoints);
    connect(mBaseConnection, &BaseConnection::controllerWeights, mFlightController, &FlightController::slotSetControllerWeights);
    connect(mBaseConnection, &BaseConnection::newConnection, this, &Simulator::slotNewConnection);

    connect(mFlightController, &FlightController::wayPointReached, mBaseConnection, &BaseConnection::slotSendWayPointReached);
    connect(mFlightController, &FlightController::wayPoints, mBaseConnection, &BaseConnection::slotSendWayPoints);
    connect(mFlightController, &FlightController::flightState, mBaseConnection, &BaseConnection::slotSendFlightState);
    connect(mFlightController, &FlightController::flightStateRestriction, mBaseConnection, &BaseConnection::slotSendFlightStateRestriction);
    connect(mFlightController, &FlightController::flightControllerValues, mBaseConnection, &BaseConnection::slotSendFlightControllerValues);

    mStatusWidget = new StatusWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mStatusWidget, &StatusWidget::simulationStart, this, &Simulator::slotSimulationStart);
    connect(mStatusWidget, &StatusWidget::simulationPause, this, &Simulator::slotSimulationPause);
    connect(mStatusWidget, &StatusWidget::timeFactorChanged, this, &Simulator::slotSetTimeFactor);
    connect(mOgreWidget, &OgreWidget::currentRenderStatistics, mStatusWidget, &StatusWidget::slotUpdateVisualization);

    mTimeFactor = mStatusWidget->getTimeFactor();
    qDebug() << "Simulator::Simulator(): setting timeFactor to" << mTimeFactor;
}

Simulator::~Simulator(void)
{
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

    delete mFlightController;
    delete mBaseConnection;
    delete mStatusWidget;
    delete mJoystick;
    delete mOgreWidget;

    qDeleteAll(*mCameras);
    delete mCameras;
}

void Simulator::keyPressEvent(QKeyEvent * event)
{
    if(mPhysics != 0 && event->key() == Qt::Key_R)
    {
        mPhysics->slotRescueVehicle();
        mOgreWidget->update();
    }
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
    connect(mPhysics, &Physics::newVehiclePose, mStatusWidget, &StatusWidget::slotUpdatePose);
    connect(mPhysics, &Physics::newVehiclePose, mFlightController, &FlightController::slotNewVehiclePose);

    connect(mStatusWidget, &StatusWidget::windDetailChanged, mPhysics, &Physics::slotSetWindSetting);

    // Only one of the two objects will emit motion signals depending on mJoystickEnabled
    // FlightController's motion should go through a clampToSafeLimits, while the joystickmotion is sent directly.
    connect(mFlightController, &FlightController::motion, mPhysics, &Physics::slotSetMotion);
    connect(mJoystick, &Joystick::motion, mPhysics, &Physics::slotSetMotion);

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

    if(mFlightController->getFlightState() == FlightState::State::Hover || mFlightController->getFlightState() == FlightState::State::ApproachWayPoint)
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

    case 1:
    {
        // thats actually button 2 on the joystick
        mPhysics->slotRescueVehicle();
        break;
    }

    case 6:
    {
        // coolie->right, idle?! nothing yet.
        break;
    }

    case 7:
    {
        // coolie->left, Hover
        if(mFlightStateRestriction.restriction != FlightStateRestriction::Restriction::RestrictionHover)
        {
            slotShowMessage("Coolie left, setting FlightStateRestriction to Hover");
            mFlightStateRestriction.restriction = FlightStateRestriction::Restriction::RestrictionHover;
            mFlightController->slotFlightStateRestrictionChanged(&mFlightStateRestriction);
            mBaseConnection->slotSendFlightStateRestriction(&mFlightStateRestriction);
        }
        mJoystickEnabled = false;
        break;
    }

    case 8:
    {
        // coolie->down, UserControl
        if(mFlightStateRestriction.restriction != FlightStateRestriction::Restriction::RestrictionUserControl)
        {
            slotShowMessage("Coolie down, setting FlightStateRestriction to UserControl");
            mFlightStateRestriction.restriction = FlightStateRestriction::Restriction::RestrictionUserControl;
            mFlightController->slotFlightStateRestrictionChanged(&mFlightStateRestriction);
            mBaseConnection->slotSendFlightStateRestriction(&mFlightStateRestriction);
        }
        mJoystickEnabled = true;
        break;
    }

    case 9:
    {
        // coolie->up, ApproachWayPoint
        if(mFlightStateRestriction.restriction != FlightStateRestriction::Restriction::RestrictionNone)
        {
            slotShowMessage("Coolie up, setting FlightStateRestriction to None");
            mFlightStateRestriction.restriction = FlightStateRestriction::Restriction::RestrictionNone;
            mFlightController->slotFlightStateRestrictionChanged(&mFlightStateRestriction);
            mBaseConnection->slotSendFlightStateRestriction(&mFlightStateRestriction);
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
        mBaseConnection->slotSendVehicleStatus(&mVehicleStatus);
        mBaseConnection->slotSendVehiclePose(mFlightController->getLastKnownPose());
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
    mFlightController->slotEmitFlightControllerInfo();
    mBaseConnection->slotSendFlightStateRestriction(&mFlightStateRestriction);
    mBaseConnection->slotSendVehicleStatus(&mVehicleStatus);
}

/*void Simulator::slotSetClampedMotion(const MotionCommand* const mc)
{
    const MotionCommand mcClamped = mc->clampedToSafeLimits();
    mPhysics->slotSetMotion(&mcClamped);
}*/
