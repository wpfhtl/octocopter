#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) :
        QMainWindow(),
        mVehicle(0),
        mMutex(QMutex::NonRecursive)
{
    qDebug() << "Simulator::Simulator()";
    QMutexLocker locker(&mMutex);

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

    connect(mOgreWidget, SIGNAL(setupFinished()), SLOT(slotOgreInitialized()), Qt::QueuedConnection);

    mBattery = new Battery(this, 12.0, 4.0);
    mBattery->setDischargeCurrent(20.0);

    mStatusWidget = new StatusWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mStatusWidget, SIGNAL(simulationStart()), SLOT(slotSimulationStart()));
    connect(mStatusWidget, SIGNAL(simulationPause()), SLOT(slotSimulationPause()));
    connect(mStatusWidget, SIGNAL(timeFactorChanged(double)), SLOT(slotSetTimeFactor(double)));
    connect(mStatusWidget, SIGNAL(timeFactorChanged(double)), mBattery, SLOT(slotSetTimeFactor(double)));
    connect(mOgreWidget, SIGNAL(currentRenderStatistics(QSize,int,float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));

    mViewUpdateTimer = new QTimer(this);
    mViewUpdateTimer->setInterval(1000/60);
    connect(mViewUpdateTimer, SIGNAL(timeout()), mOgreWidget, SLOT(update()));

    mTimeFactor = mStatusWidget->getTimeFactor();
    qDebug() << "Simulator::Simulator(): setting timeFactor to" << mTimeFactor;

    mBaseConnection = new BaseConnection(this);
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
        delete scanner;
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
    mVehicle = new Vehicle(this, mOgreWidget);
    connect(mVehicle, SIGNAL(newPose(const Ogre::Vector3&, const Ogre::Quaternion&)), mStatusWidget, SLOT(slotUpdatePose(const Ogre::Vector3&, const Ogre::Quaternion&)));

    mBaseConnection->setVehicle(mVehicle);

    // Same thing for StatusWidget, which has the configuration window. Tell it to read the config only after ogre
    // is initialized, so it can create cameras and laserscanners.
    mStatusWidget->mDialogConfiguration->slotReadConfiguration();
}

void Simulator::slotSimulationStart(void)
{
    QMutexLocker locker(&mMutex);

    mStatusWidget->slotSetButtonPauseEnabled(true);
    mStatusWidget->slotSetButtonStartEnabled(false);

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
    mVehicle->start();

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

    // Set the timer to update mOgreWidget every 25th of a second.
    mViewUpdateTimer->start();
}

void Simulator::slotSimulationPause(void)
{
    QMutexLocker locker(&mMutex);

    mStatusWidget->slotSetButtonPauseEnabled(false);
    mStatusWidget->slotSetButtonStartEnabled(true);

    mTimeSimulationPause.start();
    mVehicle->stop();

    // Notify all laserscanners
    for(int i=0; i < mLaserScanners->size(); i++)
        QMetaObject::invokeMethod(mLaserScanners->at(i), "slotPause", Qt::QueuedConnection);

    // Notify all cameras
    // Cameras currently don't live in a separate thread.
    for(int i=0; i < mCameras->size(); i++)
        QMetaObject::invokeMethod(mCameras->at(i), "slotPause", Qt::QueuedConnection);

    // Stop the update timer for the GL view
    mViewUpdateTimer->stop();
}

bool Simulator::isPaused(void) const
{
    return mTimeSimulationPause.isValid() || !mTimeSimulationStart.isValid();
}

// returns scaled time since start in milliseconds
int Simulator::getSimulationTime(void) const
{
    QMutexLocker locker(&mMutex);

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
        qDebug() << "Simulator::getSimulationTime(): never started, returning 0";
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

QList<LaserScanner*>* Simulator::getLaserScannerList(void)
{
    return mLaserScanners;
}

void Simulator::slotScanFinished(QList<CoordinateGps>)
{
    // TODO: send scanner pose and scanData to network.
    // TODO: no, we need to send a structure holding at least the scanner's position at the beginning and at the end, and the scan itself.
    // TODO: no, we need to send world coordinates instead of simple long ints. Yeah.
}
