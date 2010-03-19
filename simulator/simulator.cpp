#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) :
        QMainWindow(),
        mVehicle(0),
        mMutex(QMutex::NonRecursive)
{
    qDebug() << "Simulator::Simulator()";
    QMutexLocker locker(&mMutex);

    /* testing quats
    btQuaternion btq(deg2rad(0),deg2rad(10),deg2rad(0));
    btVector3 btv(0,0,0);
    Engine e(btTransform(btq, btv), this);
    btVector3 thrust = e.calculateThrust(10000);
    qDebug() << "qaxis:" << btq.getAxis().x() << btq.getAxis().y() << btq.getAxis().z();
    qDebug() << "qangle:" << btq.getAngle();
    qDebug() << "thrust:" << thrust.x() << thrust.y() << thrust.z();
    exit(0);
    */

    resize(1027, 900);

    mTimeSimulationPause = QTime(); // set invalid;
    mTimeSimulationStart = QTime(); // set invalid;

    mCoordinateConverter = new CoordinateConverter(CoordinateGps(53.5592, 9.83, 0));

    mLaserScanners = new QList<LaserScanner*>;

    mOgreWidget = new OgreWidget(this);
    mOgreWidget->setFocus();
    setCentralWidget(mOgreWidget);
    mOgreWidget->show();

    connect(mOgreWidget, SIGNAL(setupFinished()), SLOT(slotAddVehicle()));

    mSimulationControlWidget = new SimulationControlWidget(this, mOgreWidget);
    addDockWidget(Qt::RightDockWidgetArea, mSimulationControlWidget);
    connect(mSimulationControlWidget, SIGNAL(simulationStart()), SLOT(slotSimulationStart()));
    connect(mSimulationControlWidget, SIGNAL(simulationPause()), SLOT(slotSimulationPause()));
    connect(mSimulationControlWidget, SIGNAL(timeFactorChanged(double)), SLOT(slotSetTimeFactor(double)));

    mBattery = new Battery(this, 12.0, 4.0);
    mBattery->setDischargeCurrent(20.0);
    connect(mSimulationControlWidget, SIGNAL(timeFactorChanged(double)), mBattery, SLOT(slotSetTimeFactor(double)));

    mTimeFactor = mSimulationControlWidget->getTimeFactor();

    mStatusWidget = new StatusWidget(this, mBattery, mCoordinateConverter);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mOgreWidget, SIGNAL(currentRenderStatistics(QSize,int,float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));
}

void Simulator::slotAddVehicle(void)
{
        mVehicle = new Vehicle(this, mOgreWidget);

        connect(mVehicle, SIGNAL(newPose(const Ogre::Vector3&, const Ogre::Quaternion&)), mStatusWidget, SLOT(slotUpdatePose(const Ogre::Vector3&, const Ogre::Quaternion&)));

        qDebug() << "Simulator::Simulator(): starting vehicle thread.";
    //    mVehicle->start();

        // start a trip!
        CoordinateConverter cc;
    //    mVehicle->slotSetNextWayPoint(cc.convert(Ogre::Vector3(200, 200, -1000)));
}

void Simulator::slotSimulationStart(void)
{
    QMutexLocker locker(&mMutex);

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
        mLaserScanners->at(i)->slotStart();
}

void Simulator::slotSimulationPause(void)
{
    QMutexLocker locker(&mMutex);

    mTimeSimulationPause.start();
    mVehicle->stop();

    // Notify all laserscanners
    for(int i=0; i < mLaserScanners->size(); i++)
        mLaserScanners->at(i)->slotPause();
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
    else
    {
        // simulation is running.
//        qDebug() << "Simulator::getSimulationTime(): running, timeFactor " << mTimeFactor << "startElapsed" << mTimeSimulationStart.elapsed() << "result" << mTimeFactor * mTimeSimulationStart.elapsed();
        return mTimeFactor * mTimeSimulationStart.elapsed();
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

    // Notify all laserscanners of the new timeFactor, so they can adjust their speed.
    for(int i=0; i < mLaserScanners->size(); i++)
    {
        mLaserScanners->at(i)->setTimeFactor(mTimeFactor);
    }
}

float Simulator::getTimeFactor(void) const
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
