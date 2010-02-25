#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) :
        QMainWindow(),
        mTimeFactor(10.0f),
        mMutex(QMutex::NonRecursive)
{
    qDebug() << "Simulator::Simulator()";
    QMutexLocker locker(&mMutex);

    resize(1027, 768);

    mLaserScanners = new QList<LaserScanner*>;

    mOgreWidget = new OgreWidget(this);
    mOgreWidget->setFocus();
    setCentralWidget(mOgreWidget);
    mOgreWidget->show();

    mBattery = new Battery(this, 12.0, 4.0);
    mBattery->setDischargeCurrent(20.0);

    mSimulationControlWidget = new SimulationControlWidget(this, mOgreWidget);
    addDockWidget(Qt::RightDockWidgetArea, mSimulationControlWidget);
    connect(mSimulationControlWidget, SIGNAL(start()), SLOT(slotStartSimulation()));
    connect(mSimulationControlWidget, SIGNAL(timeFactorChanged(double)), SLOT(slotSetTimeFactor(double)));

    mStatusWidget = new StatusWidget(this, mBattery);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mOgreWidget, SIGNAL(currentRenderStatistics(QSize,int,float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));

    mVehicle = new Vehicle(this, mOgreWidget);

    qDebug() << "Simulator::Simulator(): starting vehicle thread.";
//    mVehicle->start();

    // start a trip!
    CoordinateConverter cc;
    mVehicle->slotSetNextWayPoint(cc.convert(Ogre::Vector3(200, 200, -1000)));
}

void Simulator::slotStartSimulation(void)
{
    QMutexLocker locker(&mMutex);
    mTimeSimulationStart = QTime::currentTime();
}

void Simulator::slotSetTimeFactor(double timeFactor)
{
    QMutexLocker locker(&mMutex);
    mTimeFactor = timeFactor;

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

int Simulator::getSimulationTime(void) const
{
    QMutexLocker locker(&mMutex);
    return mTimeSimulationStart.msecsTo(QTime::currentTime()) * mTimeFactor;
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
