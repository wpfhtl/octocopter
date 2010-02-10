#include "simulator.h"

#include <cstdlib>

Simulator::Simulator(void) :
        QMainWindow(),
        mTimeFactor(1.0f),
        mMutex(QMutex::NonRecursive)
{
    QMutexLocker locker(&mMutex);

    mLaserScanners = new QList<LaserScanner*>;

    mOgreWidget = new OgreWidget(this);
    mOgreWidget->setFocus();
    setCentralWidget(mOgreWidget);
    mOgreWidget->show();

    mBattery = new Battery(this, 12.0, 4.0);
    mBattery->setDischargeCurrent(20.0);

    mVehicle = new Vehicle(this, mOgreWidget->getVehicleNode());

    mStatusWidget = new StatusWidget(this, mBattery);
    addDockWidget(Qt::RightDockWidgetArea, mStatusWidget);
    connect(mOgreWidget, SIGNAL(currentRenderStatistics(QSize,int,float)), mStatusWidget, SLOT(slotUpdateVisualization(QSize, int, float)));

    mSimulationControlWidget = new SimulationControlWidget(this, mOgreWidget);
    connect(mSimulationControlWidget, SIGNAL(start()), SLOT(slotStartSimulation()));
    connect(mSimulationControlWidget, SIGNAL(timeFactorChanged(double)), SLOT(slotSetTimeFactor(double)));
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
