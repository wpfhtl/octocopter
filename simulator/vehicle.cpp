#include "vehicle.h"

Vehicle::Vehicle(Simulator *simulator, Ogre::SceneNode *vehicle) :  QThread(simulator)
{
    mSimulator = simulator;
    mVehicle = vehicle;

    mTimerUpdatePosition = new QTimer;
    mTimerUpdatePosition->setInterval(25);
    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePosition()));
}

void Vehicle::run()
{
    mTimeOfLastUpdate = mSimulator->getSimulationTime();
    mTimerUpdatePosition->start();
    QThread::exec();
}

void Vehicle::slotUpdatePosition(void)
{
    // Impress by interpolating linearly :)
    // TODO: get busy with ODE/Bullet/PhysX

    // that'd be meters/second
    const float movementSpeed = 0.1;
    const float maxDistanceToTravel = movementSpeed * ((mSimulator->getSimulationTime() - mTimeOfLastUpdate) / 1000.0f);
    mTimeOfLastUpdate = mSimulator->getSimulationTime();
    Ogre::Vector3 currentPosition = mVehicle->getPosition();

    // how far to the current wayPoint?
    float distX = mNextWayPoint.x - currentPosition.x;
    float distY = mNextWayPoint.y - currentPosition.y;
    float distZ = mNextWayPoint.z - currentPosition.z;

    // cap at maxDistanceToTravel
    distX > 0.0f ? distX = fmin(distX, maxDistanceToTravel) : distX = fmax(distX, -maxDistanceToTravel);
    distY > 0.0f ? distY = fmin(distY, maxDistanceToTravel) : distY = fmax(distY, -maxDistanceToTravel);
    distZ > 0.0f ? distZ = fmin(distZ, maxDistanceToTravel) : distZ = fmax(distZ, -maxDistanceToTravel);

    // update position
    currentPosition.x += distX;
    currentPosition.y += distY;
    currentPosition.z += distZ;

    mVehicle->setPosition(currentPosition);
}

void Vehicle::slotSetNextWayPoint(const CoordinateGps &wayPoint)
{
    mNextWayPoint = mCoordinateConverter.convert(wayPoint);
}

void Vehicle::slotShutDown(void)
{
    quit();
}
