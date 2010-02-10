#include "laserscannermanager.h"

LaserScannerManager::LaserScannerManager(Simulator *simulator) :
    QObject(parent)
{
    mSimulator = simulator;
}
