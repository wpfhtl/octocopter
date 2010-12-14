#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud) : QObject()
{
}

FlightPlannerInterface::~FlightPlannerInterface()
{
}

void FlightPlannerInterface::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    mScanVolumeMin = min;
    mScanVolumeMax = max;
}
