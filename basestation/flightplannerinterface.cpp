#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(const Pose * const pose, Octree* pointCloud) : QObject()
{
    mVehiclePose = pose;
    qDebug() << "FlightPlannerInterface c'tor.";
}

FlightPlannerInterface::~FlightPlannerInterface()
{
}

void FlightPlannerInterface::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    mScanVolumeMin = min;
    mScanVolumeMax = max;
}

void FlightPlannerInterface::sortToShortestPath(QVector<QVector3D> &wayPoints, const QVector3D &currentVehiclePosition)
{
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): vehicle is at" << currentVehiclePosition;

    float distanceBefore = 0;
    for(int i=1;i<wayPoints.size();i++) distanceBefore += wayPoints.at(i-1).distanceToLine(wayPoints.at(i), QVector3D());
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points before:" << distanceBefore;

    QVector<QVector3D> wps(wayPoints);
    float distanceBeforewps = 0;
    for(int i=1;i<wps.size();i++) distanceBeforewps += wps.at(i-1).distanceToLine(wps.at(i), QVector3D());
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): wps total distance between" << wps.size() << "points before:" << distanceBeforewps;

    wayPoints.clear();
    wayPoints.append(currentVehiclePosition);

    while(wps.size())
    {
        float closestNeighborDistance = INFINITY;
        int indexOfClosestNeighbor = -1;

        for(int i=0;i<wps.size();i++)
        {
            const float currentDistance = wayPoints.last().distanceToLine(wps.at(i), QVector3D());
            if(currentDistance < closestNeighborDistance)
            {
                closestNeighborDistance = currentDistance;
                indexOfClosestNeighbor = i;
            }
        }

        wayPoints.append(wps.at(indexOfClosestNeighbor));
        wps.remove(indexOfClosestNeighbor);
    }

    wayPoints.remove(0);

    float distanceAfter = 0;
    for(int i=1;i<wayPoints.size();i++) distanceAfter += wayPoints.at(i-1).distanceToLine(wayPoints.at(i), QVector3D());
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points after:" << distanceAfter;
}

const Pose FlightPlannerInterface::getVehiclePose(void) const
{
    return *mVehiclePose;
}
