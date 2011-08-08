#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(QWidget* widget, Octree* pointCloud) : QObject()
{
    mParentWidget = widget;
//    mVehiclePose = pose;
    mWayPointsAhead = new QList<WayPoint>;
    mWayPointsPassed = new QList<WayPoint>;
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

void FlightPlannerInterface::sortToShortestPath(QList<WayPoint> &wayPoints, const QVector3D &currentVehiclePosition)
{
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): vehicle is at" << currentVehiclePosition;

    float distanceBefore = 0;
    for(int i=1;i<wayPoints.size();i++) distanceBefore += wayPoints.at(i-1).distanceToLine(wayPoints.at(i), QVector3D());
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points before:" << distanceBefore;

    QList<WayPoint> wps(wayPoints);
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
        wps.removeAt(indexOfClosestNeighbor);
    }

    wayPoints.takeFirst();

    float distanceAfter = 0;
    for(int i=1;i<wayPoints.size();i++) distanceAfter += wayPoints.at(i-1).distanceToLine(wayPoints.at(i), QVector3D());
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points after:" << distanceAfter;
}

const Pose FlightPlannerInterface::getLastKnownVehiclePose(void) const
{
    if(mVehiclePoses.size())
        return mVehiclePoses.last();
    else
        return Pose();
}

void FlightPlannerInterface::slotVehiclePoseChanged(const Pose& pose)
{
    mVehiclePoses.append(pose);
    if(mVehiclePoses.size() > 2) mVehiclePoses.takeFirst();
}

const QVector3D FlightPlannerInterface::getCurrentVehicleVelocity() const
{
    if(mVehiclePoses.size() < 2) return QVector3D();

    const quint32 timeDiffMs = mVehiclePoses.last().timestamp - mVehiclePoses.first().timestamp;
    return (mVehiclePoses.last().position - mVehiclePoses.first().position) * (1000 / timeDiffMs);
}

void FlightPlannerInterface::slotWayPointDelete(const quint16& index)
{
    if(mWayPointsAhead->size() <= index)
    {
        qWarning() << "FlightPlannerInterface::slotWayPointDelete(): cannot delete waypoint at index" << index << ", size is only" << mWayPointsAhead->size();
        return;
    }

    mWayPointsAhead->removeAt(index);
    emit wayPointDeleted(index);
    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointInsert(const quint16& index, const WayPoint& wpt)
{
    if(index > mWayPointsAhead->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointInsert(): cannot delete waypoint at index" << index << ", size is only" << mWayPointsAhead->size();
        return;
    }

    mWayPointsAhead->insert(index, wpt);
    emit wayPointInserted(index, wpt);
    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointSwap(const quint16& i, const quint16& j)
{
    if(mWayPointsAhead->size() <= i || mWayPointsAhead->size() <= j)
    {
        qWarning() << "FlightPlannerInterface::slotWayPointSwap(): cannot swap waypoints at index" << i << "and" << j <<", size is only" << mWayPointsAhead->size();
        return;
    }
    qDebug() << "FlightPlannerInterface::slotWayPointSwap(): swapping waypoints at index" << i << "and" << j <<", size is" << mWayPointsAhead->size();

    mWayPointsAhead->swap(i,j);

    emit wayPointDeleted(j);
    emit wayPointInserted(j, mWayPointsAhead->at(j));
    emit wayPointDeleted(i);
    emit wayPointInserted(i, mWayPointsAhead->at(i));

    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointsClear()
{
    mWayPointsAhead->clear();
    emit wayPoints(*mWayPointsAhead);
    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointReached(const WayPoint)
{
    if(!mWayPointsAhead->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointReached(): mWayPointsAhead is empty, how can you reach a waypoint?";
        return;
    }

    mWayPointsPassed->append(mWayPointsAhead->takeAt(0));
    emit wayPointDeleted(0);
    emit suggestVisualization();
}

const QList<WayPoint> FlightPlannerInterface::getWayPoints()
{
    return *mWayPointsAhead;
}

void FlightPlannerInterface::getScanVolume(QVector3D& min, QVector3D& max)
{
    min = mScanVolumeMin;
    max = mScanVolumeMax;
}
