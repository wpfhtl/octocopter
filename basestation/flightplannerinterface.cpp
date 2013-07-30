#include "glwindow.h"
#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(BaseStation *basestation, GlWindow* glWidget, PointCloud *pointcloud) : QObject()
{


    qDebug() << "FlightPlannerInterface c'tor.";
}

FlightPlannerInterface::~FlightPlannerInterface()
{

}


/*
const QVector3D FlightPlannerInterface::getCurrentVehicleVelocity() const
{
    if(mVehiclePoses.size() < 2) return QVector3D();

    const Pose& last = mVehiclePoses.last();
    const Pose& secondLast = mVehiclePoses.at(mVehiclePoses.size() - 2);

    const quint32 timeDiffMs = last.timestamp - secondLast.timestamp;

    if(timeDiffMs == 0)
        return QVector3D();
    else
        return (last.getPosition() - secondLast.getPosition()) * (1000 / timeDiffMs);
}*/

/*
void FlightPlannerInterface::slotWayPointDelete(const quint16& index)
{
    if(mWaypointListMap.value("ahead")->size() <= index)
    {
        qWarning() << "FlightPlannerInterface::slotWayPointDelete(): cannot delete waypoint at index" << index << ", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }

    mWaypointListMap["ahead"]->remove(index);
    emit wayPointDeleted(index);
    emit wayPointDeleteOnRover(index);
    emit suggestVisualization();
}

// Called when the UI inserted a WPT. Tell the rover!
void FlightPlannerInterface::slotWayPointInsert(const quint16& index, const WayPoint& wpt)
{
    if(index > mWaypointListMap.value("ahead")->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointInsert(): cannot delete waypoint at index" << index << ", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }

    mWaypointListMap["ahead"]->insert(index, wpt);
    emit wayPointInserted(index, wpt);
    emit wayPointInsertOnRover(index, wpt);
    emit suggestVisualization();
}*/

//void FlightPlannerInterface::getScanVolume(QVector3D& min, QVector3D& max)
//{
//    min = mScanVolumeMin;
//    max = mScanVolumeMax;
//}


