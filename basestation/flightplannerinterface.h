#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QObject>
#include "octree.h"
#include "glwidget.h"
#include <waypoint.h>

class Pose;

class FlightPlannerInterface : public QObject
{
    Q_OBJECT
private:

protected:
    QVector3D mScanVolumeMin, mScanVolumeMax;
    QList<Pose> mVehiclePoses;
    QWidget* mParentWidget;
    QList<WayPoint>* mWayPointsAhead, *mWayPointsPassed;

    static void sortToShortestPath(QList<WayPoint> &wayPointsSetOnRover, const QVector3D &currentVehiclePosition);

public:
    // Here, basestation passes its own octree. Its up to the implementation to use it.
    FlightPlannerInterface(QWidget* widget, Octree* pointCloud);
    virtual ~FlightPlannerInterface();

    // Insert points from the laserscanners. Note that the points might also be inserted
    // into Octree* pointCloud, this might be independent from the flightplanner.
    virtual void insertPoint(LidarPoint* const point) = 0;

    const Pose getLastKnownVehiclePose(void) const;
    const QVector3D getCurrentVehicleVelocity() const;

    const QList<WayPoint> getWayPoints();

    void getScanVolume(QVector3D& min, QVector3D& max);

public slots:
    void slotWayPointDelete(const quint16& index);
    void slotWayPointInsert(const quint16& index, const WayPoint& wpt);
    void slotWayPointInsertedByRover(const quint16& index, const WayPoint& wpt);
    void slotWayPointSwap(const quint16& i, const quint16& j);
    void slotWayPointsClear();

    virtual void slotWayPointReached(const WayPoint);

    virtual void slotSetScanVolume(const QVector3D min, const QVector3D max);
    virtual void slotGenerateWaypoints() = 0;
    virtual void slotVisualize() const;

    void slotVehiclePoseChanged(const Pose& pose);

signals:
    // Emitted to tell other classes that waypoint @wpt was inserted at index @index
    void wayPointInserted(const quint16& index, const WayPoint& wpt);

    // Emitted to tell the rover that it should insert waypoint @wpt at index @index
    void wayPointInsertOnRover(const quint16& index, const WayPoint& wpt);

    // Emitted to tell other classes that waypoint @index was deleted. We do not care how or by whom
    void wayPointDeleted(const quint16& index);
    // Emitted to tell the rover that it should delete waypoint @index
    void wayPointDeleteOnRover(const quint16& index);

    void wayPointsSetOnRover(QList<WayPoint>);
    void wayPoints(QList<WayPoint>);

    void suggestVisualization();
};

#endif // FLIGHTPLANNERINTERFACE_H
