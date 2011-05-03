#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QObject>
#include "octree.h"
#include "glwidget.h"

class Pose;

class FlightPlannerInterface : public QObject
{
    Q_OBJECT
protected:
    QVector3D mScanVolumeMin, mScanVolumeMax;
    const Pose *mVehiclePose;

    static void sortToShortestPath(QVector<QVector3D> &wayPoints, const QVector3D &currentVehiclePosition);

public:
    FlightPlannerInterface(const Pose * const pose, Octree* pointCloud);
    virtual ~FlightPlannerInterface();

    // Insert points from the laserscanners. Note that the points might also be inserted
    // into Octree* pointCloud, this might be independent from the flightplanner.
    virtual void insertPoint(LidarPoint* const point) = 0;

    const Pose getVehiclePose(void) const;

public slots:
    virtual void slotSetScanVolume(const QVector3D min, const QVector3D max);
    virtual void slotWayPointReached(const QVector3D) = 0;
    virtual void slotGenerateWaypoints() = 0;
    virtual void slotVisualize() const = 0;

signals:
    void newWayPoint(const QVector3D);
    void clearWayPoints();
    void suggestVisualization();
};

#endif // FLIGHTPLANNERINTERFACE_H
