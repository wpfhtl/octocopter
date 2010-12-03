#ifndef FLIGHTPLANNERBASIC_H
#define FLIGHTPLANNERBASIC_H

#include "flightplanner.h"
#include "node.h"
#include "lidarpoint.h"

/*
  This FlightPlanner is very simple: A distinct octree is used to store the cloud points.
  Waypoints are then generated from those points using the condition of them being at least
  20m apart from each other. Also, 15m are added to the y-component so we don't scratch the
  ground.
*/

class FlightPlannerBasic : public FlightPlanner
{
    Q_OBJECT

public:
    FlightPlannerBasic(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud);
    ~FlightPlannerBasic();

    Node* insertPoint(LidarPoint* const point);

    void visualize() const;

private:
    Octree* mOctree;

signals:
    void newWayPoint(const QVector3D);

private slots:
    void slotPointInserted(const LidarPoint*);

public slots:
    void slotWayPointReached(const QVector3D);

};

#endif // FLIGHTPLANNERBASIC_H
