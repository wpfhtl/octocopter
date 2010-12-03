#ifndef FLIGHTPLANNER_H
#define FLIGHTPLANNER_H

#include <QObject>
#include "octree.h"
#include "glwidget.h"

class FlightPlannerBasic;

class FlightPlanner : public QObject
{
    Q_OBJECT
public:
    FlightPlanner(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud);
    virtual ~FlightPlanner();

    // Insert points from the laserscanners. Note that the points might also be inserted
    // into Octree* pointCloud, this might be independent from the flightplanner.
    virtual Node* insertPoint(LidarPoint* const point) = 0;

    // Issues opengl-commands to visualize the flightplanning
    virtual void visualize() const = 0;

public slots:
    virtual void slotWayPointReached(const QVector3D) = 0;

signals:
    void newWayPoint(const QVector3D);
    void clearWayPoints();
};

#endif // FLIGHTPLANNER_H
