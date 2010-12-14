#ifndef FLIGHTPLANNERINTERFACE_H
#define FLIGHTPLANNERINTERFACE_H

#include <QObject>
#include "octree.h"
#include "glwidget.h"

class FlightPlannerInterface : public QObject
{
    Q_OBJECT
protected:
    QVector3D mScanVolumeMin, mScanVolumeMax;

public:
    FlightPlannerInterface(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud);
    virtual ~FlightPlannerInterface();

    // Insert points from the laserscanners. Note that the points might also be inserted
    // into Octree* pointCloud, this might be independent from the flightplanner.
    virtual void insertPoint(LidarPoint* const point) = 0;

    // Issues opengl-commands to visualize the flightplanning
    virtual void visualize() const = 0;


public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);
    virtual void slotWayPointReached(const QVector3D) = 0;
    virtual void slotGenerateWaypoints() = 0;

signals:
    void newWayPoint(const QVector3D);
    void clearWayPoints();
    void suggestVisualization();
};

#endif // FLIGHTPLANNERINTERFACE_H
