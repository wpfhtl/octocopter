#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include "flightplanner.h"
#include "node.h"
#include "lidarpoint.h"

#include <btBulletDynamicsCommon.h>

class FlightPlannerPhysics : public FlightPlanner
{
    Q_OBJECT
public:
    FlightPlannerPhysics(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud);
    ~FlightPlannerPhysics();

    Node* insertPoint(LidarPoint* const point);

    void visualize() const;

private:
    Octree* mOctree;

    btTransform mLidarPointTransform;
    btCollisionShape *mShapeLidarPoint, *mShapeSampleSphere, *mShapeFloor;


//    btAxisSweep3 *mBtBroadphase;
    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

signals:
    void newWayPoint(const QVector3D);

private slots:
    void slotPointInserted(const LidarPoint*);

};

#endif // FLIGHTPLANNERBASIC_H
