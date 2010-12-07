#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include "flightplannerinterface.h"
#include "physicssphere.h"
#include "node.h"
#include "lidarpoint.h"
#include "openglutilities.h"

#include <btBulletDynamicsCommon.h>

class FlightPlannerPhysics : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerPhysics(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud);
    ~FlightPlannerPhysics();

    void insertPoint(LidarPoint* const point);

    void visualize() const;

private:
    Octree* mOctree;

    btTransform mLidarPointTransform;
    btCollisionShape *mShapeLidarPoint, *mShapeFloor;
    btSphereShape *mShapeSampleSphere;

    btGhostObject *mFloorGhostObject;

    QList<PhysicsSphere*> mDroppingSpheres;

//    btAxisSweep3 *mBtBroadphase;
    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

    QVector<QVector3D> getNextRoute();

signals:
    void newWayPoint(const QVector3D);

private slots:
    void slotPointInserted(const LidarPoint*);

};

#endif // FLIGHTPLANNERBASIC_H
