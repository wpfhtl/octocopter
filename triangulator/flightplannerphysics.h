#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include "flightplannerinterface.h"
//#include "physicssphere.h"
#include "node.h"
#include "lidarpoint.h"
#include "openglutilities.h"

#include "bulletdebugdrawergl.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <btBroadphaseProxy.h>

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

    BulletDebugDrawerGl* mDbgDrawer;

    // testing
//    btTransform mDeletionTriggerTransform;

    btTransform mLidarPointTransform;
    btCollisionShape *mLidarPointShape;

    btTransform mDeletionTriggerTransform;
    btCollisionShape *mDeletionTriggerShape;
    btGhostObject *mDeletionTriggerGhostObject;

    btCompoundShape *mLidarFloorShape;
    btPairCachingGhostObject *mLidarFloorGhostObject;

    btSphereShape *mShapeSampleSphere;
    QList<btRigidBody*> mSampleSpheres;

    // In this QMap, we associate SampleSphere* -> QVector3D_Last_LidarPoint_Hit
    QMap<btRigidBody*, QVector3D> mLastSampleSphereHitPositions;

    // This list contains waypoints. They are never deleted, only added.
    QList<btRigidBody*> mWayPoints;

//    btAxisSweep3 *mBtBroadphase;
    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

    QList<btRigidBody*> mSampleSphereList;

signals:
    void newWayPoint(const QVector3D);

private slots:
    void slotPointInserted(const LidarPoint*);
    void slotGenerateWaypoints();

public slots:
    void slotWayPointReached(const QVector3D);
    void slotSetScanVolume(const QVector3D min, const QVector3D max);

};

#endif // FLIGHTPLANNERBASIC_H
