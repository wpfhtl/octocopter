#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include "flightplannerinterface.h"
#include "flightplannerphysicsdialog.h"
#include "node.h"
#include "lidarpoint.h"
#include <waypoint.h>
#include "openglutilities.h"

#include "bulletdebugdrawergl.h"

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>
#include <bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

class FlightPlannerPhysics : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerPhysics(QWidget* widget, Octree* pointCloud);
    ~FlightPlannerPhysics();

    void insertPoint(LidarPoint* const point);

private:
    Octree* mOctree;
    FlightPlannerPhysicsDialog* mDialog;
    BulletDebugDrawerGl* mDbgDrawer;
    bool mPhysicsProcessingActive;

    QList<WayPoint> mWayPointsGenerated;

    // testing
//    btTransform mDeletionTriggerTransform;

    btTransform mLidarPointTransform;
    btCollisionShape *mLidarPointShape;

    btTransform mDeletionTriggerTransform;
    btCollisionShape *mDeletionTriggerShape;
    btGhostObject *mDeletionTriggerGhostObject;

    btCompoundShape *mPointCloudShape;
    btPairCachingGhostObject *mPointCloudGhostObject;

    btSphereShape *mShapeSampleSphere;
    QList<btRigidBody*> mSampleObjects;

    // In this QMap, we associate SampleSphere* -> QVector3D_Last_LidarPoint_Hit
    QMap<btRigidBody*, QVector3D> mLastSampleObjectHitPositions;

    // This list contains waypoints. They are never deleted, only added.
//    QList<btRigidBody*> mWayPoints;

//    btAxisSweep3 *mBtBroadphase;
    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

signals:
//    void newWayPointsReady(const QList<WayPoint>&);
//    void processingStatus(const QString& text, const quint8 percentReady);

private slots:
    void slotPointInserted(const LidarPoint*);
    void slotGenerateWaypoints();
    void slotCreateSampleGeometry();
    void slotDeleteSampleGeometry();
    void slotProcessPhysics(bool);
    void slotEmitWayPoints();
    void slotGravityChanged(const QVector3D& gravity);
    void slotFrictionChanged(const float& frictionPointCloudGround, const float& frictionSampleGeometry);
    void slotRestitutionChanged(const float& restitutionPointCloudGround, const float& restitutionSampleGeometry);

public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);
    void slotVisualize() const;

};

#endif // FLIGHTPLANNERBASIC_H
