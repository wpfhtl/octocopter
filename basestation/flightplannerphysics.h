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

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    // testing
//    btTransform mDeletionTriggerTransform;

    btTransform mTransformLidarPoint;
    // The shape of a LIDAR-point, i.e. a small sphere
    btCollisionShape *mLidarPointShape;

    btTransform mTransformDeletionTrigger;

    // The detection volume at the bottom.
    btCollisionShape *mDeletionTriggerShape;

    // The Ghost Object using the detection shape at the bottom.
    btGhostObject *mGhostObjectDeletionTrigger;

    // For vehicle collision avoidance
    btTransform mTransformVehicle;
    btRigidBody *mBodyVehicle;
    btDefaultMotionState* mMotionStateVehicle;
//    btCollisionShape *mShapeVehicle;
    btPairCachingGhostObject *mGhostObjectVehicle;

    btCompoundShape *mPointCloudShape;
    btPairCachingGhostObject *mGhostObjectPointCloud;

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

    // Inserts detour-waypoints between vehicle position and next waypoint if necessary.
    // Returns true if path was found, else false.
    void slotCreateSafePathToNextWayPoint();

    // Overridden from base, create safe path to next waypoint whenever current one was reached.
    void slotWayPointReached(const WayPoint);

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose& pose);

    void slotVisualize() const;

};

#endif // FLIGHTPLANNERBASIC_H
