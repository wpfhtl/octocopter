#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include "flightplannerinterface.h"

#include "bulletdebugdrawergl.h"
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>
//#include <bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

class WayPoint;
class LidarPoint;
class FlightPlannerPhysicsDialog;

class FlightPlannerPhysics : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerPhysics(QWidget* widget, Octree* pointCloud);
    ~FlightPlannerPhysics();

    void insertPoint(LidarPoint* point);

    qint64 getNumberOfPointsInCollisionOctree();

private:
    class CollisionSphereState : public btMotionState
    {
    protected:
        btTransform mTransform;

    public:
        // The minimum distance travelled by any samplesphere in this round of waypoint generation. Used as threshold for termination
        static float mFarthestDistanceTravelled;

        CollisionSphereState(const btTransform& transform) : mTransform(transform)
        {
        }

        virtual void getWorldTransform(btTransform &ret) const
        {
            ret = mTransform;
        }

        virtual void setWorldTransform(const btTransform &in)
        {
            mFarthestDistanceTravelled = std::max(mFarthestDistanceTravelled, mTransform.getOrigin().distance2(in.getOrigin()));
            mTransform = in;
        }

        void updateVbo(const quint32 vbo, const quint32 index);
    };

    Octree* mOctreeCollisionObjects;
    FlightPlannerPhysicsDialog* mDialog;
    BulletDebugDrawerGl* mDbgDrawer;
    bool mPhysicsProcessingActive;


    btRigidBody* mGapFindVolumeRigidBody;
    btCompoundShape* mGapFindVolumeCollisionShape; // a list of btstaticplaneshapes making up the gap detection volume
//    btStaticPlaneShape* mGapFindVolumeCollisionShape; // a list of btstaticplaneshapes making up the gap detection volume

    quint32 mDeletionTriggerVbo;

    quint32 mSampleSphereVbo;

    bool mFirstSphereHasHitThisIteration;

    btTransform mTransformLidarPoint;
    // The shape of a LIDAR-point, i.e. a small sphere
    btCollisionShape *mLidarPointShape;

    btTransform mDeletionTriggerTransform;

    // The detection volume at the bottom.
    btCollisionShape *mDeletionTriggerShape;

    // The Ghost Object using the detection shape at the bottom.
    btGhostObject *mDeletionTriggerGhostObject;

    // For vehicle collision avoidance
    btTransform mTransformVehicle;
    btRigidBody *mBodyVehicle;
//    btCollisionShape *mShapeVehicle;
    btPairCachingGhostObject *mGhostObjectVehicle;

    btCompoundShape *mPointCloudShape;
    btPairCachingGhostObject *mGhostObjectPointCloud;

    btSphereShape *mShapeSampleSphere;
    QList<btRigidBody*> mSampleObjects;

    // In this QMap, we associate SampleSphere* -> QVector3D_Last_LidarPoint_Hit
    QMap<btRigidBody*, QVector3D> mLastSampleObjectHitPositions;

    btDbvtBroadphase *mBtBroadphase;

    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

private slots:
    void slotPointInserted(const LidarPoint*);
    void slotGenerateWaypoints();
    void slotInitialize();
    void slotCreateSampleGeometry();
    void slotDeleteSampleGeometry();
    void slotProcessPhysics(bool);
    void slotSubmitGeneratedWayPoints();
    void slotDeleteGeneratedWayPoints();
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

    void slotVisualize();

};

#endif // FLIGHTPLANNERBASIC_H
