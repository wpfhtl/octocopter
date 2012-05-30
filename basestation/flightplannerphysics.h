#ifndef FLIGHTPLANNERPHYSICS_H
#define FLIGHTPLANNERPHYSICS_H

#include <QApplication>

#include "flightplannerinterface.h"
#include "flightplannerphysicsdialog.h"
#include "node.h"
#include "lidarpoint.h"
#include <waypoint.h>
//#include "openglutilities.h"

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

    void insertPoint(LidarPoint* point);

    qint64 getNumberOfPointsInCollisionOctree();

private:





    class CollisionSphereState : public btMotionState
    {
    protected:
//        quint16 mIndex; // The serial number of this collision sphere. Will be used to update the right place in the spheres-vbo
        btTransform mTransform;
//        quint32 mVbo; // the VBO storing the sample sphere positions. Will be updated from setWroldTransform, which is called by bullet

    public:
        CollisionSphereState(const btTransform& transform) : mTransform(transform)
        {
//            updateVbo();
        }

        virtual void getWorldTransform(btTransform &ret) const
        {
            ret = mTransform;
        }

        virtual void setWorldTransform(const btTransform &in)
        {
            mTransform = in;
            //updateVbo(); we could do it here, but thats scattered. Rather do it in a loop after mBtWorld->step()
        }

        void updateVbo(const quint32 vbo, const quint32 index);
/*
        QVector3D getPosition(void) const
        {
            const btVector3 pos = mTransform.getOrigin();
            return QVector3D(pos.x(), pos.y(), pos.z());
        }

        QQuaternion getOrientation(void) const
        {
            btQuaternion rot = mTransform.getRotation();
            return QQuaternion(rot.w(), rot.x(), rot.y(), rot.z());
        }*/
    };

    Octree* mOctreeCollisionObjects;
    FlightPlannerPhysicsDialog* mDialog;
    BulletDebugDrawerGl* mDbgDrawer;
    bool mPhysicsProcessingActive;

    quint32 mDeletionTriggerVbo;

    quint32 mSampleSphereVbo;

    bool mFirstSphereHasHitThisIteration;

//    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

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
