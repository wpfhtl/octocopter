#ifndef PHYSICS_H
#define PHYSICS_H

//#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>
#include <QDebug>

#include <OgreTerrainGroup.h>

#include "BtOgre/BtOgrePG.h"
#include "BtOgre/BtOgreGP.h"
#include "BtOgre/BtOgreExtras.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <btBulletDynamicsCommon.h>

#include "simulator.h"
#include "engine.h"
#include "ogrewidget.h"
#include "flightcontroller.h"
#include <pose.h>

class Simulator;
class MotionCommand;
class OgreWidget;
class FlightController;

class Physics : public QObject//QThread
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
//    QTimer *mTimerUpdateGps;
    quint32 mTimeOfLastPhysicsUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    Simulator *mSimulator;
    OgreWidget* mOgreWidget;
    Engine mEngine;
    QList<Ogre::SceneNode*> mEngineNodes;
    float mTotalVehicleWeight;
    QVector<btVector3> mVectorWind;
    Pose mVehiclePose;

    // for the pitch/roll-low-level-controller
    float mErrorIntegralPitch, mErrorIntegralRoll;
    float mPrevErrorPitch, mPrevErrorRoll;
    quint32 mTimeOfLastControllerUpdate;

    bool mWindEnable;
    float mWindFactor;

    bool initializeWind();

protected:
    btAxisSweep3 *mBtBroadphase;
    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

    BtOgre::DebugDrawer *mBtDebugDrawer;

    Ogre::SceneNode *mVehicleNode;
    Ogre::Entity *mVehicleEntity;
    btRigidBody *mVehicleBody;
//    btConvexHullShape *mVehicleShape;
    btBoxShape *mVehicleShape;
    BtOgre::RigidBodyState *mVehicleState;

    Ogre::Entity *mGroundEntity;
    btRigidBody *mGroundBody;
    btCollisionShape *mGroundShape;

public:
    Physics(Simulator *simulator, OgreWidget *ogreWidget);
    ~Physics();

    QVector3D getVehicleLinearVelocity() const;
    QVector3D getVehicleAngularVelocity() const;
    float getHeightAboveGround();
    const Pose* const getVehiclePose() {return &mVehiclePose;}

    FlightController* mFlightController;

signals:
    void newVehiclePose(const Pose* const);
    void windInitialized(const bool&);

public slots:
//    void start(void);
//    void stop(void);
    void slotSetMotion(const MotionCommand *const mc);
    void slotSetWindSetting(const bool& enable, const float& factor);
    void slotSetTotalVehicleWeight(const float&);
    void slotUpdatePhysics(void);

private slots:
    void slotUpdateWind();
    void slotUpdateEngineRotations();
};

#endif
