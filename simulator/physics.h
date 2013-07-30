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
//#include "flightcontroller.h"
#include <pose.h>

class Simulator;
class MotionCommand;
class OgreWidget;
class FlightController;

class Physics : public QObject//QThread
{
    Q_OBJECT

private:
    Simulator *mSimulator;
    OgreWidget* mOgreWidget;
    quint32 mTimeOfLastPhysicsUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    quint32 mTimeOfLastControllerUpdate;
    Engine mEngine;
    QList<Ogre::SceneNode*> mEngineNodes;
    static constexpr float mTotalVehicleWeight = 2.450f;
    QVector<btVector3> mVectorWind;
    Pose mVehiclePose;

    // for the pitch/roll-low-level-controller
    float mErrorIntegralPitch, mErrorIntegralRoll;
    float mPrevErrorPitch, mPrevErrorRoll;

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

signals:
    void newVehiclePose(const Pose* const);
    void windInitialized(const bool&);

public slots:
    void slotSetMotion(const MotionCommand *const mc);
    void slotSetWindSetting(const bool& enable, const float& factor);
    void slotUpdatePhysics(void);

    void slotRescueVehicle();

private slots:
    void slotUpdateWind();
};

#endif
