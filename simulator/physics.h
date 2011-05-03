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
class OgreWidget;
class FlightController;

class Physics : public QObject//QThread
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
//    QTimer *mTimerUpdateGps;
    int mTimeOfLastUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    Simulator *mSimulator;
    OgreWidget* mOgreWidget;
    Engine mEngine;
    QList<Ogre::SceneNode*> mEngineNodes;
    Battery* mBattery;
    float mTotalVehicleWeight;

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
    btConvexHullShape *mVehicleShape;
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

    FlightController* mFlightController;

signals:
    void newVehiclePose(const Pose&);

public slots:
//    void start(void);
//    void stop(void);
    void slotSetMotion(const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height);
    void slotUpdateWind();
    void slotSetTotalVehicleWeight(const float&);
    void slotUpdatePhysics(void);

private slots:
//    void slotEmitVehiclePose();
};

#endif
