#ifndef VEHICLE_H
#define VEHICLE_H

//#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>

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
#include "joystick.h"
#include <coordinateconverter.h>

class Simulator;
class OgreWidget;

class Vehicle : public QObject//QThread
{
    Q_OBJECT

    enum collisiontypes {
        COL_NOTHING = 0, //<Collide with nothing
        COL_VEHICLE = 1<<1, //<Collide with vehicles
        COL_GROUND = 1<<2, //<Collide with ground
        COL_POWERUP = 1<<3 //<Collide with powerups
    };


private:
    mutable QMutex mMutex;
    QTimer *mTimerUpdatePosition;
    int mTimeOfLastUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    Simulator *mSimulator;
    CoordinateConverter mCoordinateConverter;
    Ogre::Vector3 mNextWayPoint;
    OgreWidget* mOgreWidget;
    QList<Engine> mEngines; // probably deprecated for mEngineNodes and mEngine
    Engine mEngine;
    QList<Ogre::SceneNode*> mEngineNodes;
    Joystick *mJoystick;

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
    Vehicle(Simulator *simulator, OgreWidget *ogreWidget);
    ~Vehicle();

    QVector3D getLinearVelocity() const;
    QVector3D getAngularVelocity() const;

signals:
    void newPose(const Ogre::Vector3 pos, const Ogre::Quaternion rot);

public slots:
    void run(void);
    void start(void);
    void stop(void);
    void slotSetNextWayPoint(const CoordinateGps &wayPoint);
    void slotSetMotorSpeeds(const QList<int> &speeds);
    void slotShutDown(void);

private slots:
    void slotUpdatePosition(void);
    void slotUpdatePhysics(void);
};

#endif
