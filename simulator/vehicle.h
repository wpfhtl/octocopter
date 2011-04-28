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
#include "flightcontroller.h"
#include <coordinateconverter.h>

class Simulator;
class OgreWidget;
class FlightController;

class Vehicle : public QObject//QThread
{
    Q_OBJECT

private:
    mutable QMutex mMutex;
    QTimer *mTimerUpdatePosition;
    int mTimeOfLastUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    Simulator *mSimulator;
//    CoordinateConverter mCoordinateConverter;
//    Ogre::Vector3 mNextWayPoint;
    OgreWidget* mOgreWidget;
    QList<Engine> mEngines; // probably deprecated for mEngineNodes and mEngine
    Engine mEngine;
    QList<Ogre::SceneNode*> mEngineNodes;
    Battery* mBattery;

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
    float getHeightAboveGround();

    FlightController* mFlightController;

signals:
    void newPose(const Ogre::Vector3 pos, const Ogre::Quaternion rot);

public slots:
    void run(void);
    void start(void);
    void stop(void);
//    void slotSetNextWayPoint(const CoordinateGps &wayPoint);
//    void slotSetMotorSpeeds(const QList<int> &speeds);
    void slotSetMotion(const quint8& thrust, const qint8& nick, const qint8& roll, const qint8& yaw, const qint8& height);
    void slotShutDown(void);
    void slotUpdateWind();

private slots:
    void slotUpdatePosition(void);
    void slotUpdatePhysics(void);
};

#endif
