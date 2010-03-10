#ifndef VEHICLE_H
#define VEHICLE_H

#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>
//#include <Ogre.h>

#include "BtOgre/BtOgrePG.h"
#include "BtOgre/BtOgreGP.h"
#include "BtOgre/BtOgreExtras.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include "simulator.h"
//#include "physics.h"
#include "coordinateconverter.h"

class Simulator;

class Vehicle : public QObject//QThread
{
    Q_OBJECT

#define BIT(x) (1<<(x))
enum collisiontypes {
    COL_NOTHING = 0, //<Collide with nothing
    COL_VEHICLE = BIT(1), //<Collide with vehicles
    COL_GROUND = BIT(2), //<Collide with ground
    COL_POWERUP = BIT(3) //<Collide with powerups
};


private:
    mutable QMutex mMutex;
    QTimer *mTimerUpdatePosition;
    int mTimeOfLastUpdate; // the last simulationtime from simulator. Needed, as the physics engine needs deltas.
    Simulator *mSimulator;
    CoordinateConverter mCoordinateConverter;
    Ogre::Vector3 mNextWayPoint;
    OgreWidget* mOgreWidget;
    QList<Ogre::Vector3> mMotorPositions;

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
    btConvexShape *mVehicleShape;

    Ogre::Entity *mGroundEntity;
    btRigidBody *mGroundBody;
    btCollisionShape *mGroundShape;

public:
    Vehicle(Simulator *simulator, OgreWidget *ogreWidget);
    ~Vehicle();

signals:
    void newVehiclePose(Ogre::Vector3 pos, Ogre::Quaternion rot);

public slots:
    void run(void);
    void start(void);
    void stop(void);
    void slotSetNextWayPoint(const CoordinateGps &wayPoint);
    void slotShutDown(void);

private slots:
    void slotUpdatePosition(void);
    void slotUpdatePhysics(void);
};

#endif
