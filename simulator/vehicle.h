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
#include "CollisionShapes/btHeightfieldTerrainShape.h"

#include "simulator.h"
//#include "physics.h"
#include "coordinateconverter.h"

class Simulator;

class Vehicle : public QObject//QThread
{
    Q_OBJECT
private:
    mutable QMutex mMutex;
    QTimer *mTimerUpdatePosition;
    int mTimeOfLastUpdate;
    Simulator *mSimulator;
    CoordinateConverter mCoordinateConverter;
    Ogre::Vector3 mNextWayPoint;
    OgreWidget* mOgreWidget;

protected:
    btAxisSweep3 *mBtBroadphase;
    btDefaultCollisionConfiguration *mBtCollisionConfig;
    btCollisionDispatcher *mBtDispatcher;
    btSequentialImpulseConstraintSolver *mBtSolver;
    btDiscreteDynamicsWorld *mBtWorld;

    BtOgre::DebugDrawer *mBtDebugDrawer;

    Ogre::SceneNode *mNinjaNode;
    Ogre::Entity *mNinjaEntity;
    btRigidBody *mNinjaBody;
    btCollisionShape *mNinjaShape;

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
    void slotSetNextWayPoint(const CoordinateGps &wayPoint);
    void slotShutDown(void);

private slots:
    void slotUpdatePosition(void);
};

#endif
