#include "flightplanner.h"

FlightPlanner::FlightPlanner(QObject *parent) : QObject(parent)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctree = 0;

    // Bullet initialisation.
    mBtBroadphase = new btDbvtBroadphase(); //new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024); // = new btDbvtBroadphase();
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mBtSolver = new btSequentialImpulseConstraintSolver;

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-9.8,0));

    mLidarPointTransform.setIdentity();

    // Construct spheres for lidarPoints and sample baloons
    mShapeLidarPoint = new btSphereShape(0.1);

    mShapeSampleSphere = new btSphereShape(5);


    // Create the floor below the scenery. Spheres that hit the floor get deleted.
    mShapeFloor = new btBoxShape(btVector3(10000, 1, 10000));
    btTransform floorTransform;
    floorTransform.setOrigin(btVector3(0,-100,0));
    btDefaultMotionState* floorMotionState = new btDefaultMotionState(floorTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, floorMotionState, mShapeFloor, btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);
    // TODO: use a ghost object instead, we don't need collision response
    // http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=9&t=3026

    // Add the body to the dynamics world
    mBtWorld->addRigidBody(body);
}

FlightPlanner::~FlightPlanner()
{
    // Cleanup in the reverse order of creation/initialization

    // Remove the rigidbodies from the dynamics world and delete them
    int i;
    for(i=mBtWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
            btCollisionObject* obj = mBtWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if(body && body->getMotionState())
            {
                    delete body->getMotionState();
            }
            mBtWorld->removeCollisionObject(obj);
            delete obj;
    }

    //delete collision shapes
//    for (int j=0;j<m_collisionShapes.size();j++)
//    {
//            btCollisionShape* shape = m_collisionShapes[j];
//            delete shape;
//    }

    delete mShapeLidarPoint;

    delete mShapeSampleSphere;

    delete mBtWorld;

    delete mBtSolver;

    delete mBtBroadphase;

    delete mBtDispatcher;

    delete mBtCollisionConfig;

    if(mOctree) delete mOctree;
}





void MyNearCallback(btBroadphasePair& collisionPair,
  btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo) {

    // Do your collision logic here
    // Only dispatch the Bullet collision information if you want the physics to continue
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

mDispatcher->setNearCallback(MyNearCallback);










Node* FlightPlanner::insertPoint(LidarPoint* const point)
{
    if(mOctree == 0)
    {
        // Create the octree around the first arriving point.
        mOctree = new Octree(
                point->position - QVector3D(10, 10, 10), // min
                point->position + QVector3D(10, 10, 10),  // max
                10);

        mOctree->setMinimumPointDistance(20);

        mOctree->setPointHandler(GlWidget::drawSphere);
    }

    Node* insertionNode = mOctree->insertPoint(point);

    if(insertionNode != 0)
    {
        // The point was inserted and saved. Create a physics-point in space representing this.

        // Set the points position
        mLidarPointTransform.setOrigin(btVector3(point->position.x(), point->position.y(), point->position.z()));

        // We don't need any inertia, mass etc,. as this body is static.
        btDefaultMotionState* lidarPointMotionState = new btDefaultMotionState(mLidarPointTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, lidarPointMotionState, mShapeLidarPoint, btVector3(0,0,0));
        btRigidBody* body = new btRigidBody(rbInfo);

        // Add the body to the dynamics world
        mBtWorld->addRigidBody(body);
    }

    return insertionNode;
}

QVector<QVector3D> FlightPlanner::getNextRoute()
{
    // Now the hard work:
    //
    // 1. Drop small spheres (big enough for the quad to fit in). Delete them if they sleep on other spheres.
    // 2. Let it dry until everyone is sleeping and falling spheres are deleted
    // 3. Drop big spheres (little smaller than lidar range). Delete small spheres being touched by big spheres.
    // 4. Let it dry until everyone is sleeping and falling spheres are deleted
    // 5. Freeze the spheres to let them remain for future iterations.
    // 6. Find a nice path through the remaining spheres.

}

void FlightPlanner::visualize() const
{
    if(mOctree) mOctree->handlePoints();
}
