#include "flightplannerphysics.h"

FlightPlannerPhysics::FlightPlannerPhysics(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud) : FlightPlannerInterface(position, orientation, pointCloud)
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


    //btStaticPlaneShape - As the name suggests, the btStaticPlaneShape can represent an infinite plane or half space. This shape can only be used for static, non-moving objects. This shape has been introduced mainly for demo purposes.
    // Create the floor below the scenery. Spheres that hit the floor get deleted.
    mShapeFloor = new btBoxShape(btVector3(10000, 10, 10000));
    btTransform floorTransform;
    floorTransform.setOrigin(btVector3(0,-100,0));
//    btDefaultMotionState* floorMotionState = new btDefaultMotionState(floorTransform);
//    btRigidBody::btRigidBodyConstructionInfo floorInfo(0.0f, floorMotionState, mShapeFloor, btVector3(0,0,0));
//    btRigidBody* floorBody = new btRigidBody(floorInfo);
//    floorBody->setCollisionFlags(floorBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

    // Add the body to the dynamics world
//    mBtWorld->addRigidBody(floorBody);

    // TODO: use a ghost object instead, we don't need collision response
    // http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=9&t=3026

    mFloorGhostObject = new btmFloorGhostObject();
    mFloorGhostObject->setWorldTransform(floorTransform);
    mFloorGhostObject->setCollisionShape(mShapeFloor);
    mFloorGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

    // register a ghost pair callback to activate the world's ghost functionality
    mBtBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

    mBtWorld->addCollisionObject(mFloorGhostObject,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);
}

FlightPlannerPhysics::~FlightPlannerPhysics()
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

void FlightPlannerPhysics::insertPoint(LidarPoint* const point)
{
    if(mOctree == 0)
    {
        // Create the octree around the first arriving point.
        mOctree = new Octree(
                point->position - QVector3D(10, 10, 10), // min
                point->position + QVector3D(10, 10, 10),  // max
                10);

        mOctree->setMinimumPointDistance(1);

        mOctree->setPointHandler(OpenGlUtilities::drawSphere);

        connect(mOctree, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointInserted(const LidarPoint*)));
    }

    mOctree->insertPoint(point);
}

QVector<QVector3D> FlightPlannerPhysics::getNextRoute()
{
    // Now the hard work:
    //
    // 1. Drop small spheres (big enough for the quad to fit in). Delete them if they sleep on other spheres.
    // 2. Let it dry until everyone is sleeping and falling spheres are deleted
    // 3. Drop big spheres (little smaller than lidar range). Delete small spheres being touched by big spheres.
    // 4. Let it dry until everyone is sleeping and falling spheres are deleted
    // 5. Freeze the spheres to let them remain for future iterations.
    // 6. Find a nice path through the remaining spheres.

    // The sphere that'll be dropped from the sky.

    btTransform sampleSphereTransform;

    // Fill the sky with spheres
    for(
        float x = mScanVolumeMin.x() + mShapeSampleSphere->getRadius()/2.0;
        x <= mScanVolumeMax.x() - mShapeSampleSphere->getRadius()/2.0;
        x += mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions*/)
    {
        for(
            float z = mScanVolumeMin.z() + mShapeSampleSphere->getRadius()/2.0;
            z <= mScanVolumeMax.z() - mShapeSampleSphere->getRadius()/2.0;
            z += mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions*/)
        {
//            PhysicsSphere* ps = new PhysicsSphere(mBtWorld, 1.0f, btVector3(x, mScanVolumeMax.y(), z), this);

            sampleSphereTransform.setOrigin(btVector3(x, mScanVolumeMax.y()+5.0, z));
            // We don't need any inertia, mass etc,. as this body is static.
            btDefaultMotionState* lidarPointMotionState = new btDefaultMotionState(mLidarPointTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, lidarPointMotionState, mShapeSampleSphere, btVector3(0,0,0));
            btRigidBody* body = new btRigidBody(rbInfo);

            // Add the body to the dynamics world
            mBtWorld->addRigidBody(body);
        }
    }

    // And now let them fall from the sky...

    // We iterate through all rigidBodies touching our floor and remove/delete them.
    for(int i = 0; i < mFloorGhostObject->getNumOverlappingObjects(); i++)
    {
       // Dynamic cast to make sure its a rigid body
       btRigidBody *pRigidBody = dynamic_cast<btRigidBody *>(mFloorGhostObject->getOverlappingObject(i));
       if(pRigidBody)
       {
          pRigidBody->activate(true);
          pRigidBody->applyCentralImpulse(rayTo);
       }
    }

    return QVector<QVector3D>();
}

// A point was inserted ito our octree. Create a corresponding point in our physics world.
void FlightPlannerPhysics::slotPointInserted(const LidarPoint* lp)
{
    mLidarPointTransform.setOrigin(btVector3(lp->position.x(), lp->position.y(), lp->position.z()));

    // We don't need any inertia, mass etc,. as this body is static.
    btDefaultMotionState* lidarPointMotionState = new btDefaultMotionState(mLidarPointTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, lidarPointMotionState, mShapeLidarPoint, btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);

    // Add the body to the dynamics world
    mBtWorld->addRigidBody(body);
}

void FlightPlannerPhysics::visualize() const
{
    if(mOctree) mOctree->handlePoints();
}
