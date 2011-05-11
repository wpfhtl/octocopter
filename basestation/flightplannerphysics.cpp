#include "flightplannerphysics.h"

FlightPlannerPhysics::FlightPlannerPhysics(const Pose * const pose, Octree* pointCloud) : FlightPlannerInterface(pose, pointCloud)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctree = 0;

    // Bullet initialisation.
    mBtBroadphase = new btDbvtBroadphase; //new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024); // = new btDbvtBroadphase();
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mBtSolver = new btSequentialImpulseConstraintSolver;

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
//    mBtWorld->setGravity(btVector3(0,-9.8,0));

//    mDbgDrawer = new BulletDebugDrawerGl;
//    mBtWorld->setDebugDrawer(mDbgDrawer);

    mLidarPointTransform.setIdentity();

    // Construct spheres for lidarPoints and sampleSpheres
    mLidarPointShape = new btSphereShape(0.1);
    mShapeSampleSphere = new btSphereShape(2);

    // Set up a ghost object that deletes SampleSpheres when they hit it.
    mDeletionTriggerTransform.setIdentity();
    mDeletionTriggerTransform.setOrigin(btVector3(0.0, mScanVolumeMin.y()-10.0, 0.0));
    mDeletionTriggerShape = new btBoxShape(btVector3(500, 10, 500)); // These are HALF-extents!
    mDeletionTriggerGhostObject = new btGhostObject;
    mDeletionTriggerGhostObject->setWorldTransform(mDeletionTriggerTransform);
    mDeletionTriggerGhostObject->setCollisionShape(mDeletionTriggerShape);
    mDeletionTriggerGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mDeletionTriggerGhostObject, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // Set up a ghost object with a compound shape, containing all btSphereShapes
    btTransform lidarFloorTransform;
    lidarFloorTransform.setIdentity();

    mLidarFloorShape = new btCompoundShape(false);

    mLidarFloorGhostObject = new btPairCachingGhostObject();
    mLidarFloorGhostObject->setFriction(1000.0);
//    mLidarFloorGhostObject->setRestitution(?);
    mLidarFloorGhostObject->setWorldTransform(lidarFloorTransform);
    mLidarFloorGhostObject->setCollisionShape(mLidarFloorShape);
//    mLidarFloorGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mLidarFloorGhostObject, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // register a ghost pair callback to activate the world's ghost functionality
    mBtBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

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

    delete mLidarPointShape;

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

        mOctree->setMinimumPointDistance(2);

        mOctree->setPointHandler(OpenGlUtilities::drawPoint);

        connect(mOctree, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointInserted(const LidarPoint*)));
    }

    mOctree->insertPoint(point);
}

void FlightPlannerPhysics::slotGenerateWaypoints()
{
    // Now the hard work:
    //
    // 1. Drop small spheres (big enough for the vehicle to fit in). Delete them if they sleep on other spheres.
    // 2. Let it dry until everyone is sleeping and falling spheres are deleted
    // 3. Drop big spheres (little smaller than lidar range). Delete small spheres being touched by big spheres.
    // 4. Let it dry until everyone is sleeping and falling spheres are deleted
    // 5. Freeze the spheres to let them remain for future iterations.
    // 6. Find a nice path through the remaining spheres.

    // The sphere that'll be dropped from the sky.

    qDebug() << "starting simulated annealing";

    btTransform sampleSphereTransform;

    // Fill the sky with spheres
    for(
        float x = mScanVolumeMin.x() + mShapeSampleSphere->getRadius()/2.0;
        x <= mScanVolumeMax.x() - mShapeSampleSphere->getRadius()/2.0;
        x += 2*mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions*/)
    {
        for(
            float z = mScanVolumeMin.z() + mShapeSampleSphere->getRadius()/2.0;
            z <= mScanVolumeMax.z() - mShapeSampleSphere->getRadius()/2.0;
            z += 2*mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions*/)
        {
//            PhysicsSphere* ps = new PhysicsSphere(mBtWorld, 1.0f, btVector3(x, mScanVolumeMax.y(), z), this);

            sampleSphereTransform.setOrigin(btVector3(x, mScanVolumeMax.y()+5.0, z));
            // We don't need any inertia, mass etc,. as this body is static.
            btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mShapeSampleSphere->getRadius(), sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setFriction(0.01);

            // Add the body to the dynamics world
            mSampleSpheres.append(body);
            mBtWorld->addRigidBody(body);
        }
    }

    qDebug() << "number of spheres:"<< mSampleSpheres.size();

    mLastSampleSphereHitPositions.clear();

    QList<WayPoint> newWayPoints;

    // And now let them fall from the sky...
    for(int i=0;i<1000;i++)
    {
        emit processingStatus("Computing physics...", ((i+1)/10));

        mBtWorld->applyGravity();
        mBtWorld->stepSimulation(0.01, 10);

        btBroadphasePairArray& pairs = mLidarFloorGhostObject->getOverlappingPairCache()->getOverlappingPairArray();
        for (int j=0; j<pairs.size(); ++j)
        {
            const btBroadphasePair& pair = pairs[j];
            btBroadphaseProxy* proxy = pair.m_pProxy0->m_clientObject != mLidarFloorGhostObject ? pair.m_pProxy0 : pair.m_pProxy1;

//            btCollisionObject* obj = (btCollisionObject*)proxy->m_clientObject;
            btRigidBody *rigidBody = dynamic_cast<btRigidBody *>((btCollisionObject*)(proxy->m_clientObject));
            if(rigidBody)
            {
                btTransform t = rigidBody->getWorldTransform();
                QVector3D pos(t.getOrigin().x(), t.getOrigin().y()+5, t.getOrigin().z());
//                qDebug() << "a samplesphere has hit the lidarground at" << pos;
                mLastSampleSphereHitPositions.insert(rigidBody, pos);
            }
            else qDebug() << "dynamic cast failed!";
        }


        /* OLD WAY, ONLY AABB IS CHECKED
        // We iterate through all SampleSpheres touching our floor and remember WHERE they were when they touched the floor.
        for(int j = 0; j < mLidarFloorGhostObject->getNumOverlappingObjects(); j++)
        {
           // Dynamic cast to make sure its a rigid body
           btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mLidarFloorGhostObject->getOverlappingObject(j));
           if(rigidBody)
           {
               btTransform t;
               rigidBody->getMotionState()->getWorldTransform(t);
               QVector3D pos(t.getOrigin().x(), t.getOrigin().y()+5, t.getOrigin().z());
//               qDebug() << "a samplesphere has hit the lidarground at" << pos;
               mLastSampleSphereHitPositions.insert(rigidBody, pos);
           }
        }*/

        if(mDeletionTriggerGhostObject->getNumOverlappingObjects())
            qDebug() << "physics iteration" << i << "number of objects fallen through:" << mDeletionTriggerGhostObject->getNumOverlappingObjects();

        // We iterate through all rigidBodies touching our deletionTrigger and remove/delete them.
        for(int j = 0; j < mDeletionTriggerGhostObject->getNumOverlappingObjects(); j++)
        {
           // Dynamic cast to make sure its a rigid body
           btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mDeletionTriggerGhostObject->getOverlappingObject(j));
           if(rigidBody)
           {
               qDebug() << "removing sphere that has dropped too far.";

               // THIS IS THE INTERESTING PART! IF THE SPHERE HAS PREVIOUSLY HIT A LIDARPOINT, MAKE THAT A NEW WAYPOINT!
               if(mLastSampleSphereHitPositions.contains(rigidBody))
               {
//                   QVector3D pos = mLastSampleSphereHitPositions.value(rigidBody) + QVector3D(0.0, 3 * mShapeSampleSphere->getRadius(), 0.0);
                   WayPoint w(mLastSampleSphereHitPositions.value(rigidBody) + QVector3D(0.0, 5 * mShapeSampleSphere->getRadius(), 0.0));
//                   emit newWayPoint(pos);
                   newWayPoints.append(w);

                   sampleSphereTransform.setOrigin(btVector3(w.x(), w.y(), w.z()));
                   // We don't need any inertia, mass etc,. as this body is static.
                   btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
                   btRigidBody::btRigidBodyConstructionInfo rbInfo(0, sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
                   btRigidBody* bodyNew = new btRigidBody(rbInfo);
                   bodyNew->setFriction(0.01);

                   // Add the body to the dynamics world
                   mWayPoints.append(bodyNew);
                   mBtWorld->addRigidBody(bodyNew);
               }

               mBtWorld->removeRigidBody(rigidBody);
               mSampleSpheres.removeOne(rigidBody);
               delete rigidBody->getMotionState();
               delete rigidBody;
           }
        }

//        qDebug() << "requesting visualization" << i;

        emit suggestVisualization();
        QApplication::processEvents();
    }

    qDebug() << "number of waypoints to emit:" << mSampleSpheres.size();

    sortToShortestPath(newWayPoints, mVehiclePose->position);

    emit newWayPointsReady(newWayPoints);
//    foreach(const QVector3D& wpt, newWaypoints)
//        emit newWayPoint(wpt);

    // remove the old samplespheres...
    foreach(btRigidBody* body, mSampleSpheres)
    {
//        btTransform t;
//        body->getMotionState()->getWorldTransform(t);
//        QVector3D wp(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
//        qDebug() << "new waypoint" << wp;
//        emit newWayPoint(wp);

        mBtWorld->removeRigidBody(body);
        mSampleSpheres.removeOne(body);
    }
}

// A point was inserted ito our octree. Create a corresponding point in our physics world.
void FlightPlannerPhysics::slotPointInserted(const LidarPoint* lp)
{
    // We add this lidarPoint as a sphereShape child to the mLidarFloorShape, which is a btCompoundShape
    mLidarPointTransform.setOrigin(btVector3(lp->position.x(), lp->position.y(), lp->position.z()));
    mLidarFloorShape->addChildShape(mLidarPointTransform, mLidarPointShape);
}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerPhysics::slotVisualize() const
{
    mBtWorld->debugDrawWorld();

    // Draw the scanVolume
    OpenGlUtilities::drawAabb(mScanVolumeMin, mScanVolumeMax, QColor(0, 0, 255, 255), 3);

    // Draw Octree
    if(mOctree)
    {
        glPointSize(4);
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_POINTS);
        mOctree->handlePoints(); // => glVertex3f()...
        glEnd();
    }

    // Draw waypoints
    const float radius = mShapeSampleSphere->getRadius();
    foreach(const btRigidBody* body, mWayPoints)
    {
        btTransform t;
        body->getMotionState()->getWorldTransform(t);
        OpenGlUtilities::drawSphere(QVector3D(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()), radius, 20, QColor(0,255,0, 32));
    }
    // Draw sampleSpheres
    foreach(const btRigidBody* body, mSampleSpheres)
    {
        btTransform t;
        body->getMotionState()->getWorldTransform(t);
        OpenGlUtilities::drawSphere(QVector3D(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()), radius, 20, QColor(255,0,0, 32));
    }

    // Draw the ghostobject's AABB
    btVector3 min, max;
    mDeletionTriggerGhostObject->getCollisionShape()->getAabb(mDeletionTriggerTransform, min, max);
//    qDebug() << "ghost aabb" << QVector3D(min.x(), min.y(), min.z()) << QVector3D(max.x(), max.y(), max.z());
    OpenGlUtilities::drawAabb(QVector3D(min.x(), min.y(), min.z()), QVector3D(max.x(), max.y(), max.z()), QColor(255, 255, 255));
}

void FlightPlannerPhysics::slotWayPointReached(const QVector3D)
{
}

void FlightPlannerPhysics::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);
    mDeletionTriggerTransform.setOrigin(btVector3(0.0, mScanVolumeMin.y()-10.0, 0.0)); // 5.0 is the half-height of the boxShape
}
