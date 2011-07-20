#include "flightplannerphysics.h"

FlightPlannerPhysics::FlightPlannerPhysics(QWidget* widget, const Pose * const pose, Octree* pointCloud) : FlightPlannerInterface(widget, pose, pointCloud)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctree = 0;

    mPhysicsProcessingActive = false;

    mDialog = new FlightPlannerPhysicsDialog(mParentWidget);
    connect(mDialog, SIGNAL(createSampleGeometry()), SLOT(slotCreateSampleGeometry()));
    connect(mDialog, SIGNAL(deleteSampleGeometry()), SLOT(slotDeleteSampleGeometry()));
    connect(mDialog, SIGNAL(processPhysics(bool)), SLOT(slotProcessPhysics(bool)));
    connect(mDialog, SIGNAL(submitWayPoints()), SLOT(slotEmitWayPoints()));

    connect(mDialog, SIGNAL(gravityChanged(QVector3D)), SLOT(slotGravityChanged(QVector3D)));

    // Bullet initialisation.
    mBtBroadphase = new btDbvtBroadphase; //new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024); // = new btDbvtBroadphase();
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mBtSolver = new btSequentialImpulseConstraintSolver;

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);

//    mDbgDrawer = new BulletDebugDrawerGl;
//    mBtWorld->setDebugDrawer(mDbgDrawer);

    mLidarPointTransform.setIdentity();

    // Construct spheres for lidarPoints and sampleSpheres
    mLidarPointShape = new btSphereShape(0.1);
    mShapeSampleSphere = new btSphereShape(2);

    // Set up a ghost object that deletes SampleSpheres when they hit it.
    mDeletionTriggerTransform.setIdentity();
    mDeletionTriggerTransform.setOrigin(btVector3(mScanVolumeMin.x(), mScanVolumeMin.y()-10.0, mScanVolumeMin.z()));

    mDeletionTriggerShape = new btBoxShape(
                btVector3( // These are HALF-extents!
                    (mScanVolumeMax.x() - mScanVolumeMin.x()) / 2.0,
                    10,
                    (mScanVolumeMax.z() - mScanVolumeMin.z()) / 2.0));

    mDeletionTriggerGhostObject = new btGhostObject;
    mDeletionTriggerGhostObject->setWorldTransform(mDeletionTriggerTransform);
    mDeletionTriggerGhostObject->setCollisionShape(mDeletionTriggerShape);
    mDeletionTriggerGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mDeletionTriggerGhostObject, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // Set up a ghost object with a compound shape, containing all btSphereShapes
    btTransform lidarFloorTransform;
    lidarFloorTransform.setIdentity();

    mPointCloudShape = new btCompoundShape(false);

    mPointCloudGhostObject = new btPairCachingGhostObject();
    mPointCloudGhostObject->setFriction(10.0);
//    mLidarFloorGhostObject->setRestitution(?);
    mPointCloudGhostObject->setWorldTransform(lidarFloorTransform);
    mPointCloudGhostObject->setCollisionShape(mPointCloudShape);
//    mLidarFloorGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mPointCloudGhostObject, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

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
    // First, just show the dialog for generating-options
    mDialog->show();

    // Now the hard work:
    //
    // 1. Drop small spheres (big enough for the vehicle to fit in). Delete them if they sleep on other spheres.
    // 2. Let it dry until everyone is sleeping and falling spheres are deleted
    // 3. Drop big spheres (little smaller than lidar range). Delete small spheres being touched by big spheres.
    // 4. Let it dry until everyone is sleeping and falling spheres are deleted
    // 5. Freeze the spheres to let them remain for future iterations.
    // 6. Find a nice path through the remaining spheres.

    // The sphere that'll be dropped from the sky.
}

// A point was inserted ito our octree. Create a corresponding point in our physics world.
void FlightPlannerPhysics::slotPointInserted(const LidarPoint* lp)
{
    // We add this lidarPoint as a sphereShape child to the mLidarFloorShape, which is a btCompoundShape
    mLidarPointTransform.setOrigin(btVector3(lp->position.x(), lp->position.y(), lp->position.z()));
    mPointCloudShape->addChildShape(mLidarPointTransform, mLidarPointShape);
}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerPhysics::slotVisualize() const
{
    mBtWorld->debugDrawWorld();

    // Draw the scanVolume
    glDisable(GL_LIGHTING);
    OpenGlUtilities::drawAabb(mScanVolumeMin, mScanVolumeMax, QColor(150, 150, 255, 150), 2);
//    glEnable(GL_LIGHTING);

    // Draw Octree
    if(mOctree)
    {
        glPointSize(4);
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
//        glDisable(GL_LIGHTING);
        glBegin(GL_POINTS);
        mOctree->handlePoints(); // => glVertex3f()...
        glEnd();
//        glEnable(GL_LIGHTING);
    }

        glEnable(GL_BLEND);							// Enable Blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);			// Type Of Blending To Use
    //    glBlendFunc( GL_SRC_ALPHA, GL_ONE );
//        glBlendFunc( GL_ZERO, GL_ONE_MINUS_SRC_ALPHA );
    //    glBlendFunc( GL_SRC_ALPHA_SATURATE, GL_ONE );
    //    glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);


    const float radius = mShapeSampleSphere->getRadius();

    // Draw passed waypoints
    foreach(const WayPoint& wpt, *mWayPointsPassed)
        OpenGlUtilities::drawSphere(wpt, radius, 20, QColor(128,255,128, 32));

    // Draw fresh waypoints
    foreach(const WayPoint& wpt, mWayPointsGenerated)
        OpenGlUtilities::drawSphere(wpt, radius, 20, QColor(255,128,128, 32));

    // Draw future waypoints
    foreach(const WayPoint& wpt, *mWayPointsAhead)
        OpenGlUtilities::drawSphere(wpt, radius, 20, QColor(255,255,128, 64));

    // Draw sampleSpheres
    foreach(const btRigidBody* body, mSampleObjects)
    {
        btTransform t;
        body->getMotionState()->getWorldTransform(t);
        OpenGlUtilities::drawSphere(QVector3D(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()), radius, 20, QColor(128,128,255, 32));
    }

    glDisable(GL_BLEND);

    // Draw the ghostobject's AABB
    btVector3 min, max;
    mDeletionTriggerGhostObject->getCollisionShape()->getAabb(mDeletionTriggerTransform, min, max);
//    qDebug() << "ghost aabb" << QVector3D(min.x(), min.y(), min.z()) << QVector3D(max.x(), max.y(), max.z());
//    glDisable(GL_LIGHTING);
    OpenGlUtilities::drawAabb(QVector3D(min.x(), min.y(), min.z()), QVector3D(max.x(), max.y(), max.z()), QColor(255,150,150, 150), 2);
    glEnable(GL_LIGHTING);
}

void FlightPlannerPhysics::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);

    mDeletionTriggerTransform.setOrigin(
                btVector3(
                    mScanVolumeMin.x() + (mScanVolumeMax.x() - mScanVolumeMin.x()) / 2.0,
                    mScanVolumeMin.y()-5.0,
                    mScanVolumeMin.z() + (mScanVolumeMax.z() - mScanVolumeMin.z()) / 2.0
                    )
                );

    // recreate deletionTriggerShape with correct size.
    delete mDeletionTriggerShape;

    mDeletionTriggerShape = new btBoxShape(
                btVector3( // These are HALF-extents!
                    (mScanVolumeMax.x() - mScanVolumeMin.x()) / 1.5,
                    5,
                    (mScanVolumeMax.z() - mScanVolumeMin.z()) / 1.5)
                );
    mDeletionTriggerGhostObject->setCollisionShape(mDeletionTriggerShape);
    mDeletionTriggerGhostObject->setWorldTransform(mDeletionTriggerTransform);
}

void FlightPlannerPhysics::slotGravityChanged(const QVector3D& gravity)
{
    mBtWorld->setGravity(
                btVector3(
                    gravity.x(),
                    gravity.y(),
                    gravity.z())
                );
}

void FlightPlannerPhysics::slotCreateSampleGeometry()
{
    mShapeSampleSphere->setUnscaledRadius(mDialog->getSampleSphereRadius());

    btTransform sampleSphereTransform;

    quint16 numberOfObjectsCreated = 0;

    // Fill the sky with spheres
    for(
        float x = mScanVolumeMin.x() + mShapeSampleSphere->getRadius()/2.0;
        x <= mScanVolumeMax.x() - mShapeSampleSphere->getRadius()/2.0;
        x += 2 * mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions between sample geometry*/)
    {
        for(
            float z = mScanVolumeMin.z() + mShapeSampleSphere->getRadius()/2.0;
            z <= mScanVolumeMax.z() - mShapeSampleSphere->getRadius()/2.0;
            z += 2*mShapeSampleSphere->getRadius() + 0.1 /*Margin to prevent unnecessary collisions*/)
        {
            sampleSphereTransform.setOrigin(btVector3(x, mScanVolumeMax.y()+mShapeSampleSphere->getRadius(), z));
            // We don't need any inertia, mass etc,. as this body is static.
            btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mShapeSampleSphere->getRadius(), sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setFriction(0.01);

            numberOfObjectsCreated++;

            // Add the body to the dynamics world
            mSampleObjects.append(body);
            mBtWorld->addRigidBody(body);
        }
    }

    mDialog->slotAppendMessage(QString("Created %1 sample geometry objects with size %2, we now have %3 objects").arg(numberOfObjectsCreated).arg(mDialog->getSampleSphereRadius()).arg(mSampleObjects.size()));

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotDeleteSampleGeometry()
{
    // remove the old sample geometry...
    mDialog->slotAppendMessage(QString("Deleting %1 sample geometry objects").arg(mSampleObjects.size()));

    foreach(btRigidBody* body, mSampleObjects)
    {
        mBtWorld->removeRigidBody(body);
        mSampleObjects.removeOne(body);
    }

    mLastSampleObjectHitPositions.clear();

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotProcessPhysics(bool process)
{
    mPhysicsProcessingActive = process;
    if(mPhysicsProcessingActive)
    {
        mDialog->slotAppendMessage("Starting physics processing.");
        mDialog->slotSetProgress(0, 0, 0);
    }

    if(mPhysicsProcessingActive)
    {
//        btTransform sampleSphereTransform;

        // And now let them fall from the sky...
        while(true)
        {
            // Using QApplication::processEvents(), we give the app a chance to call this method while this method is running(!)
            // to set mPhysicsProcessingActive to false. If that happened, we will use this chance to break our processing.
            if(!mPhysicsProcessingActive)
            {
                mDialog->slotAppendMessage("Stopping physics processing.");
                mDialog->slotSetProgress(0, 0, 100);
                break;
            }

            mBtWorld->applyGravity();
            mBtWorld->stepSimulation(0.01, 10);

            // Ask the PointCloud GhostObject (representing the low-res octree with lidarPoints) whether anything hit it. If yes,
            // record what SampleSphere hit the pointCloud at what position.
            btBroadphasePairArray& pairs = mPointCloudGhostObject->getOverlappingPairCache()->getOverlappingPairArray();
            for (int j=0; j<pairs.size(); ++j)
            {
                const btBroadphasePair& pair = pairs[j];
                btBroadphaseProxy* proxy = pair.m_pProxy0->m_clientObject != mPointCloudGhostObject ? pair.m_pProxy0 : pair.m_pProxy1;

                //            btCollisionObject* obj = (btCollisionObject*)proxy->m_clientObject;
                btRigidBody *rigidBody = dynamic_cast<btRigidBody *>((btCollisionObject*)(proxy->m_clientObject));
                if(rigidBody)
                {
                    btTransform t = rigidBody->getWorldTransform();
                    QVector3D pos(t.getOrigin().x(), t.getOrigin().y()+5, t.getOrigin().z());
                    //                qDebug() << "a samplesphere has hit the lidarground at" << pos;
                    mLastSampleObjectHitPositions.insert(rigidBody, pos);
                }
                else qDebug() << "dynamic cast failed!";
            }


            /* OLD WAY, ONLY AABB IS CHECKED
        // We iterate through all SampleSpheres touching our floor and remember WHERE they were when they touched the floor.
        for(int j = 0; j < mPointCloudGhostObject->getNumOverlappingObjects(); j++)
        {
           // Dynamic cast to make sure its a rigid body
           btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mPointCloudGhostObject->getOverlappingObject(j));
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
                qDebug() << "number of objects fallen through in this iteration:" << mDeletionTriggerGhostObject->getNumOverlappingObjects();

            // We iterate through all rigidBodies touching our deletionTrigger and remove/delete them.
            for(int j = 0; j < mDeletionTriggerGhostObject->getNumOverlappingObjects(); j++)
            {
                // Dynamic cast to make sure its a rigid body
                btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mDeletionTriggerGhostObject->getOverlappingObject(j));
                if(rigidBody)
                {
                    qDebug() << "removing sphere that has dropped too far.";

                    // THIS IS THE INTERESTING PART! IF THE SPHERE HAS PREVIOUSLY HIT A LIDARPOINT, MAKE THAT A NEW WAYPOINT!
                    if(mLastSampleObjectHitPositions.contains(rigidBody))
                    {
                        WayPoint w(mLastSampleObjectHitPositions.value(rigidBody) + QVector3D(0.0, 5 * mShapeSampleSphere->getRadius(), 0.0));
                        mWayPointsGenerated.append(w);

                        // This would add the waypoint sphere as static object to the physics world
//                        sampleSphereTransform.setOrigin(btVector3(w.x(), w.y(), w.z()));
//                        // We don't need any inertia, mass etc,. as this body is static.
//                        btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
//                        btRigidBody::btRigidBodyConstructionInfo rbInfo(0, sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
//                        btRigidBody* bodyNew = new btRigidBody(rbInfo);
//                        bodyNew->setFriction(0.01);

                        // Add the body to the dynamics world
//                        mWayPoints.append(bodyNew);
//                        mBtWorld->addRigidBody(bodyNew);
                    }

                    btTransform pos;
                    rigidBody->getMotionState()->getWorldTransform(pos);
                    mDialog->slotAppendMessage(QString("Sample Geometry touched deletion trigger at %1 %2 %3, deleting.").arg(pos.getOrigin().x()).arg(pos.getOrigin().y()).arg(pos.getOrigin().z()));
                    mBtWorld->removeRigidBody(rigidBody);
                    mSampleObjects.removeOne(rigidBody);
                    delete rigidBody->getMotionState();
                    delete rigidBody;
                }
            }

            emit suggestVisualization();
            QApplication::processEvents();
        }
    }
}

void FlightPlannerPhysics::slotEmitWayPoints()
{
    mDialog->slotAppendMessage(QString("%1 waypoints present, adding another %2, then sorting.").arg(mWayPointsAhead->size()).arg(mWayPointsGenerated.size()));

    mWayPointsAhead->append(mWayPointsGenerated);
    mWayPointsGenerated.clear();
    sortToShortestPath(*mWayPointsAhead, mVehiclePose->position);
    emit wayPointsCleared();
    emit wayPoints(*mWayPointsAhead);

    emit suggestVisualization();
}
