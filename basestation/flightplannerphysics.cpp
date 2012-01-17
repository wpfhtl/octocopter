#include "flightplannerphysics.h"

FlightPlannerPhysics::FlightPlannerPhysics(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctreeCollisionObjects = 0;

    mPhysicsProcessingActive = false;

    mFirstSphereHasHitThisIteration = false;

    mDialog = new FlightPlannerPhysicsDialog(mParentWidget);
    connect(mDialog, SIGNAL(createSampleGeometry()), SLOT(slotCreateSampleGeometry()));
    connect(mDialog, SIGNAL(deleteSampleGeometry()), SLOT(slotDeleteSampleGeometry()));
    connect(mDialog, SIGNAL(processPhysics(bool)), SLOT(slotProcessPhysics(bool)));
    connect(mDialog, SIGNAL(createSafePath()), SLOT(slotCreateSafePathToNextWayPoint()));
    connect(mDialog, SIGNAL(submitWayPoints()), SLOT(slotSubmitGeneratedWayPoints()));
    connect(mDialog, SIGNAL(deleteWayPoints()), SLOT(slotDeleteGeneratedWayPoints()));

    connect(mDialog, SIGNAL(gravityChanged(QVector3D)), SLOT(slotGravityChanged(QVector3D)));
    connect(mDialog, SIGNAL(frictionChanged(float,float)), SLOT(slotFrictionChanged(float,float)));
    connect(mDialog, SIGNAL(restitutionChanged(float,float)), SLOT(slotRestitutionChanged(float,float)));

    // Bullet initialisation.
    mBtBroadphase = new btDbvtBroadphase; //new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024); // = new btDbvtBroadphase();
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mBtSolver = new btSequentialImpulseConstraintSolver;

    /* from irc:
      broadphase detects eg. two trimeshes that have overlapping AABB,
      midphase detects which triangles from each trimesh have overlapping AABB and
      narrowphase does collision detection between these triangles. */
    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-10,0));

    mDbgDrawer = new BulletDebugDrawerGl;
    mBtWorld->setDebugDrawer(mDbgDrawer);

    mTransformLidarPoint.setIdentity();

    // Construct spheres for lidarPoints and sampleSpheres
    mLidarPointShape = new btSphereShape(0.1);
    mShapeSampleSphere = new btSphereShape(2.0);

    // Set up a ghost object that deletes SampleSpheres when they hit it.
    mTransformDeletionTrigger.setIdentity();
    mTransformDeletionTrigger.setOrigin(btVector3(mScanVolumeMin.x(), mScanVolumeMin.y()-10.0, mScanVolumeMin.z()));

    mDeletionTriggerShape = new btBoxShape(
                btVector3( // These are HALF-extents!
                    (mScanVolumeMax.x() - mScanVolumeMin.x()) / 2,
                    10,
                    (mScanVolumeMax.z() - mScanVolumeMin.z()) / 2));

    mGhostObjectDeletionTrigger = new btGhostObject;
    mGhostObjectDeletionTrigger->setWorldTransform(mTransformDeletionTrigger);
    mGhostObjectDeletionTrigger->setCollisionShape(mDeletionTriggerShape);
    mGhostObjectDeletionTrigger->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mGhostObjectDeletionTrigger, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // Set up a ghost object with a compound shape, containing all btSphereShapes
    btTransform lidarFloorTransform;
    lidarFloorTransform.setIdentity();

    mPointCloudShape = new btCompoundShape(false);

    mGhostObjectPointCloud = new btPairCachingGhostObject;
    mGhostObjectPointCloud->setFriction(mDialog->getFrictionGround());
    mGhostObjectPointCloud->setRestitution(mDialog->getRestitutionGround());
    mGhostObjectPointCloud->setWorldTransform(lidarFloorTransform);
    mGhostObjectPointCloud->setCollisionShape(mPointCloudShape);
//    mGhostObjectPointCloud->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    mBtWorld->addCollisionObject(mGhostObjectPointCloud, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // Set up vehicle collision avoidance. We create one normal btRigidBody to interact with the world and one ghost object to register collisions
    mTransformVehicle.setIdentity();
//    mShapeVehicle = new btSphereShape(2.0);
    mMotionStateVehicle = new btDefaultMotionState;
    mBodyVehicle = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(1.0, mMotionStateVehicle, new btSphereShape(2.5), btVector3(0, 0, 0)));
//    mBtWorld->addRigidBody(mBodyVehicle);

    mGhostObjectVehicle = new btPairCachingGhostObject;
    mGhostObjectVehicle->setFriction(10.0);
    mGhostObjectVehicle->setRestitution(0.5);
    mGhostObjectVehicle->setWorldTransform(mTransformVehicle);
    mGhostObjectVehicle->setCollisionShape(mBodyVehicle->getCollisionShape());
//    mBtWorld->addCollisionObject(mGhostObjectVehicle, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // register a ghost pair callback to activate the world's ghost functionality
    mBtBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
}

void FlightPlannerPhysics::slotCreateSafePathToNextWayPoint()
{
    /* We want to check whether the direct-line-of-sight between vehicle and next waypoint is free-to-fly
       or needs detour-waypoints inserted for safe navigation */
    Q_ASSERT(false);
    Q_ASSERT(!mPhysicsProcessingActive && "Physics processing (for waypoint generation?) is active, this collides with creating safe paths, which also use physics.");

    // First, delete leftover sample geometry, this just slows down physics and causes unnecessary collisions with vehicle ghost object
    slotDeleteSampleGeometry();

    // Set the vehicle to the vehicle's current position
//    btTransform transformVehicle = getLastKnownVehiclePose().getTransform();
//    mBodyVehicle->getMotionState()->setWorldTransform(transformVehicle);
//    mBodyVehicle->getMotionState()->getWorldTransform(transformVehicle);
//    mGhostObjectVehicle->setWorldTransform(transformVehicle);
    qDebug() << "initializing vehicle body and ghost object to" << mTransformVehicle.getOrigin().x() << mTransformVehicle.getOrigin().y() << mTransformVehicle.getOrigin().z();

    // Save gravity for later, then disable it.
//    const btVector3 gravityOld = mBtWorld->getGravity();
    mBtWorld->setGravity(btVector3(0,0,0));

    // Create the next waypoint in the physics world
    const btVector3 wayPointPosition(mWayPointsAhead->first().x(), mWayPointsAhead->first().y(), mWayPointsAhead->first().z());
    btRigidBody* wayPointBody = new btRigidBody(
                btRigidBody::btRigidBodyConstructionInfo(
                    0.0,
                    new btDefaultMotionState(btTransform(btQuaternion(), wayPointPosition)),
                    new btSphereShape(1.0),
                    btVector3(0,0,0))
                );

    mBtWorld->addRigidBody(wayPointBody);
    mBtWorld->addRigidBody(mBodyVehicle);

    bool wayPointReached = false;

    while(!wayPointReached)
    {
        // Make the vehicle's body rotate so that it will bounce upwards on collision.
//        mBodyVehicle->setAngularVelocity(btVector3(1,2,3)); // TODO

        // Apply a constant force to push the vehicle towards the waypoint
//        mBodyVehicle->getMotionState()->getWorldTransform(transformVehicle);
        const btVector3 vectorToWayPoint = (wayPointPosition - mTransformVehicle.getOrigin()).normalized();
        qDebug() << "applying force" << vectorToWayPoint.x() << vectorToWayPoint.y() << vectorToWayPoint.z();
//        mBodyVehicle->applyCentralForce(vectorToWayPoint*10);
        mBodyVehicle->applyCentralForce(btVector3(0,10,0));

        usleep(500000);
        suggestVisualization();

        btVector3 force = mBodyVehicle->getTotalForce();
        qDebug() << "total force on mBodyVehicle is" << force.x() << force.y() << force.z();

        // Step the world. The vehicle will move
        mBtWorld->stepSimulation(0.01, 10); // ???

        usleep(500000);
        suggestVisualization();

        // Move the vehicle ghost object to the vehicle's rigidBody
//        done in slotVehiclePoseChanged now mBodyVehicle->getMotionState()->getWorldTransform(transformVehicle);
        mTransformVehicle = mBodyVehicle->getWorldTransform();
        qDebug() << "after step, mBodyVehicle has moved to" << mTransformVehicle.getOrigin().x() << mTransformVehicle.getOrigin().y() << mTransformVehicle.getOrigin().z();
        mGhostObjectVehicle->setWorldTransform(mTransformVehicle);

        force = mBodyVehicle->getTotalForce();
        qDebug() << "total force on mBodyVehicle is" << force.x() << force.y() << force.z();

        /*
        // Ask the Vehicle GhostObject whether anything hit it. If yes, create a waypoint slightly backwards.
        btBroadphasePairArray& pairs = mGhostObjectVehicle->getOverlappingPairCache()->getOverlappingPairArray();
        qDebug() << pairs.size() << "pairs, vehicle body at" << mTransformVehicle.getOrigin().x() << mTransformVehicle.getOrigin().y() << mTransformVehicle.getOrigin().z();
        for(int j=0; j<pairs.size(); ++j)
        {
            const btBroadphasePair& pair = pairs[j];
            btBroadphaseProxy* proxy = pair.m_pProxy0->m_clientObject != mGhostObjectVehicle ? pair.m_pProxy0 : pair.m_pProxy1;

            //            btCollisionObject* obj = (btCollisionObject*)proxy->m_clientObject;
            btRigidBody *rigidBody = dynamic_cast<btRigidBody *>((btCollisionObject*)(proxy->m_clientObject));
            if(rigidBody)
            {
                // The GhostObject did hit @rigidBody.
                const btVector3 pos = rigidBody->getWorldTransform().getOrigin();
                if(rigidBody = wayPointBody)
                {
                    // The Vehicle has hit the waypoint, we're done!
                    qDebug() << "FlightPlannerPhysics::createSafePathToNextWayPoint(): vehicle hit waypoint in simulation, path is now clear, returning.";
                    mDialog->slotAppendMessage(QString("Vehicle hit waypoint in simulation, path is clear after adding %1 detour waypoints.").arg(mWayPointsDetour.size()));

                    // Prepend our detour-waypoints to the mWayPointsAhead-list
                    for(int i=0; i< mWayPointsDetour.size(); i++)
                        mWayPointsAhead->insert(i, mWayPointsDetour.at(i));
                    mWayPointsDetour.clear();
                    wayPointReached = true;
                    break;
                }
                else
                {
                    // The vehicle body has hit an object other than the waypoint, probably a lidar point.
                    // For now, just create a waypoint in this position.
                    mDialog->slotAppendMessage("Adding a detour waypoint for safe navigation.");
                    mWayPointsDetour.append(
                                WayPoint(
                                    QVector3D(pos.x(), pos.y(), pos.z()),
                                    WayPoint::DETOUR
                                    )
                                );
                    usleep(1000000);
                }
            }
            else qDebug() << "dynamic cast failed!";
        }*/

        QApplication::processEvents();

        usleep(10000);
        suggestVisualization();
    }

    // Clean up physics world and heap
    mBtWorld->removeRigidBody(mBodyVehicle);
    mBtWorld->removeRigidBody(wayPointBody);
    delete wayPointBody->getMotionState();
    delete wayPointBody->getCollisionShape();

//    mBtWorld->setGravity(gravityOld);
}

void FlightPlannerPhysics::slotInitialize()
{
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

    if(mOctreeCollisionObjects) delete mOctreeCollisionObjects;
}

void FlightPlannerPhysics::insertPoint(LidarPoint* const point)
{
    if(mOctreeCollisionObjects == 0)
    {
        // Create the octree around the first arriving point.
        mOctreeCollisionObjects = new Octree(
                point->position - QVector3D(10, 10, 10), // min
                point->position + QVector3D(10, 10, 10),  // max
                10);

        mOctreeCollisionObjects->setMinimumPointDistance(1.0);

        mOctreeCollisionObjects->setPointHandler(OpenGlUtilities::drawPoint);

        connect(mOctreeCollisionObjects, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointInserted(const LidarPoint*)));
    }

    mOctreeCollisionObjects->insertPoint(point);
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
    mTransformLidarPoint.setOrigin(btVector3(lp->position.x(), lp->position.y(), lp->position.z()));
    mPointCloudShape->addChildShape(mTransformLidarPoint, mLidarPointShape);
}

// NOT in a glBegin()/glEnd() pair.
void FlightPlannerPhysics::slotVisualize() const
{
//    mBtWorld->debugDrawWorld();
    FlightPlannerInterface::slotVisualize();

    // Draw Octree
    if(mOctreeCollisionObjects)
    {
        glPointSize(4);
        glColor4f(1.0f, 0.0f, 0.0f, 0.8f);
//        glDisable(GL_LIGHTING);
        glBegin(GL_POINTS);
        mOctreeCollisionObjects->handlePoints(); // => glVertex3f()...
        glEnd();
//        glEnable(GL_LIGHTING);
    }

        glEnable(GL_BLEND);							// Enable Blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);			// Type Of Blending To Use
    //    glBlendFunc( GL_SRC_ALPHA, GL_ONE );
//        glBlendFunc( GL_ZERO, GL_ONE_MINUS_SRC_ALPHA );
    //    glBlendFunc( GL_SRC_ALPHA_SATURATE, GL_ONE );
    //    glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);

    // draw next waypoint
    if(mWayPointsAhead->size())
    {
    //        qDebug() << "drawing next wpt at" << mWayPointsAhead->first();
        OpenGlUtilities::drawSphere(mWayPointsAhead->first(), 0.8, 20.0, QColor(255,0,0,100));
    }


    const float radiusSampleGeometry = mShapeSampleSphere->getRadius();
    const float radiusWptScan = 1.5;
    const float radiusWptDetour = 1.0;

    // Draw passed waypoints
    foreach(const WayPoint& wpt, *mWayPointsPassed)
        if(wpt.purpose == WayPoint::SCAN)
            OpenGlUtilities::drawSphere(wpt, radiusWptScan, 20, QColor(128,255,128, 255));
        else
            OpenGlUtilities::drawSphere(wpt, radiusWptDetour, 20, QColor(128,255,128, 255));

    // Draw freshly generated waypoints
    foreach(const WayPoint& wpt, mWayPointsGenerated)
        if(wpt.purpose == WayPoint::SCAN)
            OpenGlUtilities::drawSphere(wpt, radiusWptScan, 20, QColor(255,128,128, 220));
        else
            OpenGlUtilities::drawSphere(wpt, radiusWptDetour, 20, QColor(255,128,128, 220));

    // Draw future waypoints
    foreach(const WayPoint& wpt, *mWayPointsAhead)
        if(wpt.purpose == WayPoint::SCAN)
            OpenGlUtilities::drawSphere(wpt, radiusWptScan, 20, QColor(255,255,128, 255));
        else
            OpenGlUtilities::drawSphere(wpt, radiusWptDetour, 20, QColor(255,255,128, 255));

    // Draw detour waypoints
    foreach(const WayPoint& wpt, mWayPointsDetour)
        OpenGlUtilities::drawSphere(wpt, radiusWptDetour, 20, QColor(0,255,0, 64));

    // Draw sampleSpheres
    foreach(const btRigidBody* body, mSampleObjects)
    {
        btTransform t;
        body->getMotionState()->getWorldTransform(t);
        OpenGlUtilities::drawSphere(QVector3D(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()), radiusSampleGeometry, 20, QColor(128,128,255, 100));
    }

    // Draw the vehicle's physics body
    btTransform tf;
    mMotionStateVehicle->getWorldTransform(tf);
    OpenGlUtilities::drawSphere(
                QVector3D(
                    tf.getOrigin().x(),
                    tf.getOrigin().y(),
                    tf.getOrigin().z()
                    ),
                1.1,
                1,
                QColor(255,0,0, 255)
                );

//    qDebug() << "motionStateVehicle is at" << tf.getOrigin().x() << tf.getOrigin().y() << tf.getOrigin().z();
//    qDebug() << "mTransformVehicle is at" << mTransformVehicle.getOrigin().x() << mTransformVehicle.getOrigin().y() << mTransformVehicle.getOrigin().z();
//    btTransform hans = mGhostObjectVehicle->getWorldTransform();
//    qDebug() << "mGhostObjectVehicle is at" << hans.getOrigin().x() << hans.getOrigin().y() << hans.getOrigin().z();

    glDisable(GL_BLEND);

    // Draw the ghostobject-AABBs
    btVector3 min, max;
    mGhostObjectDeletionTrigger->getCollisionShape()->getAabb(mTransformDeletionTrigger, min, max);
    OpenGlUtilities::drawAabb(QVector3D(min.x(), min.y(), min.z()), QVector3D(max.x(), max.y(), max.z()), QColor(255,150,150, 150), 2);

    mGhostObjectVehicle->getCollisionShape()->getAabb(mTransformVehicle, min, max);
//    OpenGlUtilities::drawAabb(QVector3D(min.x(), min.y(), min.z()), QVector3D(max.x(), max.y(), max.z()), QColor(0,250,0, 150), 2);
    glEnable(GL_LIGHTING);
}

void FlightPlannerPhysics::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);

    mTransformDeletionTrigger.setOrigin(
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
                    (mScanVolumeMax.x() - mScanVolumeMin.x()) / 1.8,
                    5,
                    (mScanVolumeMax.z() - mScanVolumeMin.z()) / 1.8)
                );
    mGhostObjectDeletionTrigger->setCollisionShape(mDeletionTriggerShape);
    mGhostObjectDeletionTrigger->setWorldTransform(mTransformDeletionTrigger);
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

void FlightPlannerPhysics::slotFrictionChanged(const float& frictionPointCloudGround, const float& frictionSampleGeometry)
{
    mGhostObjectPointCloud->setFriction(frictionPointCloudGround);

    for(int i=0;i<mSampleObjects.size();i++)
    {
        mSampleObjects.at(i)->setFriction(frictionSampleGeometry);
    }
}

void FlightPlannerPhysics::slotRestitutionChanged(const float& restitutionPointCloudGround, const float& restitutionSampleGeometry)
{
    mGhostObjectPointCloud->setRestitution(restitutionPointCloudGround);

    for(int i=0;i<mSampleObjects.size();i++)
    {
        mSampleObjects.at(i)->setRestitution(restitutionSampleGeometry);
    }
}

void FlightPlannerPhysics::slotCreateSampleGeometry()
{
    mShapeSampleSphere->setUnscaledRadius(mDialog->getSampleSphereRadius());

    btTransform sampleSphereTransform;

    quint16 numberOfObjectsCreated = 0;

    if(mDialog->getGenerationType() == FlightPlannerPhysicsDialog::GenerateRain)
    {

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
                btRigidBody* body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mShapeSampleSphere->getRadius(), sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0)));
                body->setFriction(mDialog->getFrictionSampleGeometry());
                body->setRestitution(mDialog->getRestitutionSampleGeometry());

                numberOfObjectsCreated++;

                // Add the body to the dynamics world
                mSampleObjects.append(body);
                mBtWorld->addRigidBody(body);
            }
        }
    }
    else if(mDialog->getGenerationType() == FlightPlannerPhysicsDialog::GenerateShootFromVehicle)
    {
        // Shoot from Vehicle
        for(int i=0;i<mDialog->getEmitCount();i++)
        {
            sampleSphereTransform.setOrigin(
                        btVector3(
                            mVehiclePoses.last().position.x(),
                            mVehiclePoses.last().position.y(),
                            mVehiclePoses.last().position.z())
                        );

            // We don't need any inertia, mass etc,. as this body is static.
            btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mShapeSampleSphere->getRadius(), sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setFriction(mDialog->getFrictionSampleGeometry());
            body->setRestitution(mDialog->getRestitutionSampleGeometry());

            QVector3D emitVelocity = mDialog->getEmitVelocity();

            if(mDialog->useRelativeVelocity())
            {
                emitVelocity += getCurrentVehicleVelocity();
            }

            body->setLinearVelocity(btVector3(emitVelocity.x(), emitVelocity.y(), emitVelocity.z()));

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
        delete body->getMotionState();
        delete body;
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
            btBroadphasePairArray& pairs = mGhostObjectPointCloud->getOverlappingPairCache()->getOverlappingPairArray();
            for (int j=0; j<pairs.size(); ++j)
            {
                const btBroadphasePair& pair = pairs[j];
                btBroadphaseProxy* proxy = pair.m_pProxy0->m_clientObject != mGhostObjectPointCloud ? pair.m_pProxy0 : pair.m_pProxy1;

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


            if(mGhostObjectDeletionTrigger->getNumOverlappingObjects())
            {
//                qDebug() << "number of objects fallen through in this iteration:" << mGhostObjectDeletionTrigger->getNumOverlappingObjects();
                if(!mFirstSphereHasHitThisIteration && false)
                {
                    // first sphere! halt the processing and give time for a screenshot! Just for the paper...
                    mFirstSphereHasHitThisIteration = true;
                    mPhysicsProcessingActive = false;
                    break;
                }
            }

            // We iterate through all rigidBodies touching our deletionTrigger and remove/delete them.
            for(int j = 0; j < mGhostObjectDeletionTrigger->getNumOverlappingObjects(); j++)
            {
                // Dynamic cast to make sure its a rigid body
                btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mGhostObjectDeletionTrigger->getOverlappingObject(j));
                if(rigidBody)
                {
//                    qDebug() << "removing sphere that has dropped too far.";

                    // THIS IS THE INTERESTING PART! IF THE SPHERE HAS PREVIOUSLY HIT A LIDARPOINT, MAKE THAT A NEW WAYPOINT!
                    if(mLastSampleObjectHitPositions.contains(rigidBody))
                    {
                        WayPoint w(mLastSampleObjectHitPositions.take(rigidBody) + QVector3D(0.0, /*5 * mShapeSampleSphere->getRadius()*/7.5, 0.0));

                        if( // only use waypoints in scanvolume
                                   w.x() < mScanVolumeMax.x()
                                && w.y() < mScanVolumeMax.y()
                                && w.z() < mScanVolumeMax.z()
                                && w.x() > mScanVolumeMin.x()
                                && w.y() > mScanVolumeMin.y()
                                && w.z() > mScanVolumeMin.z()
                        )
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

            if(mDialog->visualizationActive()) emit suggestVisualization();
            QApplication::processEvents();
        }
    }
}

void FlightPlannerPhysics::slotSubmitGeneratedWayPoints()
{
    mDialog->slotAppendMessage(QString("%1 waypoints present, adding another %2, then sorting.").arg(mWayPointsAhead->size()).arg(mWayPointsGenerated.size()));

    mWayPointsAhead->append(mWayPointsGenerated);
    mWayPointsGenerated.clear();
    sortToShortestPath(*mWayPointsAhead, mVehiclePoses.last().position);
    emit wayPointsSetOnRover(*mWayPointsAhead);
    emit wayPoints(*mWayPointsAhead);

    // just for creating paper-screenshots
   slotDeleteSampleGeometry();
    mFirstSphereHasHitThisIteration = false;

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotDeleteGeneratedWayPoints()
{
    mDialog->slotAppendMessage(QString("%1 generated waypoints deleted on user request.").arg(mWayPointsGenerated.size()));

    mWayPointsGenerated.clear();

    // just for creating paper-screenshots
    mFirstSphereHasHitThisIteration = false;

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotWayPointReached(const WayPoint wpt)
{
    FlightPlannerInterface::slotWayPointReached(wpt);

//    slotCreateSafePathToNextWayPoint();
}

void FlightPlannerPhysics::slotVehiclePoseChanged(const Pose& pose)
{
    FlightPlannerInterface::slotVehiclePoseChanged(pose);

    mTransformVehicle = pose.getTransform();
    mBodyVehicle->setWorldTransform(mTransformVehicle);
    mMotionStateVehicle->setWorldTransform(mTransformVehicle);
    mGhostObjectVehicle->setWorldTransform(mTransformVehicle);
    Q_ASSERT(mMotionStateVehicle == mBodyVehicle->getMotionState());

    // Check for collisions using the ghost object?!
}

qint64 FlightPlannerPhysics::getNumberOfPointsInCollisionOctree()
{
    if(mOctreeCollisionObjects != 0)
        return mOctreeCollisionObjects->getNumberOfItems();
    else
        return 0;
}
