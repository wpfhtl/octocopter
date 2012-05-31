#include <GL/glew.h>
#include <GL/gl.h>

#include <QApplication>

#include <waypoint.h>
#include "lidarpoint.h"
#include "flightplannerphysics.h"
#include "flightplannerphysicsdialog.h"

float FlightPlannerPhysics::CollisionSphereState::mFarthestDistanceTravelled = 0.0f;

FlightPlannerPhysics::FlightPlannerPhysics(QWidget* widget, Octree* pointCloud) : FlightPlannerInterface(widget, pointCloud)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctreeCollisionObjects = 0;

    mDeletionTriggerShape = 0;
    mDeletionTriggerVbo = 0;
    mSampleSphereVbo = 0;

    mWaypointListMap.insert("generated", new WayPointList(QColor(255,255,0,128)));

    mPhysicsProcessingActive = false;

    mFirstSphereHasHitThisIteration = false;

    mDialog = new FlightPlannerPhysicsDialog(widget);
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
    // register a ghost pair callback to activate the world's ghost functionality
    mBtBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mBtSolver = new btSequentialImpulseConstraintSolver;

    /*
      broadphase detects two objects that have overlapping AABB,
      midphase detects which triangles from each trimesh have overlapping AABB and
      narrowphase does collision detection between these triangles.
    */
    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-10,0));

    mDbgDrawer = new BulletDebugDrawerGl;
    mBtWorld->setDebugDrawer(mDbgDrawer);

    mTransformLidarPoint.setIdentity();

    // Construct spheres for lidarPoints and sampleSpheres
    mLidarPointShape = new btSphereShape(0.05);
    mShapeSampleSphere = new btSphereShape(2.0);

    // Location and shape will be set in slotSetScanVolume();
    mDeletionTriggerGhostObject = new btGhostObject;
    mDeletionTriggerGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE); // So that objects can get inside. Doesnt matter, as they'll be deleted anyways

    // Set up a ghost object with a compound shape, containing all btSphereShapes
    btTransform lidarFloorTransform;
    lidarFloorTransform.setIdentity();
    mPointCloudShape = new btCompoundShape();
    mGhostObjectPointCloud = new btPairCachingGhostObject;
    mGhostObjectPointCloud->setFriction(mDialog->getFrictionGround());
    mGhostObjectPointCloud->setRestitution(mDialog->getRestitutionGround());
    mGhostObjectPointCloud->setWorldTransform(lidarFloorTransform);
    mGhostObjectPointCloud->setCollisionShape(mPointCloudShape);
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
}

void FlightPlannerPhysics::slotSetScanVolume(const QVector3D min, const QVector3D max)
{
    FlightPlannerInterface::slotSetScanVolume(min, max);

    mDeletionTriggerTransform.setIdentity();
    mDeletionTriggerTransform.setOrigin(
                btVector3(
                    mScanVolumeMin.x() + (mScanVolumeMax.x() - mScanVolumeMin.x()) / 2.0f,
                    mScanVolumeMin.y() - 10.0f,
                    mScanVolumeMin.z() + (mScanVolumeMax.z() - mScanVolumeMin.z()) / 2.0f
                    )
                );

    // Recreate deletionTriggerShape with correct size.
    delete mDeletionTriggerShape;
    mDeletionTriggerShape = new btBoxShape(
                btVector3( // These are HALF-extents!
                           (mScanVolumeMax.x() - mScanVolumeMin.x()) * 1.5f,
                           10.0f,
                           (mScanVolumeMax.z() - mScanVolumeMin.z()) * 1.5f)
                );
    mDeletionTriggerGhostObject->setWorldTransform(mDeletionTriggerTransform);
    mDeletionTriggerGhostObject->setCollisionShape(mDeletionTriggerShape);

    // Add mDeletionTriggerGhostObject only once (if its not already present).
    if(mBtWorld->getCollisionObjectArray().findLinearSearch(mDeletionTriggerGhostObject) == mBtWorld->getCollisionObjectArray().size())
        mBtWorld->addCollisionObject(mDeletionTriggerGhostObject, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

    // delete the deletionTrigger VBO, so it will be recreated when rendering the next frame
    if(mDeletionTriggerVbo)
    {
        glDeleteBuffers(1, &mDeletionTriggerVbo);
        mDeletionTriggerVbo = 0;
    }
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
    const WayPoint wpt = mWaypointListMap.value("ahead")->first();
    const btVector3 wayPointPosition(wpt.x(), wpt.y(), wpt.z());

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
    // Nothing. We need to override, as its pure virtual in base.
}

FlightPlannerPhysics::~FlightPlannerPhysics()
{
    // Cleanup in the reverse order of creation/initialization

    // Remove the rigidbodies from the dynamics world and delete them
    for(int i=mBtWorld->getNumCollisionObjects()-1; i>=0 ;i--)
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

void FlightPlannerPhysics::insertPoint(LidarPoint* point)
{
    if(mOctreeCollisionObjects == 0)
    {
        // Create the octree around the first arriving point.
        mOctreeCollisionObjects = new Octree(
                    point->position - QVector3D(10, 10, 10), // min
                    point->position + QVector3D(10, 10, 10),  // max
                    10, //maxPerLeaf
                    20000 // expectedMax
                    );

        mOctreeCollisionObjects->mPointColor = QColor(255,0,0,200);
        mOctreeCollisionObjects->setMinimumPointDistance(1.6);
        connect(mOctreeCollisionObjects, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointInserted(const LidarPoint*)));
    }

    mOctreeCollisionObjects->insertPoint(point);
}

void FlightPlannerPhysics::slotGenerateWaypoints()
{
    // Just show the dialog for generating-options
    mDialog->show();
}

// A point was inserted ito our octree. Create a corresponding point in our physics world.
void FlightPlannerPhysics::slotPointInserted(const LidarPoint* lp)
{
    // We add this lidarPoint as a sphereShape child to the mPointCloudShape, which is a btCompoundShape
    mTransformLidarPoint.setIdentity();
    mTransformLidarPoint.setOrigin(btVector3(lp->position.x(), lp->position.y(), lp->position.z()));
    mPointCloudShape->addChildShape(mTransformLidarPoint, mLidarPointShape);
}

void FlightPlannerPhysics::slotVisualize()
{
//    mBtWorld->debugDrawWorld();
    FlightPlannerInterface::slotVisualize();

    // Draw the collision octree
    if(mOctreeCollisionObjects)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);
        mShaderProgramDefault->setUniformValue("fixedColor",
                                               QVector4D(
                                                   mOctreeCollisionObjects->mPointColor.redF(),
                                                   mOctreeCollisionObjects->mPointColor.greenF(),
                                                   mOctreeCollisionObjects->mPointColor.blueF(),
                                                   mOctreeCollisionObjects->mPointColor.alphaF()
                                                   )
                                               );

        mOctreeCollisionObjects->updateVbo();

        // Render pointcloud using all initialized VBOs (there might be none when no points exist)
        QMapIterator<quint32, quint32> i(mOctreeCollisionObjects->mVboIdsAndSizes);
        while(i.hasNext())
        {
            i.next();
            glBindBuffer(GL_ARRAY_BUFFER, i.key());
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, 0);
            glDrawArrays(GL_POINTS, 0, i.value()); // Number of Elements, not bytes
            glDisableVertexAttribArray(0);
        }

        mShaderProgramDefault->release();
    }

    // Set/reset/draw the detection volume/groundplane/deletion trigger
    if(!mDeletionTriggerVbo)
    {
        btVector3 min, max;
        mDeletionTriggerShape->getAabb(mDeletionTriggerTransform, min, max);

        QVector<float> deletionTriggerVertices;
        // Fill the vertices buffer with vertices for quads and lines
        deletionTriggerVertices
                // 1 back
                << min.x() << min.y() << min.z() << 1.0f
                << max.x() << min.y() << min.z() << 1.0f
                << max.x() << max.y() << min.z() << 1.0f
                << min.x() << max.y() << min.z() << 1.0f

                // 2 front
                << max.x() << min.y() << max.z() << 1.0f
                << min.x() << min.y() << max.z() << 1.0f
                << min.x() << max.y() << max.z() << 1.0f
                << max.x() << max.y() << max.z() << 1.0f

                // 3 left
                << min.x() << min.y() << max.z() << 1.0f
                << min.x() << min.y() << min.z() << 1.0f
                << min.x() << max.y() << min.z() << 1.0f
                << min.x() << max.y() << max.z() << 1.0f

                // 4 right
                << max.x() << min.y() << min.z() << 1.0f
                << max.x() << min.y() << max.z() << 1.0f
                << max.x() << max.y() << max.z() << 1.0f
                << max.x() << max.y() << min.z() << 1.0f

                // 6 top
                << min.x() << max.y() << min.z() << 1.0f
                << max.x() << max.y() << min.z() << 1.0f
                << max.x() << max.y() << max.z() << 1.0f
                << min.x() << max.y() << max.z() << 1.0f

                // 5 bottom
                << min.x() << min.y() << max.z() << 1.0f
                << max.x() << min.y() << max.z() << 1.0f
                << max.x() << min.y() << min.z() << 1.0f
                << min.x() << min.y() << min.z() << 1.0f;

        glGenBuffers(1, &mDeletionTriggerVbo);
        glBindBuffer(GL_ARRAY_BUFFER, mDeletionTriggerVbo);

        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * (deletionTriggerVertices.size() /*+ deletionTriggerColors.size()*/), NULL, GL_STATIC_DRAW);

        glBufferSubData(
                    GL_ARRAY_BUFFER,
                    0, // offset in the VBO
                    deletionTriggerVertices.size() * sizeof(float), // how many bytes to store?
                    (void*)(deletionTriggerVertices.constData()) // data to store
                    );
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    /* diable drawing the detection olume for now
    mShaderProgramDefault->bind();
    mShaderProgramDefault->setUniformValue("useFixedColor", true);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
    {
        glBindBuffer(GL_ARRAY_BUFFER, mDeletionTriggerVbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position

        // draw the lines around the box
        mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 0.2f, 0.2f, 0.8f));
        glDrawArrays(GL_LINE_LOOP, 0, 4);
        glDrawArrays(GL_LINE_LOOP, 4, 4);
        glDrawArrays(GL_LINE_LOOP, 8, 4);
        glDrawArrays(GL_LINE_LOOP, 12, 4);
        glDrawArrays(GL_LINE_LOOP, 16, 4);
        glDrawArrays(GL_LINE_LOOP, 20, 4);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDisableVertexAttribArray(0);
    }
    glDisable(GL_BLEND);
    mShaderProgramDefault->release();*/

    // Draw sampleSpheres
    if(mSampleSphereVbo)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        mShaderProgramSpheres->bind();
        mShaderProgramSpheres->setUniformValue("useFixedColor", true);
        mShaderProgramSpheres->setUniformValue("fixedColor", QVector4D(0.6f, 0.6f, 1.0f, 0.5f));
        mShaderProgramSpheres->setUniformValue("particleRadius", mDialog->getSampleSphereRadius());
        glBindBuffer(GL_ARRAY_BUFFER, mSampleSphereVbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0); // position
        glDrawArrays(GL_POINTS, 0, mSampleObjects.size());
        glDisableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        mShaderProgramSpheres->release();
        glDisable(GL_BLEND);
    }
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

    // The geometry we create needs to be rendered. So create a VBO for it.
    if(!mSampleSphereVbo) glGenBuffers(1, &mSampleSphereVbo);

    if(mDialog->getGenerationType() == FlightPlannerPhysicsDialog::GenerateRain)
    {
        // Fill the sky with spheres
        for(
            float x = mScanVolumeMin.x() + mShapeSampleSphere->getRadius() * 1.2f; // small padding
            x <= mScanVolumeMax.x() - mShapeSampleSphere->getRadius() * 1.2f; // small padding
            x += 2.0f * mShapeSampleSphere->getRadius() + 0.1f /*Margin to prevent unnecessary collisions between sample geometry*/)
        {
            for(
                float z = mScanVolumeMin.z() + mShapeSampleSphere->getRadius() * 1.2f; // small padding
                z <= mScanVolumeMax.z() - mShapeSampleSphere->getRadius() * 1.2f; // small padding
                z += 2.0f * mShapeSampleSphere->getRadius() + 0.1f /*Margin to prevent unnecessary collisions*/)
            {
                sampleSphereTransform.setOrigin(btVector3(x, mScanVolumeMax.y()-(mShapeSampleSphere->getRadius()*1.2f), z));
                // We don't need any inertia, mass etc,. as this body is static.
                CollisionSphereState* sampleSphereMotionState = new CollisionSphereState(sampleSphereTransform);
                btRigidBody* body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mShapeSampleSphere->getRadius(), sampleSphereMotionState, mShapeSampleSphere));
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
            const QVector3D pos = mVehiclePoses.last().getPosition();
            sampleSphereTransform.setOrigin(btVector3(pos.x(), pos.y(), pos.z()));

            // We don't need any inertia, mass etc,. as this body is static.
            CollisionSphereState* sampleSphereMotionState = new CollisionSphereState(sampleSphereTransform);
            btRigidBody* body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mShapeSampleSphere->getRadius(), sampleSphereMotionState, mShapeSampleSphere));
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

    glBindBuffer(GL_ARRAY_BUFFER, mSampleSphereVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(QVector3D) * mSampleObjects.size(), NULL, GL_DYNAMIC_DRAW);
    // Update the VBO
    for (int index=0; index<mSampleObjects.size(); ++index)
    {
        ((CollisionSphereState*)(mSampleObjects.at(index)->getMotionState()))->updateVbo(mSampleSphereVbo, index);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    mDialog->slotAppendMessage(QString("Created %1 sample geometry objects with size %2, we now have %3 objects").arg(numberOfObjectsCreated).arg(mDialog->getSampleSphereRadius()).arg(mSampleObjects.size()));

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotDeleteSampleGeometry()
{
    // remove the old sample geometry...
    mDialog->slotAppendMessage(QString("Deleting %1 leftover sample geometry objects").arg(mSampleObjects.size()));

    foreach(btRigidBody* body, mSampleObjects)
    {
        mBtWorld->removeRigidBody(body);
        mSampleObjects.removeOne(body);
        delete body->getMotionState();
        delete body;
    }

    glDeleteBuffers(1, &mSampleSphereVbo);

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
        // And now let them fall from the sky...
        QTime wayPointGenerationStartTime = QTime::currentTime();
        for(int physicsIterations = 0;true;physicsIterations++)
        {
            // Using QApplication::processEvents(), we give the app a chance to call this method while this method is running(!)
            // to set mPhysicsProcessingActive to false. If that happened, we will use this chance to break our processing.
            if(!mPhysicsProcessingActive)
            {
                mDialog->slotAppendMessage("Stopping physics processing.");
                mDialog->slotSetProgress(0, 0, 100);
                break;
            }

            // Reset the motion threshold. When everything becomes slow, terminate
            CollisionSphereState::mFarthestDistanceTravelled = 0.0f;

            mBtWorld->applyGravity();

            QTime profiler = QTime::currentTime(); profiler.start();
            mBtWorld->stepSimulation(0.05, 10);
            qDebug() << "FlightPlannerPhysics::slotProcessPhysics(): step cpu time in ms:" << profiler.elapsed() << "fps:" << 1000 / std::max(profiler.elapsed(),1);
            usleep(500000);

            printf("%1.7f\n", CollisionSphereState::mFarthestDistanceTravelled); fflush(stdout);
            if(physicsIterations > 50 && CollisionSphereState::mFarthestDistanceTravelled < 0.0001f)
            {
                mPhysicsProcessingActive = false;
                mDialog->slotAppendMessage(QString("Waypoint generation took %1 iterations and %2 ms for sparse pointcloud size %3")
                                           .arg(physicsIterations)
                                           .arg(wayPointGenerationStartTime.elapsed())
                                           .arg(mOctreeCollisionObjects->getNumberOfItems()));
                slotSubmitGeneratedWayPoints();
                break;
            }

            // According to http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=3901, we need this
            // for the ghostobject to perform narrowphase collision detection. And yes, that seems to make a difference!
            mBtWorld->getDispatcher()->dispatchAllCollisionPairs(
                        mGhostObjectPointCloud->getOverlappingPairCache(),
                        mBtWorld->getDispatchInfo(),
                        mBtWorld->getDispatcher());

            // Ask the PointCloud GhostObject (representing the low-res octree with lidarPoints) whether anything hit it. If yes,
            // record what SampleSphere hit the pointCloud at what position.
            btManifoldArray   manifoldArray;
            const btBroadphasePairArray& pairArray = mGhostObjectPointCloud->getOverlappingPairCache()->getOverlappingPairArray();
            for(int j=0; j<pairArray.size(); ++j)
            {
                manifoldArray.clear();
                const btBroadphasePair& pair = pairArray[j];


                // Don't get the pair from the world, get it from the GhotObject! Otherwise, collisions seem to be with pointcloud AABB!
                //btBroadphasePair* collisionPair = mBtWorld->getPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
                btBroadphasePair* collisionPair = mGhostObjectPointCloud->getOverlappingPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
                if(!collisionPair)
                    continue;

                const btBroadphaseProxy* proxy = pair.m_pProxy0->m_clientObject != mGhostObjectPointCloud ? pair.m_pProxy0 : pair.m_pProxy1;
                btRigidBody *rigidBody = dynamic_cast<btRigidBody*>((btCollisionObject*)(proxy->m_clientObject));
                if(!rigidBody) continue;

                if(collisionPair->m_algorithm)
                    collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

                for(int j=0;j<manifoldArray.size();j++)
                {
                    btPersistentManifold* manifold = manifoldArray[j];
                    for (int p=0;p<manifold->getNumContacts();p++)
                    {
                        const btManifoldPoint&pt = manifold->getContactPoint(p);
                        if(pt.getDistance()<0.f)
                        {
                            const btVector3& ptA = pt.getPositionWorldOnA();
                            const QVector3D pos(ptA.x(), ptA.y() + mShapeSampleSphere->getRadius(), ptA.z());
                            mLastSampleObjectHitPositions.insert(rigidBody, pos);
                        }
                    }
                }

            }
//            mBtWorld->removeCollisionObject(mGhostObjectPointCloud);

            if(mDeletionTriggerGhostObject->getNumOverlappingObjects())
            {
                //qDebug() << "number of objects fallen through in this iteration:" << mGhostObjectDeletionTrigger->getNumOverlappingObjects();
                if(!mFirstSphereHasHitThisIteration && false)
                {
                    // first sphere! halt the processing and give time for a screenshot! Just for the paper...
                    mFirstSphereHasHitThisIteration = true;
                    mPhysicsProcessingActive = false;
                    break;
                }
            }

            // We iterate through all rigidBodies touching our deletionTrigger and remove/delete them.
            for(int j = 0; j < mDeletionTriggerGhostObject->getNumOverlappingObjects(); j++)
            {
                // Dynamic cast to make sure its a rigid body
                btRigidBody *rigidBody = dynamic_cast<btRigidBody *>(mDeletionTriggerGhostObject->getOverlappingObject(j));
                if(rigidBody)
                {
                    //qDebug() << "removing sphere that has dropped too far.";

                    // THIS IS THE INTERESTING PART! IF THE SPHERE HAS PREVIOUSLY HIT A LIDARPOINT, MAKE THAT A NEW WAYPOINT!
                    if(mLastSampleObjectHitPositions.contains(rigidBody))
                    {
                        WayPoint w(mLastSampleObjectHitPositions.take(rigidBody) + QVector3D(0.0, 2.0f * mShapeSampleSphere->getRadius(), 0.0));

                        // only use waypoints in scanvolume
                        if(
                                w.x() < mScanVolumeMax.x()
                                && w.y() < mScanVolumeMax.y()
                                && w.z() < mScanVolumeMax.z()
                                && w.x() > mScanVolumeMin.x()
                                && w.y() > mScanVolumeMin.y()
                                && w.z() > mScanVolumeMin.z()
                                )
                        {

                            // Do not append waypoints in close vicinity to other waypoints!
                            // We check ALL waypoint lists here: passed, generated, ...
                            bool wayPointInCloseVicinity = false;

                            QMapIterator<QString, WayPointList*> i(mWaypointListMap);
                            while(!wayPointInCloseVicinity && i.hasNext())
                            {
                                i.next();
                                const QList<WayPoint>* list = i.value()->list();
                                for(int i = 0; i < list->size() && !wayPointInCloseVicinity; ++i)
                                {
                                    if(list->at(i).distanceToLine(w, QVector3D()) < mShapeSampleSphere->getRadius())
                                    {
                                        wayPointInCloseVicinity = true;
                                    }
                                }
                            }

                            if(!wayPointInCloseVicinity) mWaypointListMap["generated"]->append(w);

                            // This would add the waypoint sphere as static object to the physics world
                            //sampleSphereTransform.setOrigin(btVector3(w.x(), w.y(), w.z()));
                            // We don't need any inertia, mass etc,. as this body is static.
                            //btDefaultMotionState* sampleSpherePointMotionState = new btDefaultMotionState(sampleSphereTransform);
                            //btRigidBody::btRigidBodyConstructionInfo rbInfo(0, sampleSpherePointMotionState, mShapeSampleSphere, btVector3(0,0,0));
                            //btRigidBody* bodyNew = new btRigidBody(rbInfo);
                            //bodyNew->setFriction(0.01);

                            // Add the body to the dynamics world
                            //mWayPoints.append(bodyNew);
                            //mBtWorld->addRigidBody(bodyNew);
                        }
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

            if(mDialog->visualizationActive() && mSampleSphereVbo)
            {
                // update the sample-sphere-VBO with all motionstate data
                glBindBuffer(GL_ARRAY_BUFFER, mSampleSphereVbo);
                for(int index=0; index<mSampleObjects.size(); ++index)
                    ((CollisionSphereState*)(mSampleObjects.at(index)->getMotionState()))->updateVbo(mSampleSphereVbo, index);
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                emit suggestVisualization();
            }
            QApplication::processEvents();
        }
    }
}

void FlightPlannerPhysics::slotSubmitGeneratedWayPoints()
{
    mDialog->slotAppendMessage(QString("%1 waypoints present, adding another %2, then sorting.").arg(mWaypointListMap.value("ahead")->size()).arg(mWaypointListMap.value("generated")->size()));

    mWaypointListMap["ahead"]->append(mWaypointListMap.value("generated"));
    mWaypointListMap["generated"]->clear();

    if(mVehiclePoses.size())
        mWaypointListMap["ahead"]->sortToShortestPath(mVehiclePoses.last().getPosition());
    else
        mWaypointListMap["ahead"]->sortToShortestPath(QVector3D(0,0,0));

    emit wayPointsSetOnRover(*mWaypointListMap.value("ahead")->list());
    emit wayPoints(*mWaypointListMap.value("ahead")->list());

    // just for creating paper-screenshots
    slotDeleteSampleGeometry();
    mFirstSphereHasHitThisIteration = false;

    mLastSampleObjectHitPositions.clear();

    emit suggestVisualization();
}

void FlightPlannerPhysics::slotDeleteGeneratedWayPoints()
{
    mDialog->slotAppendMessage(QString("%1 generated waypoints deleted on user request.").arg(mWaypointListMap.value("generated")->size()));

    mWaypointListMap["generated"]->clear();

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

// This Needs to be in cpp for glew include, which would mess up compilation in the header
void FlightPlannerPhysics::CollisionSphereState::updateVbo(const quint32 vbo, const quint32 index)
{
    glBufferSubData(
                GL_ARRAY_BUFFER,
                index * 3 * sizeof(float), // offset in the VBO
                3 * sizeof(float), // how many bytes to store?
                (void*)(&mTransform.getOrigin()) // data to store. btVector3 has 4 floats, but we use just 3
                );
}
