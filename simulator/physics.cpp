#include "physics.h"

Physics::Physics(Simulator *simulator, OgreWidget *ogreWidget) :
//        QThread(simulator),
        QObject(simulator),
        mMutex(QMutex::NonRecursive)
{
    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    mTotalVehicleWeight = 2.270;

    initializeWind();

    mErrorIntegralPitch = mErrorIntegralRoll = 0.0;

    // Bullet initialisation.
    mBtBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
    mBtCollisionConfig = new btDefaultCollisionConfiguration;
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);
    mBtSolver = new btSequentialImpulseConstraintSolver;

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-9.8,0));

    //----------------------------------------------------------
    // Debug drawing!
    //----------------------------------------------------------

    mBtDebugDrawer = new BtOgre::DebugDrawer(mOgreWidget->sceneManager()->getRootSceneNode(), mBtWorld);
    mBtWorld->setDebugDrawer(mBtDebugDrawer);
    mBtDebugDrawer->setDebugMode(3);

    //----------------------------------------------------------
    // Vehicle!
    //----------------------------------------------------------
    mVehicleEntity = mOgreWidget->sceneManager()->createEntity("vehicleEntity", "quadrocopter.mesh");
    // "vehicleNode" is fixed, used in ogrewidget.cpp

    // Place the vehicle somewhere above some building
    if(mOgreWidget->mEntities.size())
        mVehicleNode = mOgreWidget->createVehicleNode("vehicleNode", mOgreWidget->mEntities.values().first()->sceneNode->_getDerivedPosition() + Ogre::Vector3(10, 0, 10), Ogre::Quaternion::IDENTITY);
    else
        mVehicleNode = mOgreWidget->createVehicleNode("vehicleNode", Ogre::Vector3(10, 100, 10), Ogre::Quaternion::IDENTITY);

    mVehicleNode->setPosition(140,90,150);

    mVehicleNode->attachObject(mVehicleEntity);

    mEngineNodes.append(mVehicleNode->createChildSceneNode(Ogre::Vector3(+0.00, +0.00, -0.20), Ogre::Quaternion(Ogre::Degree(000), Ogre::Vector3(1, 0, 0))));  // engine 1, forward, CW
    mEngineNodes.at(0)->attachObject(mOgreWidget->sceneManager()->createEntity("engineF", "engine.mesh"));
    mEngineNodes.at(0)->attachObject(mOgreWidget->sceneManager()->createEntity("labelF", "f.mesh"));

    mEngineNodes.append(mVehicleNode->createChildSceneNode(Ogre::Vector3(+0.00, +0.00, +0.20), Ogre::Quaternion(Ogre::Degree(000), Ogre::Vector3(1, 0, 0))));  // engine 2, backward,  CW
    mEngineNodes.at(1)->attachObject(mOgreWidget->sceneManager()->createEntity("engineB", "engine.mesh"));
    mEngineNodes.at(1)->attachObject(mOgreWidget->sceneManager()->createEntity("labelB", "b.mesh"));

    mEngineNodes.append(mVehicleNode->createChildSceneNode(Ogre::Vector3(-0.20, +0.00, +0.00), Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3(1, 0, 0))));  // engine 3, left,  CCW
    mEngineNodes.at(2)->attachObject(mOgreWidget->sceneManager()->createEntity("engineL", "engine.mesh"));
    mEngineNodes.at(2)->attachObject(mOgreWidget->sceneManager()->createEntity("labelL", "l.mesh"));

    mEngineNodes.append(mVehicleNode->createChildSceneNode(Ogre::Vector3(+0.20, +0.00, +0.00), Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3(1, 0, 0))));  // engine 4, right, CCW
    mEngineNodes.at(3)->attachObject(mOgreWidget->sceneManager()->createEntity("engineR", "engine.mesh"));
    mEngineNodes.at(3)->attachObject(mOgreWidget->sceneManager()->createEntity("labelR", "r.mesh"));

//    qDebug << "Vehicle::Vehicle: absolute position of right left engine:" << mEngineNodes.at()

    // Place the vehicle 10m above the ground - does not seem to work.
    Ogre::TerrainGroup::RayResult rayResult = mOgreWidget->mTerrainGroup->rayIntersects(Ogre::Ray(mVehicleNode->_getDerivedPosition() + Ogre::Vector3(0,1000,0), Ogre::Vector3::NEGATIVE_UNIT_Y));
    if(rayResult.hit)
    {
        mVehicleNode->setPosition(rayResult.position.x, rayResult.position.y + 0.2, rayResult.position.z);
        qDebug() << "Vehicle::Vehicle(): creating vehicle, setting to height" << rayResult.position.y + 20.0;
    }

    // Make camera look at vehicle
//    mOgreWidget->setCameraPosition(
//            mVehicleNode->_getDerivedPosition() + Ogre::Vector3(0.0,2.0,-10.0),
//            OgreWidget::TRANSLATION_ABSOLUTE,
//            Ogre::Node::TS_WORLD,
//            mVehicleNode);

    // Create collision shape
//    BtOgre::StaticMeshToShapeConverter converter(mVehicleEntity);
//    mVehicleShape = converter.createConvex();

    // Reduce vertices (20k to maybe 100?)
//    btShapeHull* hull = new btShapeHull(mVehicleShape);
//    btScalar margin = mVehicleShape->getMargin();
//    hull->buildHull(margin);
//    mVehicleShape = new btConvexHullShape((btScalar*)hull->getVertexPointer(), hull->numVertices()/*, sizeof(btVector3)*/);

    // We set the collision-shape manually. Note the higher height of the device, it'll hopfully help
    // us to catch high-speed collisions better
    /*
    mVehicleShape = new btConvexHullShape;
    mVehicleShape->addPoint(btVector3(0.4, 0.2, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, 0.2, 0.4));
    mVehicleShape->addPoint(btVector3(-0.4, 0.2, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, 0.2, -0.4));

    mVehicleShape->addPoint(btVector3(0.4, -0.1, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, -0.1, 0.4));
    mVehicleShape->addPoint(btVector3(-0.4, -0.1, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, -0.1, -0.4));
    */
    mVehicleShape = new btBoxShape(btVector3(0.35, 0.15, 0.35));

//    mVehicleShape->setLocalScaling(btVector3(2,1,2));

    //Calculate inertia.
    btVector3 inertia;
    mVehicleShape->calculateLocalInertia(mTotalVehicleWeight, inertia);

    //Create BtOgre MotionState (connects Ogre and Bullet).
    mVehicleState = new BtOgre::RigidBodyState(mVehicleNode);
    connect(mVehicleState, SIGNAL(newPose(const Pose&)), SIGNAL(newVehiclePose(const Pose&)));

    //Create the Body.
    mVehicleBody = new btRigidBody(mTotalVehicleWeight, mVehicleState, mVehicleShape, inertia);
    mVehicleBody->setFriction(0.8);
    mVehicleBody->setRestitution(0.6);
//    mVehicleBody->setCollisionFlags(mVehicleBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

    // linear, angular
    mVehicleBody->setDamping(.5, .7);

    mFlightController = new FlightController;

    mVehicleBody->setActivationState(DISABLE_DEACTIVATION); // probably unnecessary
    mBtWorld->addRigidBody(mVehicleBody);
//    mVehicleBody->setActivationState(ISLAND_SLEEPING);

    // Construct the bullet collision shapes.
    QMapIterator<Ogre::Entity*, OgreWidget::MeshInformation*> i(mOgreWidget->mEntities);
    while(i.hasNext())
    {
        i.next();
        Ogre::Entity* e = i.key();
        const Ogre::SceneNode* s = i.value()->sceneNode;

        // Get the mesh from the entity
        Ogre::MeshPtr myMesh = e->getMesh();

        // Get the submesh and associated data
        Ogre::SubMesh* subMesh = myMesh->getSubMesh(0);

        Ogre::IndexData*  indexData = subMesh->indexData;
        Ogre::VertexData* vertexData = subMesh->vertexData;

        // Get the position element
        const Ogre::VertexElement* posElem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

        // Get a pointer to the vertex buffer
        Ogre::HardwareVertexBufferSharedPtr vBuffer = vertexData->vertexBufferBinding->getBuffer(posElem->getSource());

        // Get a pointer to the index buffer
        Ogre::HardwareIndexBufferSharedPtr iBuffer = indexData->indexBuffer;

        // The vertices and indices used to create the triangle mesh
        std::vector<Ogre::Vector3> vertices;
        vertices.reserve(vertexData->vertexCount);

        std::vector<unsigned long> indices;
        indices.reserve(indexData->indexCount);

        // Lock the Vertex Buffer (READ ONLY)
        unsigned char* vertex = static_cast<unsigned char*>(vBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        float* pReal = NULL;

        for(size_t j = 0; j < vertexData->vertexCount; ++j, vertex += vBuffer->getVertexSize())
        {
            posElem->baseVertexPointerToElement(vertex, &pReal);
            Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
            vertices.push_back(pt);
        }

        vBuffer->unlock();

        bool use32bitindexes = (iBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        // Lock the Index Buffer (READ ONLY)
        unsigned long* pLong = static_cast<unsigned long*>(iBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        if(use32bitindexes)
        {
            for(size_t k = 0; k < indexData->indexCount; ++k)
            {
                indices.push_back(pLong[k]);
            }
        }
        else
        {
            for(size_t k = 0; k < indexData->indexCount; ++k)
            {
                indices.push_back(static_cast<unsigned long>(pShort[k]) );
            }
        }

        iBuffer->unlock();

        // We now have vertices and indices ready to go

        // Create the triangle mesh
        btTriangleMesh* triMesh = new btTriangleMesh(use32bitindexes);
        btVector3 vert0, vert1, vert2;
        int i=0;

        // For every triangle
        for (size_t y=0; y<indexData->indexCount/3; y++)
        {
            // Set each vertex
            vert0.setValue(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z);
            vert1.setValue(vertices[indices[i+1]].x, vertices[indices[i+1]].y, vertices[indices[i+1]].z);
            vert2.setValue(vertices[indices[i+2]].x, vertices[indices[i+2]].y, vertices[indices[i+2]].z);

            // Add the triangle into the triangle mesh
            float scale = s->_getDerivedScale().x;
            triMesh->addTriangle(vert0 * scale, vert1 * scale, vert2 * scale);

            // Increase index count
            i += 3;
        }

        // Add the triangle mesh into the Bullet world
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,0,0) );

        // Create the collision shape from the triangle mesh
        btBvhTriangleMeshShape* triMeshShape = new btBvhTriangleMeshShape(triMesh, true);

        btScalar mass(0.0f);
        btVector3 localInertia(0,0,0);

        // Use the default motion state
        btDefaultMotionState* triMotionState = new btDefaultMotionState(startTransform);

        // Create the rigid body
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, triMotionState, triMeshShape, localInertia);
        btRigidBody* triBody = new btRigidBody(rbInfo);

        qDebug() << "setting object to" << s->_getDerivedPosition().x << s->_getDerivedPosition().y << s->_getDerivedPosition().z;

        triBody->getWorldTransform().setOrigin(btVector3(s->_getDerivedPosition().x, s->_getDerivedPosition().y, s->_getDerivedPosition().z));
        triBody->getWorldTransform().setRotation(btQuaternion(s->_getDerivedOrientation().x, s->_getDerivedOrientation().y, s->_getDerivedOrientation().z, s->_getDerivedOrientation().w) );

        // Set additional collision flags
        // triBody->setCollisionFlags(triBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        triBody->setFriction(0.4);

        // Add the body to the dynamics world
        mBtWorld->addRigidBody(triBody);
    }

    // Create terrain collision shape - http://www.ogre3d.org/forums/viewtopic.php?t=58756

     Ogre::TerrainGroup::TerrainIterator ti = mOgreWidget->mTerrainGroup->getTerrainIterator();
     Ogre::Terrain* pTerrain;
     while(ti.hasMoreElements())
     {
         // ugly hack, use last terrain, there should only be one.
         pTerrain = ti.getNext()->instance;
         qDebug() << "one terrain.";
     }

     float* terrainHeightData = pTerrain->getHeightData();
     Ogre::Vector3 terrainPosition = pTerrain->getPosition();

     float * pDataConvert= new float[pTerrain->getSize() * pTerrain->getSize()];
     for(int i=0;i<pTerrain->getSize();i++)
        memcpy(
                    pDataConvert + (pTerrain->getSize() * i), // destination
                    terrainHeightData + pTerrain->getSize() * (pTerrain->getSize()-i-1), // source
                    sizeof(float)*(pTerrain->getSize()) // size
                    );

     float metersBetweenVertices = pTerrain->getWorldSize()/(pTerrain->getSize()-1);
     btVector3 localScaling(metersBetweenVertices, 1, metersBetweenVertices);

//     qDebug() << "terrainsize is" << pTerrain->getSize(),
//     qDebug() << "terrainposition y is" << terrainPosition.y;
//     qDebug() << "terrainpos or old" << terrainPosition.y + (pTerrain->getMaxHeight()-pTerrain->getMinHeight())/2;
//     qDebug() << "terrainpos or new" << terrainPosition.y + (pTerrain->getMaxHeight())/2;
//     qDebug() << "terrainheight min" << pTerrain->getMinHeight();
//     qDebug() << "terrainheight max" << pTerrain->getMaxHeight();

     btHeightfieldTerrainShape* groundShape = new btHeightfieldTerrainShape(
                 pTerrain->getSize(),
                 pTerrain->getSize(),
                 pDataConvert,
                 1.0, // heightScale
                 0.0,// WAS: pTerrain->getMinHeight(), but that yields 0 on first run, 75 when loading cached terrain. So this is a nice hack.
                 pTerrain->getMaxHeight(),
                 1,
                 PHY_FLOAT,
                 true);

     groundShape->setUseDiamondSubdivision(false);
     groundShape->setLocalScaling(localScaling);

     mGroundBody = new btRigidBody(0, new btDefaultMotionState(), groundShape);

     mGroundBody->setFriction(0.4);
     mGroundBody->setRestitution(0.6);

     mGroundBody->getWorldTransform().setOrigin(
                 btVector3(
                     terrainPosition.x,
                     terrainPosition.y + (pTerrain->getMaxHeight()/2),
                     terrainPosition.z));

     mGroundBody->getWorldTransform().setRotation(
                 btQuaternion(
                     Ogre::Quaternion::IDENTITY.x,
                     Ogre::Quaternion::IDENTITY.y,
                     Ogre::Quaternion::IDENTITY.z,
                     Ogre::Quaternion::IDENTITY.w));

     mBtWorld->addRigidBody(mGroundBody);

//     mBtDebugDrawer->step();
}

Physics::~Physics()
{
    // Free Bullet stuff.
    delete mBtSolver;
    delete mBtDispatcher;
    delete mBtCollisionConfig;
    delete mBtBroadphase;
}

void Physics::slotSetMotion(const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height)
{
//    qDebug() << "Physics::slotSetMotion(): updating physics forces with thrust" << thrust << "pitch" << pitch << "roll" << roll << "yaw" << yaw;

    /*
      The MikroKopter moves in his own ways, this is how it translates the given externControls into motion:

       - For thrust, its easy: more thrust, more force upward. This is directly reflected below.

       - For yaw != 0, the kopter will (when seen from above) rotate CW for positive values and CCW for
         negative values. The higher the value, the faster it will rotate.

       - For pitch, the given qint8 is close to the angle the kopter will assume. For example, a value
         of 90 will make it pitch almost 90 degrees forward and HOLD that orientation until the value changes.

       - Same for roll.

       What does this mean for our physics implementation? We do NOT translate the numbers into forces as we
       tried before, as that would make the kopter do loopings for e.g. pitch=90.

       Instead, we have to construct a low-level controller that will apply forces until the kopter is in the
       requested orientation.
      */

    // The Kopter weighs 1750g with full load (gps, atomboard, hokuyo),
    // and the 500mAh 4S weighs 520g, adding up to 2270g. According to
    // http://gallery.mikrokopter.de/main.php/v/tech/Okto2_5000_Payload.gif.html?g2_imageViewsIndex=1,

    // Apply general thrust. A value of 255 means 100% thrust, which means around 10A at full battery
    const float thrustCurrent = (((float)thrust) / 255.0) * 10.0 * (mSimulator->mBattery->voltageCurrent() / mSimulator->mBattery->voltageMax());
    const float thrustScalar = mEngine.calculateThrust(thrustCurrent) * 8;
    const Ogre::Vector3 thrustVectorOgre = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(0, thrustScalar, 0);
    mVehicleBody->applyCentralForce(btVector3(thrustVectorOgre.x, thrustVectorOgre.y, thrustVectorOgre.z));

    // Discharge Battery:
    // When we yaw, pitch or roll, we always take x RPM from one pair of motors and add it to another pair. Since
    // the rpm/current-curve is pretty linear for small changes, we assume that yawing, pitching and rolling do
    // not really affect overall current consumption. Not really valid, but good enough for us.
    mSimulator->mBattery->slotSetDischargeCurrent(thrustCurrent * 8);

    // Yaw!
    // Let us wildly assume that the motors go 2000rpm faster/slower on maximum yaw
    const double torqueScalarYaw = mEngine.calculateTorque(2000 * (((float)yaw) / 128.0));
    Ogre::Vector3 torqueVectorYaw = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(0.0, torqueScalarYaw, 0.0);
    mVehicleBody->applyTorque(btVector3(torqueVectorYaw.x, torqueVectorYaw.y, torqueVectorYaw.z));

    // Controller for pitch and roll: First, we need to know the current pitch and roll:
    Ogre::Matrix3 mat;
    mVehicleNode->_getDerivedOrientation().ToRotationMatrix(mat);
    Ogre::Radian currentYaw, currentPitch, currentRoll;
    mat.ToEulerAnglesYXZ(currentYaw, currentPitch, currentRoll);

    const float timeDiff = std::min(0.2, (float)(mSimulator->getSimulationTime() - mTimeOfLastControllerUpdate) / 1000.0); // elapsed time since last call in seconds
    static float Kp = 2.1;
    static float Ki = 0.4;
    static float Kd = 2.0;

    // Pitch
    const float errorPitch = 0.9*((float)pitch) + currentPitch.valueDegrees();
    mErrorIntegralPitch += errorPitch*timeDiff;
    const float derivativePitch = (errorPitch - mPrevErrorPitch + 0.00001)/timeDiff;
    const float outputPitch = (Kp*errorPitch) + (Ki*mErrorIntegralPitch) + (Kd*derivativePitch);
    mPrevErrorPitch = errorPitch;

    Ogre::Vector3 torqueVectorPitch = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(-outputPitch/10.0, 0.0, 0.0);
    mVehicleBody->applyTorque(btVector3(torqueVectorPitch.x, torqueVectorPitch.y, torqueVectorPitch.z));

    // Roll
    const float errorRoll = 0.9*((float)roll) - currentRoll.valueDegrees();
    mErrorIntegralRoll += errorRoll*timeDiff;
    double derivativeRoll = (errorRoll - mPrevErrorRoll + 0.00001)/timeDiff;
    double outputRoll = (Kp*errorRoll) + (Ki*mErrorIntegralRoll) + (Kd*derivativeRoll);
    mPrevErrorRoll = errorRoll;

    Ogre::Vector3 torqueVectorRoll = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(0.0, 0.0, outputRoll/10.0);
    mVehicleBody->applyTorque(btVector3(torqueVectorRoll.x, torqueVectorRoll.y, torqueVectorRoll.z));

//    qDebug() << "llctrlout p should" << pitch << "is" << currentPitch.valueDegrees() << "error" << errorPitch << "derivative" << derivativePitch << "output" << outputPitch;
//    qDebug() << "llctrlout r should" << roll << "is" << currentRoll.valueDegrees() << "error" << errorRoll << "derivative" << derivativeRoll << "output" << outputRoll;
}

bool Physics::initializeWind()
{
    QFile windDataFile("winddata.txt");
    if(!windDataFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Physics::initializeWind(): couldn't open" << windDataFile.fileName() << ", no wind available";
        emit windInitialized(false);
        return false;
    }

    while(!windDataFile.atEnd())
    {
        QString line(windDataFile.readLine());

        QStringList record = line.split(';');

        // record now contains 7 values:
        // 0: S is always 0
        // 1 2 3: X, Y, Z components of the windvector, in m/s
        // 4: T is temperature
        // 5: V is horizontal windspeed
        // 6: D is wind-direction in degrees

        btVector3 windVector(
                    record.at(1).toFloat(),
                    record.at(2).toFloat(),
                    record.at(3).toFloat()
                    );

        mVectorWind.append(windVector);
    }

    qDebug() << "Physics::initializeWind(): read" << mVectorWind.size() << "wind samples.";
    emit windInitialized(true);
    return true;
}

void Physics::slotSetWindSetting(const bool& enable, const float& factor)
{
    mWindEnable = enable;
    mWindFactor = factor;
}

void Physics::slotUpdateWind()
{
    if(mWindEnable && mWindFactor != 0.0 && !mVectorWind.empty())
    {
        const quint32 sampleNumber = (mSimulator->getSimulationTime() / 100) % mVectorWind.size();

        btVector3 windVector = mVectorWind.at(sampleNumber);
        mVehicleBody->applyCentralForce(windVector * mWindFactor);

//        qDebug() << "Physics::slotUpdateWind(): applying wind sample" << sampleNumber << ":" << windVector.x() << windVector.y() << windVector.z() << "and factor" << mWindFactor;
    }
//    else
//        qDebug() << "Physics::slotUpdateWind(): not applying wind: enabled:" << mWindEnable << "factor" << mWindFactor << "samples:" << mVectorWind.size();
}

float Physics::getHeightAboveGround()
{
    Ogre::TerrainGroup::RayResult rayResult = mOgreWidget->mTerrainGroup->rayIntersects(Ogre::Ray(mVehicleNode->_getDerivedPosition() + Ogre::Vector3(0,1000,0), Ogre::Vector3::NEGATIVE_UNIT_Y));
    if(rayResult.hit)
    {
        return mVehicleNode->_getDerivedPosition().y - rayResult.position.y;
    }

    return -1.0;
}

void Physics::slotUpdatePhysics(void)
{
    slotUpdateWind();

    const int simulationTime = mSimulator->getSimulationTime(); // milliseconds
    const btScalar deltaS = std::max(0.0f, (simulationTime - mTimeOfLastPhysicsUpdate) / 1000.0f); // elapsed time since last call in seconds
    const int maxSubSteps = 20;
    const btScalar fixedTimeStep = 1.0 / 60.0;
//    qDebug() << "Physics::slotUpdatePhysics(): stepping physics, time is" << simulationTime << "delta" << deltaS;

    mVehicleBody->applyDamping(deltaS);

    //Q_ASSERT(deltaS < maxSubSteps * fixedTimeStep); // http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
//    mVehicleBody->applyForce(btVector3(0, 20, 0), btVector3(0, 0, 0));
    mBtWorld->stepSimulation(deltaS, maxSubSteps, fixedTimeStep);
//    mVehicleBody->applyForce(btVector3(0, 20, 0), btVector3(0, 0, 0));

//  mBtDebugDrawer->step();

    mTimeOfLastPhysicsUpdate = simulationTime;

    // Set all laserscanners new position
    // The lidars are attached to the vehicle's scenenode, so stepSimulation should update the vehicle's sceneNode and
    // thus also update the laserscanner's position. BUT the lidars do up to 60k RSQ/s, so pushing these changes
    // should be more efficient than having the LaserScanners poll on every RaySceneQuery.
    QList<LaserScanner*>* laserScanners = mSimulator->getLaserScannerList();
    foreach(LaserScanner* ls, *laserScanners)
    {
//        Ogre::SceneNode* scannerNode = mOgreWidget->sceneManager()->getSceneNode(ls->objectName().append("_node").toStdString());
        Ogre::SceneNode* scannerNode = ls->getSceneNode();
            scannerNode->_update(false, true);
        ls->slotSetScannerPose(scannerNode->_getDerivedPosition(), scannerNode->_getDerivedOrientation());
    }

    // TODO: If we loop with more than 25fps, update more slowly
    // TODO: why the fuck does a timer in vehicle drive the gl update?
    //mOgreWidget->update();
}

QVector3D Physics::getVehicleLinearVelocity() const
{
    const btVector3 vL = mVehicleBody->getLinearVelocity();
    return QVector3D(vL.x(), vL.y(), vL.z());
}

QVector3D Physics::getVehicleAngularVelocity() const
{
    const btVector3 vA = mVehicleBody->getAngularVelocity();
    return QVector3D(vA.x(), vA.y(), vA.z());
}

void Physics::slotSetTotalVehicleWeight(const float& weight)
{
    mTotalVehicleWeight = weight;
}
