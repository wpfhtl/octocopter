#include "vehicle.h"

Vehicle::Vehicle(Simulator *simulator, OgreWidget *ogreWidget) :
//        QThread(simulator),
        QObject(simulator),
        mMutex(QMutex::NonRecursive)
{
    mSimulator = simulator;
    mOgreWidget = ogreWidget;


    mTimerUpdatePosition = new QTimer(this);
    mTimerUpdatePosition->setInterval(1000/60);
    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePosition()));

    // Set up motors, as seen from the top
    // http://www.mikrokopter.de/ucwiki/ElektronikVerkabelung#head-4e6a59d9824e114e97488c7fdaa2b1d75025bae9
//    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, -0.20))));  // engine 1, forward, CW
//    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, +0.20))));  // engine 2, backward,  CW
//    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(+0.20, +0.00, +0.00))));  // engine 3, right, CCW
//    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(-0.20, +0.00, +0.00))));  // engine 4, left,  CCW


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
    mVehicleNode = mOgreWidget->createVehicleNode("vehicleNode", mOgreWidget->mEntities.values().first()->sceneNode->_getDerivedPosition() + Ogre::Vector3(10, 0, 10), Ogre::Quaternion::IDENTITY);
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
    mVehicleShape = new btConvexHullShape;
    mVehicleShape->addPoint(btVector3(0.4, 0.2, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, 0.2, 0.4));
    mVehicleShape->addPoint(btVector3(-0.4, 0.2, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, 0.2, -0.4));

    mVehicleShape->addPoint(btVector3(0.4, -0.1, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, -0.1, 0.4));
    mVehicleShape->addPoint(btVector3(-0.4, -0.1, 0.0));
    mVehicleShape->addPoint(btVector3(0.0, -0.1, -0.4));

//    mVehicleShape->setLocalScaling(btVector3(2,1,2));

    //Calculate inertia.
    btScalar mass = 0.5;
    btVector3 inertia;
    mVehicleShape->calculateLocalInertia(mass, inertia);

    //Create BtOgre MotionState (connects Ogre and Bullet).
    mVehicleState = new BtOgre::RigidBodyState(mVehicleNode);
    connect(mVehicleState, SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)), SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)));

    //Create the Body.
    mVehicleBody = new btRigidBody(mass, mVehicleState, mVehicleShape, inertia);
    mVehicleBody->setFriction(0.8);
    mVehicleBody->setRestitution(0.6);
//    mVehicleBody->setCollisionFlags(mVehicleBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

    // linear, angular
    mVehicleBody->setDamping(.5, .7);

    mFlightController = new FlightController(simulator, this, mVehicleState);

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
                    pDataConvert+pTerrain->getSize() * i, // source
                    terrainHeightData + pTerrain->getSize() * (pTerrain->getSize()-i-1), // target
                    sizeof(float)*(pTerrain->getSize()) // size
                    );

     float metersBetweenVertices = pTerrain->getWorldSize()/(pTerrain->getSize()-1);
     btVector3 localScaling(metersBetweenVertices, 1, metersBetweenVertices);

//     qDebug() << "terrainposition y is" << terrainPosition.y;
//     qDebug() << "terrainpos or old" << terrainPosition.y + (pTerrain->getMaxHeight()-pTerrain->getMinHeight())/2;
//     qDebug() << "terrainpos or new" << terrainPosition.y + (pTerrain->getMaxHeight())/2;
//     qDebug() << "terrainheight min" << pTerrain->getMinHeight();
//     qDebug() << "terrainheight max" << pTerrain->getMaxHeight();

     btHeightfieldTerrainShape* groundShape = new btHeightfieldTerrainShape(
                 pTerrain->getSize(),
                 pTerrain->getSize(),
                 pDataConvert,
                 1/*ignore*/,
                 0.0,// WAS: pTerrain->getMinHeight(), but that yields 0 on first run, 75 when loading cached terrain. So this is a nice hack.
                 pTerrain->getMaxHeight(),
                 1,
                 PHY_FLOAT,
                 true);

     groundShape->setUseDiamondSubdivision(true);
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

Vehicle::~Vehicle()
{
    // Free Bullet stuff.
    delete mBtSolver;
    delete mBtDispatcher;
    delete mBtCollisionConfig;
    delete mBtBroadphase;
}

void Vehicle::run()
{
//    mTimeOfLastUpdate = mSimulator->getSimulationTime();
//    mTimerUpdatePosition->start();
//    QThread::exec();
}

void Vehicle::slotUpdatePosition(void)
{
    QMutexLocker locker(&mMutex);

    int f,b,l,r;

    mFlightController->getEngineSpeeds(f, b, l, r);

    QList<int> motorSpeeds;

    motorSpeeds << f << b << -l << -r;

//    qDebug() << "Vehicle::slotUpdatePosition():\t\t" << "f" << f << "b" << b << "l" << l << "r" << r;

    slotSetMotorSpeeds(motorSpeeds);

    //slotSetWind();

    slotUpdatePhysics();

//    qDebug() << "Vehicle::slotUpdatePosition(): done";
}

//void Vehicle::slotSetNextWayPoint(const CoordinateGps &wayPoint)
//{
//    QMutexLocker locker(&mMutex);

//    mNextWayPoint = mCoordinateConverter.convert(wayPoint);
//}

void Vehicle::slotSetMotorSpeeds(const QList<int> &speeds)
{
    Q_ASSERT(speeds.size() == mEngineNodes.size());

    // We apply forces to the vehicle depending on the speeds of its propellers.
    for(int i=0;i<speeds.size();++i)
    {
        const double thrustScalar = mEngine.calculateThrust(speeds.at(i));

        // f b -l -r

        const Ogre::Vector3 thrustVectorOgre = mEngineNodes.at(i)->_getDerivedOrientation() * Ogre::Vector3(0, thrustScalar, 0);
        const btVector3 thrustVectorBt(thrustVectorOgre.x, thrustVectorOgre.y, thrustVectorOgre.z);
//        const Ogre::Vector3 pos = mEngineNodes.at(i)->_getDerivedPosition() - mVehicleNode->_getDerivedPosition();
        const Ogre::Vector3 pos = mVehicleNode->_getDerivedOrientation() * mEngineNodes.at(i)->getPosition();
        const btVector3 position(pos.x, pos.y, pos.z);
        mVehicleBody->applyForce(thrustVectorBt, position);
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): thrust" << i << thrustScalar << thrustVectorOgre.x << thrustVectorOgre.y << thrustVectorOgre.z << "at" << position.x() << position.y() << position.z();

        const double torqueScalar = mEngine.calculateTorque(speeds.at(i));
        Ogre::Vector3 torque = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(0.0, torqueScalar, 0.0);
        mVehicleBody->applyTorque(btVector3(torque.x, torque.y, torque.z));
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): torque y:" << i << torqueScalar << torque.y;
    }

//    qDebug() << "Vehicle::slotSetMotorSpeeds(): total yaw torque is" << mVehicleBody->getTotalTorque().y();
}

float Vehicle::getHeightAboveGround()
{
    Ogre::TerrainGroup::RayResult rayResult = mOgreWidget->mTerrainGroup->rayIntersects(Ogre::Ray(mVehicleNode->_getDerivedPosition() + Ogre::Vector3(0,1000,0), Ogre::Vector3::NEGATIVE_UNIT_Y));
    if(rayResult.hit)
    {
        return mVehicleNode->_getDerivedPosition().y - rayResult.position.y;
    }

    return -1.0;
}

void Vehicle::slotUpdatePhysics(void)
{
    const int simulationTime = mSimulator->getSimulationTime(); // milliseconds
    const btScalar deltaS = std::max(0.0f, (simulationTime - mTimeOfLastUpdate) / 1000.0f); // elapsed time since last call in seconds
    const int maxSubSteps = 20;
    const btScalar fixedTimeStep = 1.0 / 60.0;
//    qDebug() << "Vehicle::slotUpdatePhysics(): stepping physics, time is" << simulationTime << "delta" << deltaS;

    mVehicleBody->applyDamping(deltaS);

    //Q_ASSERT(deltaS < maxSubSteps * fixedTimeStep); // http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
//    mVehicleBody->applyForce(btVector3(0, 20, 0), btVector3(0, 0, 0));
    mBtWorld->stepSimulation(deltaS, maxSubSteps, fixedTimeStep);
//    mVehicleBody->applyForce(btVector3(0, 20, 0), btVector3(0, 0, 0));

//  mBtDebugDrawer->step();

    mTimeOfLastUpdate = simulationTime;

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

void Vehicle::start(void)
{
    qDebug() << "Vehicle::start(): starting timer.";
    mTimerUpdatePosition->start();
}

void Vehicle::stop(void)
{
    qDebug() << "Vehicle::stop(): stopping timer.";
    mTimerUpdatePosition->stop();
}

void Vehicle::slotShutDown(void)
{
    QMutexLocker locker(&mMutex);

//    quit();
}

QVector3D Vehicle::getLinearVelocity() const
{
    const btVector3 vL = mVehicleBody->getLinearVelocity();
    return QVector3D(vL.x(), vL.y(), vL.z());
}

QVector3D Vehicle::getAngularVelocity() const
{
    const btVector3 vA = mVehicleBody->getAngularVelocity();
    return QVector3D(vA.x(), vA.y(), vA.z());
}

