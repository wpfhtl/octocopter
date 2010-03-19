#include "vehicle.h"

Vehicle::Vehicle(Simulator *simulator, OgreWidget *ogreWidget) :
//        QThread(simulator),
        QObject(simulator),
        mMutex(QMutex::NonRecursive)
{
    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    mTimerUpdatePosition = new QTimer(this);
    mTimerUpdatePosition->setInterval(1000/25);
    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePosition()));

    // Set up motors, as seen from the top
    // http://www.mikrokopter.de/ucwiki/ElektronikVerkabelung#head-4e6a59d9824e114e97488c7fdaa2b1d75025bae9
    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, -0.20))));  // engine 1, front, CW
    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, +0.20))));  // engine 2, back,  CW
    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(+0.20, +0.00, +0.00))));  // engine 3, right, CCW
    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(-0.20, +0.00, +0.00))));  // engine 4, left,  CCW

    //Bullet initialisation.
    mBtBroadphase = new btAxisSweep3(btVector3(-2000,-2000,-2000), btVector3(2000,2000,2000), 1024);
    mBtCollisionConfig = new btDefaultCollisionConfiguration();
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);
    mBtSolver = new btSequentialImpulseConstraintSolver();

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-9.8,0));

    //----------------------------------------------------------
    // Debug drawing!
    //----------------------------------------------------------

    mBtDebugDrawer = new BtOgre::DebugDrawer(mOgreWidget->sceneManager()->getRootSceneNode(), mBtWorld);
    mBtWorld->setDebugDrawer(mBtDebugDrawer);
    mBtDebugDrawer->setDebugMode(1);

    //----------------------------------------------------------
    // Vehicle!
    //----------------------------------------------------------
    qDebug() << "Vehicle::Vehicle(): creating vehicle";
    mVehicleEntity = mOgreWidget->sceneManager()->createEntity("vehicleEntity", "quad.mesh");
    // "vehicleNode" is fixed, used in ogrewidget.cpp
    mVehicleNode = mOgreWidget->sceneManager()->getRootSceneNode()->createChildSceneNode("vehicleNode", Ogre::Vector3(0,10,0), Ogre::Quaternion::IDENTITY);
    mVehicleNode->attachObject(mVehicleEntity);

    //Create shape.
    BtOgre::StaticMeshToShapeConverter converter(mVehicleEntity);
    mVehicleShape = converter.createConvex();

    // Reduce vertices (20k to maybe 100?)
    btShapeHull* hull = new btShapeHull(mVehicleShape);
    btScalar margin = mVehicleShape->getMargin();
    hull->buildHull(margin);
    mVehicleShape = new btConvexHullShape((btScalar*)hull->getVertexPointer(), hull->numVertices()/*, sizeof(btVector3)*/);

    //Calculate inertia.
    btScalar mass = 1.5;
    btVector3 inertia;
    mVehicleShape->calculateLocalInertia(mass, inertia);

    //Create BtOgre MotionState (connects Ogre and Bullet).
    mVehicleState = new BtOgre::RigidBodyState(mVehicleNode);
    connect(mVehicleState, SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)), SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)));

    //Create the Body.
    mVehicleBody = new btRigidBody(mass, mVehicleState, mVehicleShape, inertia);
//    mVehicleBody->setCollisionFlags(mVehicleBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

    mVehicleBody->setDamping(.5, .9);

//    mVehicleBody->setActivationState(ISLAND_SLEEPING);
    mBtWorld->addRigidBody(mVehicleBody/*, COL_VEHICLE, COL_GROUND*/);
//    mVehicleBody->setActivationState(ISLAND_SLEEPING);

    //----------------------------------------------------------
    // Load terrain!
    //----------------------------------------------------------
   std::string terrainFileStr = "terrain.cfg";

   Ogre::DataStreamPtr configStream = Ogre::ResourceGroupManager::getSingleton().openResource(terrainFileStr, Ogre::ResourceGroupManager::getSingleton().getWorldResourceGroupName());
   Ogre::ConfigFile config;
   config.load(configStream);

   Ogre::String widthStr = config.getSetting("PageSize");
   int width = atoi(widthStr.c_str());

   Ogre::String imgFile = config.getSetting("Heightmap.image");

   Ogre::Image* heightmap = new Ogre::Image();
   heightmap->load(imgFile, Ogre::ResourceGroupManager::getSingleton().getWorldResourceGroupName());

   qDebug() << "Vehicle::Vehicle(): image buffer size" << heightmap->getSize();

   Ogre::String maxHeightStr = config.getSetting("MaxHeight");
   btScalar maxHeight = atof(maxHeightStr.c_str());
   btScalar heightScale = maxHeight / 256;

   // This code for the localScaling is taken directly from the TerrainSceneManager, adapted to a btVector3
   btVector3 localScaling(1, 1, 1);
   localScaling.setX(atof(config.getSetting("PageWorldX").c_str()) / (width -1));
   localScaling.setZ(atof(config.getSetting("PageWorldZ").c_str()) / (width -1));

   // And now, we actually call Bullet. heightmap needs to be on the heap, as bullet does not copy it.
   mGroundShape = new btHeightfieldTerrainShape(width, width, heightmap->getData(), heightScale, 0, maxHeight, 1, PHY_UCHAR, false);
   mGroundShape->setLocalScaling(localScaling);

   // All thats left is to line up the Ogre::SceneNode with the btHeightfieldTerrainShape.
   // We have to do it this way because of differences in how Bullet and Ogre orient the shapes.
   // In Ogre, the terrain's top left corner is at (0, 0, 0) in the local TransformSpace
   // In Bullet, not only is the center of the btHeighfield terrrain shape at (0, 0, 0), but from
   //      what I can tell, its immovable.
   btVector3 min, max;
   mGroundShape->getAabb(btTransform::getIdentity(), min, max);
   Ogre::SceneNode *sNode = mOgreWidget->sceneManager()->getSceneNode("Terrain");
   sNode->setPosition(BtOgre::Convert::toOgre(min));

   // Finally, create your btMotionState, and btRigidBody, and all the rigid body to the physics world.
   BtOgre::RigidBodyState* terrainState = new BtOgre::RigidBodyState(sNode, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
   mGroundBody = new btRigidBody(0, terrainState, mGroundShape, btVector3(0, 0, 0));
//   mGroundBody->setCollisionFlags(mGroundBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
   mBtWorld->addRigidBody(mGroundBody, COL_GROUND, COL_NOTHING);

   // Debug, add a plane like the tutorial says
//   btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
//   btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
//   btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
//   btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
//   mBtWorld->addRigidBody(groundRigidBody);

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
//    Q_ASSERT(false);
//    qDebug() << "Vehicle::slotUpdatePosition()";
    QMutexLocker locker(&mMutex);

    /*
    // Impress by interpolating linearly :)
    // TODO: get busy with ODE/Bullet/PhysX

    // that'd be meters/second
    // TODO: use our own time to minimize cross-thread dependiencies
    const float movementSpeed = 0.1;
    const float maxDistanceToTravel = movementSpeed * ((mSimulator->getSimulationTime() - mTimeOfLastUpdate) / 1000.0f);
    mTimeOfLastUpdate = mSimulator->getSimulationTime();
    Ogre::Vector3 currentPosition = mOgreWidget->getVehiclePosition();

    // how far to the current wayPoint?
    float distX = mNextWayPoint.x - currentPosition.x;
    float distY = mNextWayPoint.y - currentPosition.y;
    float distZ = mNextWayPoint.z - currentPosition.z;

    // cap at maxDistanceToTravel
    distX > 0.0f ? distX = fmin(distX, maxDistanceToTravel) : distX = fmax(distX, -maxDistanceToTravel);
    distY > 0.0f ? distY = fmin(distY, maxDistanceToTravel) : distY = fmax(distY, -maxDistanceToTravel);
    distZ > 0.0f ? distZ = fmin(distZ, maxDistanceToTravel) : distZ = fmax(distZ, -maxDistanceToTravel);

    // update position
    currentPosition.x += distX;
    currentPosition.y += distY;
    currentPosition.z += distZ;

    mOgreWidget->setVehiclePosition(currentPosition, OgreWidget::TRANSLATION_ABSOLUTE, Ogre::Node::TS_WORLD);
    */

    const Ogre::Vector3 vectorToTarget = mVehicleNode->_getDerivedPosition() - mNextWayPoint;
//    const float yawToTarget = mVehicleNode->_getDerivedOrientation().getYaw();

    QList<int> motorSpeeds;
    // motorSpeeds << front << back << left << right
    motorSpeeds << 16000 << 16000 << -16000 << -16000;

    slotSetMotorSpeeds(motorSpeeds);

    //slotSetWind();

    slotUpdatePhysics();

//    qDebug() << "Vehicle::slotUpdatePosition(): done";
}

void Vehicle::slotSetNextWayPoint(const CoordinateGps &wayPoint)
{
    QMutexLocker locker(&mMutex);

    mNextWayPoint = mCoordinateConverter.convert(wayPoint);
}

void Vehicle::slotSetMotorSpeeds(const QList<int> &speeds)
{
    Q_ASSERT(speeds.size() <= mEngines.size());

    // We apply forces to the vehicle depending on the speeds of its propellers.

    for(int i=0;i<speeds.size();++i)
    {
        const btVector3 thrust = mEngines.at(i).calculateThrust(speeds.at(i));
        const btVector3 position = mEngines.at(i).getPosition();
        mVehicleBody->applyForce(thrust, position);
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): thrust" << i << thrust.x() << thrust.y() << thrust.z() << "at" << position.x() << position.y() << position.z();

        const btVector3 torque = mEngines.at(i).calculateTorque(speeds.at(i));
        mVehicleBody->applyTorque(torque);
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): torque" << i << torque.x() << torque.y() << torque.z();
    }

//    qDebug() << "Vehicle::slotSetMotorSpeeds(): total yaw torque is" << mVehicleBody->getTotalTorque().y();
}


void Vehicle::slotUpdatePhysics(void)
{
    const int simulationTime = mSimulator->getSimulationTime(); // milliseconds
    const btScalar deltaS = (simulationTime - mTimeOfLastUpdate) / 1000.0f; // elapsed time since last call in seconds
    const int maxSubSteps = 20;
    const btScalar fixedTimeStep = 1.0 / 60.0;
//    qDebug() << "Vehicle::slotUpdatePhysics(): stepping physics, time is" << simulationTime << "delta" << deltaS;

//    mVehicleBody->applyDamping(deltaS);

    Q_ASSERT(deltaS < maxSubSteps * fixedTimeStep); // http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
    mBtWorld->stepSimulation(deltaS, maxSubSteps, fixedTimeStep);

//    mBtDebugDrawer->step();

    mTimeOfLastUpdate = simulationTime;
    mOgreWidget->update();
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
