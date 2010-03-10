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
//    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePosition()));
    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePhysics()));

    // Set up motors
    mMotorPositions.append(Ogre::Vector3(+0.00, +0.00, -0.20));  // engine 1, front
    mMotorPositions.append(Ogre::Vector3(+0.20, +0.00, +0.00));  // engine 2, right
    mMotorPositions.append(Ogre::Vector3(+0.00, +0.00, +0.20));  // engine 3, back
    mMotorPositions.append(Ogre::Vector3(-0.20, +0.00, +0.00));  // engine 4, left

    //Bullet initialisation.
    mBtBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
    mBtCollisionConfig = new btDefaultCollisionConfiguration();
    mBtDispatcher = new btCollisionDispatcher(mBtCollisionConfig);
    mBtSolver = new btSequentialImpulseConstraintSolver();

    mBtWorld = new btDiscreteDynamicsWorld(mBtDispatcher, mBtBroadphase, mBtSolver, mBtCollisionConfig);
    mBtWorld->setGravity(btVector3(0,-9.8,0));


    //Some normal stuff.
//    mOgreWidget->sceneManager()->setAmbientLight(ColourValue(0.7,0.7,0.7));
//    mCamera->setPosition(Vector3(10,50,10));
//    mCamera->lookAt(Vector3::ZERO);
//    mCamera->setNearClipDistance(0.05);
//    LogManager::getSingleton().setLogDetail(LL_BOREME);


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
//    mVehicleNode->scale(0.035,0.035,0.035);
//    mVehicleNode->rotate(Ogre::Vector3::UNIT_X, Ogre::Degree(90), Ogre::Node::TS_LOCAL);
    mVehicleNode->attachObject(mVehicleEntity);

    //Create shape.
    BtOgre::StaticMeshToShapeConverter converter(mVehicleEntity);
    mVehicleShape = converter.createConvex();

    // Reduce vertices (20k to maybe 100?)
    btShapeHull* hull = new btShapeHull(mVehicleShape);
    btScalar margin = mVehicleShape->getMargin();
    hull->buildHull(margin);
    /*btConvexHullShape**/ mVehicleShape = new btConvexHullShape((btScalar*)hull->getVertexPointer(), hull->numVertices()/*, sizeof(btVector3)*/);


    qDebug() << "Vehicle::Vehicle(): number of points in vehicle shape:" << ((btConvexHullShape*)mVehicleShape)->getNumPoints();

    // Scale
//    btVector3 scale(0.035,0.035,0.035);
//    mVehicleShape->setLocalScaling(scale);

    //Calculate inertia.
    btScalar mass = 1.5;
    btVector3 inertia;
    mVehicleShape->calculateLocalInertia(mass, inertia);

    //Create BtOgre MotionState (connects Ogre and Bullet).
    BtOgre::RigidBodyState *vehicleState = new BtOgre::RigidBodyState(mVehicleNode);

    //Create the Body.
    mVehicleBody = new btRigidBody(mass, vehicleState, mVehicleShape, inertia);
//    mVehicleBody->setCollisionFlags(mVehicleBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

//    mVehicleBody->setActivationState(ISLAND_SLEEPING);
    mBtWorld->addRigidBody(mVehicleBody/*, COL_VEHICLE, COL_GROUND*/);
//    mVehicleBody->setActivationState(ISLAND_SLEEPING);

    //----------------------------------------------------------
    // Load terrain!
    //----------------------------------------------------------
   std::string terrainFileStr = "terrain.cfg";

   // ogreWidget does that, back off!
   //mOgreWidget->sceneManager()->setWorldGeometry( terrainFileStr );


   // terrainFileStr is a string, representing the file name of *.cfg file to  be used by TerrainSceneManager

   // Load the *.cfg file into a ConfigFile object
   //terrainSceneManager->setWorldGeometry(terrainFileStr);

   Ogre::DataStreamPtr configStream = Ogre::ResourceGroupManager::getSingleton().openResource(terrainFileStr, Ogre::ResourceGroupManager::getSingleton().getWorldResourceGroupName());
   Ogre::ConfigFile config;
   config.load(configStream);

   /* Load the various settings we need.
   * These are:
   * width:  Saved in the terrainFile as PageSize.  We pass width to Bullet twice, as both width and height,
   *     because heightfields in Ogre are square.
   * imgFileStr: the string representing the image from which we want to load the heightmap data.
   * heightmap: an Ogre::Image object, which contains the data itself.  A reference to this needs to be kept
   *     somewhere, because just keeps a pointer to the data, and the data itself will be deleted when the
   *     Image is deleted at the end of this block.  I found this out the hard way >.>
   * maxHeight: the heightmap will be scaled to go from zero to maxHeight
   * heightScale: this is how much each number in the heightmap (0 - 255) will translate to in Ogre/Bullet units.
   *    This number is obtained by dividing the maxHeight by 256 which is the total number of different steps in
   *    the heightmap before interpolation.
   */
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
   btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
   btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
   btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
   btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
   mBtWorld->addRigidBody(groundRigidBody);

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
    Q_ASSERT(false);
    qDebug() << "Vehicle::slotUpdatePosition()";
    QMutexLocker locker(&mMutex);

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

    qDebug() << "Vehicle::slotUpdatePosition(): done";
}

void Vehicle::slotSetNextWayPoint(const CoordinateGps &wayPoint)
{
    QMutexLocker locker(&mMutex);

    mNextWayPoint = mCoordinateConverter.convert(wayPoint);
}

void Vehicle::slotSetFanSpeeds(const QList<int> &speeds)
{


    Q_ASSERT(speeds.size() <= mMotorPositions.size());

    for(int i=0;i<speeds.size();++i)
        mVehicleBody->applyForce(
                btVector3( // force
                        0,
                        5,
                        0),
                btVector3( // offset
                        mMotorPositions.at(i).x,
                        mMotorPositions.at(i).y,
                        mMotorPositions.at(i).z
                        )
                );

    //mVehicleBody->applyCentralForce(btVector3(0,5,0));
    mVehicleBody->applyForce(btVector3(0, 5, 0), btVector3(.2, 0, 0));
    mVehicleBody->applyForce(btVector3(0, 5, 0), btVector3(-.2, 0, 0));
    mVehicleBody->applyForce(btVector3(0, 5, 0), btVector3(0, 0, .2));
    mVehicleBody->applyForce(btVector3(0, 2, 0), btVector3(0, 0, -.2));

    //    mVehicleBody->applyTorque(btVector3(10,0,0));

    const Ogre::Vector3 vectorToTarget = mVehicleNode->_getDerivedPosition() - mNextWayPoint;
    const float yawToTarget = mVehicleNode->_getDerivedOrientation().getYaw();

}

void Vehicle::slotUpdatePhysics(void)
{
    const int simulationTime = mSimulator->getSimulationTime(); // milliseconds
    const btScalar deltaS = (simulationTime - mTimeOfLastUpdate) / 1000.0f; // elapsed time since last call in seconds
    const int maxSubSteps = 100;
    const btScalar fixedTimeStep = 1.0 / 60.0;
    qDebug() << "Vehicle::slotUpdatePhysics(): stepping physics, time is" << simulationTime << "delta" << deltaS;

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
