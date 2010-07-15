#include "vehicle.h"

Vehicle::Vehicle(Simulator *simulator, OgreWidget *ogreWidget) :
//        QThread(simulator),
        QObject(simulator),
        mMutex(QMutex::NonRecursive)
{
    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    // create joystick
    mJoystick = new Joystick();
    qDebug() << "Vehicle::Vehicle(): initialized joystick, isvalid():" << mJoystick->isValid();

    mTimerUpdatePosition = new QTimer(this);
    mTimerUpdatePosition->setInterval(1000/25);
    connect(mTimerUpdatePosition, SIGNAL(timeout()), SLOT(slotUpdatePosition()));

    // Set up motors, as seen from the top
    // http://www.mikrokopter.de/ucwiki/ElektronikVerkabelung#head-4e6a59d9824e114e97488c7fdaa2b1d75025bae9
//    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, -0.20))));  // engine 1, forward, CW
//    mEngines.append(Engine(btTransform(btQuaternion(000,deg2rad(000),000), btVector3(+0.00, +0.00, +0.20))));  // engine 2, backward,  CW
//    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(+0.20, +0.00, +0.00))));  // engine 3, right, CCW
//    mEngines.append(Engine(btTransform(btQuaternion(000,000,deg2rad(180)), btVector3(-0.20, +0.00, +0.00))));  // engine 4, left,  CCW


    // Bullet initialisation.
    mBtBroadphase = new btAxisSweep3(btVector3(-20000,-20000,-20000), btVector3(20000,20000,20000), 1024);
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
    mVehicleNode = mOgreWidget->createVehicleNode("vehicleNode", Ogre::Vector3(0,0,0), Ogre::Quaternion::IDENTITY);
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

    // Place the vehicle 10m above the ground
    Ogre::TerrainGroup::RayResult rayResult = mOgreWidget->mTerrainGroup->rayIntersects(Ogre::Ray(Ogre::Vector3(0,1000,0), Ogre::Vector3::NEGATIVE_UNIT_Y));
    if(rayResult.hit)
    {
        mVehicleNode->setPosition(rayResult.position.x, rayResult.position.y + 10.0, rayResult.position.z);
        qDebug() << "Vehicle::Vehicle(): creating vehicle, setting to height" << rayResult.position.y + 10.0;
    }

    // Make camera look at vehicle
//    mOgreWidget->setCameraPosition(
//            mVehicleNode->_getDerivedPosition() + Ogre::Vector3(0.0,2.0,-10.0),
//            OgreWidget::TRANSLATION_ABSOLUTE,
//            Ogre::Node::TS_WORLD,
//            mVehicleNode);

    // Create collision shape
    BtOgre::StaticMeshToShapeConverter converter(mVehicleEntity);
    mVehicleShape = converter.createConvex();

    // Reduce vertices (20k to maybe 100?)
    btShapeHull* hull = new btShapeHull(mVehicleShape);
    btScalar margin = mVehicleShape->getMargin();
    hull->buildHull(margin);
    mVehicleShape = new btConvexHullShape((btScalar*)hull->getVertexPointer(), hull->numVertices()/*, sizeof(btVector3)*/);

    //Calculate inertia.
    btScalar mass = 0.5;
    btVector3 inertia;
    mVehicleShape->calculateLocalInertia(mass, inertia);

    //Create BtOgre MotionState (connects Ogre and Bullet).
    mVehicleState = new BtOgre::RigidBodyState(mVehicleNode);
    connect(mVehicleState, SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)), SIGNAL(newPose(const Ogre::Vector3, const Ogre::Quaternion)));

    //Create the Body.
    mVehicleBody = new btRigidBody(mass, mVehicleState, mVehicleShape, inertia);
//    mVehicleBody->setCollisionFlags(mVehicleBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

    // linear, angular
    mVehicleBody->setDamping(.5, .7);

    mVehicleBody->setActivationState(DISABLE_DEACTIVATION); // probably unnecessary
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
//   localScaling.setX(atof(config.getSetting("PageWorldX").c_str()) / (width -1));
//   localScaling.setZ(atof(config.getSetting("PageWorldZ").c_str()) / (width -1));

   // And now, we actually call Bullet. heightmap needs to be on the heap, as bullet does not copy it.
   mGroundShape = new btHeightfieldTerrainShape(width, width, heightmap->getData(), heightScale, 0, maxHeight, 1, PHY_UCHAR, false);
//   mGroundShape = new btHeightfieldTerrainShape(12000, 12000, heightmap->getData(), heightScale, 0, maxHeight, 1, PHY_UCHAR, false);
   mGroundShape->setLocalScaling(localScaling);

   // All thats left is to line up the Ogre::SceneNode with the btHeightfieldTerrainShape.
   // We have to do it this way because of differences in how Bullet and Ogre orient the shapes.
   // In Ogre, the terrain's top left corner is at (0, 0, 0) in the local TransformSpace
   // In Bullet, not only is the center of the btHeighfield terrrain shape at (0, 0, 0), but from
   // what I can tell, its immovable.
   btVector3 min, max;
   mGroundShape->getAabb(btTransform::getIdentity(), min, max);
//   Ogre::SceneNode *sNode;// = mOgreWidget->sceneManager()->getSceneNode("Terrain");
//   sNode->setPosition();

   // Finally, create your btMotionState, and btRigidBody, and all the rigid body to the physics world.
   BtOgre::RigidBodyState* terrainState = new BtOgre::RigidBodyState(mOgreWidget->mTerrainGroup);
   mGroundBody = new btRigidBody(0.0, terrainState, mGroundShape);
   mBtWorld->addRigidBody(mGroundBody);
   terrainState->setWorldTransform(btTransform(btQuaternion::getIdentity(), min + BtOgre::Convert::toBullet(Ogre::Vector3(0, -190, 0))));
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

    float joyX, joyY, joyZ, joyR;
    mJoystick->getValues(joyX, joyY, joyZ, joyR);

    // Set motor-base-speed according to thrust between 0 and 40000
    int f = (-joyR + 0.5) * 20000;
    int b = f;
    int l = f;
    int r = f;
//    qDebug() << "joyVals\t\t" << joyX << joyY << joyZ << joyR;
//    qDebug() << "baseSpeed\t" << f << b << l << r;

    // Steering sensitivity is higher in slow motion
    const int maxSteeringPitchRoll = 500 / mSimulator->getTimeFactor();
    const int maxSteeringYaw = 150 / mSimulator->getTimeFactor();

    // When stick goes right, joyX goes up => l+ r-
    l += (maxSteeringPitchRoll * joyX);
    r -= (maxSteeringPitchRoll * joyX);

    // When stick goes forward, joyY goes up => b- f+
    b -= (maxSteeringPitchRoll * joyY);
    f += (maxSteeringPitchRoll * joyY);

    // When stick is yawed...
    f += (maxSteeringYaw * -joyZ);
    b += (maxSteeringYaw * -joyZ);
    l -= (maxSteeringYaw * -joyZ);
    r -= (maxSteeringYaw * -joyZ);

//    qDebug() << "endSpeed\t" << "f" << f << "b" << b << "l" << l << "r" << r;

    QList<int> motorSpeeds;
    // motorSpeeds << forward << backward << left << right
    motorSpeeds << f << b << -l << -r;

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
    Q_ASSERT(speeds.size() <= mEngineNodes.size());

    // We apply forces to the vehicle depending on the speeds of its propellers.
/*
    for(int i=0;i<speeds.size();++i)
    {
        const btVector3 thrustScalar = mEngines.at(i).calculateThrust(speeds.at(i));
        const Ogre::Vector3 thrustVectorOgre = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(thrustScalar.x(), thrustScalar.y(), thrustScalar.z());
        const btVector3 thrustVectorBt(thrustVectorOgre.x, thrustVectorOgre.y, thrustVectorOgre.z);
        const btVector3 position = mEngines.at(i).getPosition();
        mVehicleBody->applyForce(thrustVectorBt, position);
        //qDebug() << "Vehicle::slotSetMotorSpeeds(): thrust" << i << thrustVectorOgre.x << thrustVectorOgre.y << thrustVectorOgre.z << "at" << position.x() << position.y() << position.z();

        const btVector3 torque = mEngines.at(i).calculateTorque(speeds.at(i));
        mVehicleBody->applyTorque(torque);
        //qDebug() << "Vehicle::slotSetMotorSpeeds(): torque" << i << torque.x() << torque.y() << torque.z();
    }
*/

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
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): thrust" << i << thrustVectorOgre.x << thrustVectorOgre.y << thrustVectorOgre.z << "at" << position.x() << position.y() << position.z();

        const double torqueScalar = mEngine.calculateTorque(speeds.at(i));
        Ogre::Vector3 torque = mVehicleNode->_getDerivedOrientation() * Ogre::Vector3(0.0, torqueScalar, 0.0);
        mVehicleBody->applyTorque(btVector3(torque.x, torque.y, torque.z));
//        qDebug() << "Vehicle::slotSetMotorSpeeds(): torque y:" << i << torque;
    }

//    qDebug() << "Vehicle::slotSetMotorSpeeds(): total yaw torque is" << mVehicleBody->getTotalTorque().y();
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

//    mBtDebugDrawer->step();

    mTimeOfLastUpdate = simulationTime;

    // Set all laserscanners new position
    // The lidars are attached to the vehicle's scenenode, so stepSimulation should update the vehicle's sceneNode and
    // thus also update the laserscanner's position. BUT the lidars do up t 60k RSQ/s, so pushing these changes
    // should be more efficient than having the LaserScanners poll on every RaySceneQuery.
    QList<LaserScanner*>* laserScanners = mSimulator->getLaserScannerList();
    foreach(LaserScanner* ls, *laserScanners)
    {
        Ogre::SceneNode* scannerNode = mOgreWidget->sceneManager()->getSceneNode(ls->objectName().append("_node").toStdString());
        ls->slotSetScannerPose(scannerNode->_getDerivedPosition(), scannerNode->_getDerivedOrientation());
    }

    // TODO: If we loop with more than 25fps, update more slowly
    // TODO: why the fuck does a timer in vehicle drive the gl update?
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

