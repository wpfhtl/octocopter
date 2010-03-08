#include "laserscanner.h"

LaserScanner::LaserScanner(
        Simulator* simulator,
        OgreWidget* ogreWidget,
        const float range,
        const int speed,
        const int angleStart,
        const int angleStop,
        const float angleStep) :
        mRaySceneQuery(0), mRayObject(0), mRayNode(0)
{
    qDebug() << "LaserScanner::LaserScanner()";
    Q_ASSERT(angleStart < angleStop);

    mSimulator = simulator;
    mOgreWidget = ogreWidget;
    mRange = range;
    mSpeed = (float)speed;
    mAngleStart = angleStart;
    mAngleStop = angleStop;
    mAngleStep = angleStep;
    mTimeFactor = mSimulator->getTimeFactor();

    // We'll need a unique name, so why not use our own address...
    char address[50];
    sprintf(address, "%p", (void*)this);
    setObjectName("LaserScanner_at_" + QString(address));

    // Create a scene node in ogre that is attached to the vehicle.
    mScannerNode = mOgreWidget->createScannerNode(objectName());

    mCurrentScanAngle = mAngleStart;

    // Get the RaySceneQuery from Ogre
    mRaySceneQuery = mOgreWidget->createRaySceneQuery();
    mRaySceneQuery->setSortByDistance(true, 1); // just return the first 1 intersecting element.
    // Not sure on this, see http://www.ogre3d.org/wiki/index.php/Intermediate_Tutorial_3#Query_Masks
    mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK | Ogre::SceneManager::ENTITY_TYPE_MASK | Ogre::SceneManager::STATICGEOMETRY_TYPE_MASK);

    // Initialize the beams visualization. The name needs to be unique, so I use a dirty hack.
    qDebug() << "LaserScanner::LaserScanner(): creating manualObjet for ray, my address is" << this;
    mOgreWidget->createManualObject("Ray_From_" + objectName(), &mRayObject, &mRayNode, mRayMaterial);
    mRayObject->setDynamic(true);
    mRayMaterial->setReceiveShadows(false);
    mRayMaterial->getTechnique(0)->setLightingEnabled(true);
    mRayMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
    mRayMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
    mRayMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);
    mRayNode->attachObject(mRayObject);

    // When we emit a scan, make mSimulator receive it
    connect(this, SIGNAL(scanFinished(QList<CoordinateGps>)), mSimulator, SLOT(slotScanFinished(QList<CoordinateGps>)));

    mTimerScanStep = new QTimer(this);
    mTimerScanStep->setSingleShot(true);
    connect(mTimerScanStep, SIGNAL(timeout()), SLOT(slotDoScanStep()));

    // make the scanner appear
    mOgreWidget->update();

    if(! mSimulator->isPaused())
    {
        qDebug() << "LaserScanner::LaserScanner(): almost done, starting first slotDoScanStep()";
        slotDoScanStep();
    }
}

void LaserScanner::slotDoScanStep(void)
{
//    qDebug() << "LaserScanner::slotDoScanStep()";
     // Set up scene query.
//    Feb 10 [23:03:51] <kernelpanic_> Excuse me, even after reading the Quaternion primer, I can't figure out how to create a new
//                                     Ray(mySceneNode.getPosition(), mySceneNode.getOrientation()). ::getOrientation() returns a
//                                     Quaternion (I understand that), but Ray wants a Vector3 as direction. Why is that and how do
//                                     I convert from Quat to Vector3?
//    Feb 10 [23:06:37] <Tenttu> multiple NEGATIVE_UNIT_Z with the quaternion
//    Feb 10 [23:06:48] <Landon> kernelpanic_: how about this: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=46750&p=321002&hilit=quaternion
//    Feb 10 [23:07:22] <kernelpanic_> Landon: yeah, that sounds like my problem. Thank you, I'm reading
//    Feb 10 [23:16:52] <kernelpanic_> Tenttu: why exactly the Z-axis?
//    Feb 10 [23:17:44] <Tenttu> it's forward vector
//    Feb 10 [23:18:07] <Tenttu> and when you rotate it with the quaternion you get the direction
//    Feb 10 [23:18:50] <Tenttu> but camera does have getDirection() though
//    Feb 10 [23:19:14] <kernelpanic_> I'm afraid I probably haven't really understood quaternions yet :| I guess I'll read the links referenced in that forum thread.

    // Build a quaternion that represents the laserbeam's current rotation
    Ogre::Quaternion quatBeamRotation(Ogre::Degree(mCurrentScanAngle), Ogre::Vector3::UNIT_Y);
    Ogre::Ray laserBeam(mScannerNode->_getDerivedPosition(), mScannerNode->_getDerivedOrientation() * quatBeamRotation * Ogre::Vector3::NEGATIVE_UNIT_Z);
    mRaySceneQuery->setRay(laserBeam);

    // Visualize the ray in Ogre? Fuck yeah!
    mRayObject->clear();
    mRayObject->begin(QString("RayFrom_" + objectName() + "_material").toStdString(), Ogre::RenderOperation::OT_LINE_LIST);
    mRayObject->position(laserBeam.getPoint(0.0));
    mRayObject->position(laserBeam.getPoint(mRange));
    mRayObject->end();
    mOgreWidget->update();

    // Perform the scene query
    Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute();

    // TODO: This only gets the bounding-box. Extend to the polygons by using
    // http://www.ogre3d.org/wiki/index.php/Raycasting_to_the_polygon_level

    // Don't save relative coordinates, the scanner's position will change
    // until the very next iteration. Thus, save world (=gps) coordinates.
    if(result.size())
        mScanData << mCoordinateConverter.convert(laserBeam.getPoint((result.front().distance)));

    // Increase mCurrentScanAngle by mAngleStep for the next laserBeam
    mCurrentScanAngle += mAngleStep;

    // Theoretically, there is no reason to emit complete scans instead of "streaming"
    // single world coordinates. I suspect that fewer callbacks, fewer and bigger
    // network-packets will be more performant, thought. It might actually be even
    // smarter to emit every n CoordinateGps, when n marshalled CoordinateGps reach
    // the interface's MTU. But thats for later, we're still simulating right now...

    float degreesToNextScan;

    if(mCurrentScanAngle > mAngleStop)
    {
        // This scan is finished, emit it...
        emit scanFinished(mScanData);

        // ...and schedule the next scan. The time to next scan is inferred from
        // rotational speed and angle between mCurrentScanAngle and angleStart.
        degreesToNextScan = 360.0 - mAngleStop + mAngleStart;
        qDebug() << "LaserScanner::slotDoScanStep(): scan done, degrees to next scan:" << degreesToNextScan << "timeFactor is" << mTimeFactor << "speed is" << mSpeed;

        // Set mCurrentScanAngle for the next scan to mAngleStart
        mCurrentScanAngle = mAngleStart;
    }
    else
    {
        // Scan is not finished, schedule the next ray
        degreesToNextScan = mAngleStep;
    }

    // 6000 == 360.0[degress/round] / 60.0[seconds/minute] * 1000.0[milliseconds/second]
    mTimerScanStep->setInterval((int)(degreesToNextScan / mSpeed * 6000.0 * mTimeFactor));
    mTimerScanStep->start();
//    QTimer::singleShot(
//            (int)(degreesToNextScan / mSpeed * 6000.0 * mTimeFactor),
//            this,
//            SLOT(slotDoScanStep())
//            );

//    qDebug() << "LaserScanner::slotDoScanStep(): done.";
}

void LaserScanner::slotPause(void)
{
    mTimerScanStep->stop();
}

void LaserScanner::slotStart(void)
{
    mTimerScanStep->start();
}

Ogre::SceneNode* LaserScanner::getSceneNode(void)
{
    return mScannerNode;
}

void LaserScanner::setTimeFactor(float timeFactor)
{
    mTimeFactor = timeFactor;
}

// Getters for the properties
float LaserScanner::range(void) const
{
    return mRange;
}

int LaserScanner::speed(void) const
{
    return (int)mSpeed;
}

int LaserScanner::angleStart(void) const
{
    return mAngleStart;
}

int LaserScanner::angleStop(void) const
{
    return mAngleStop;
}

float LaserScanner::angleStep(void) const
{
    return mAngleStep;
}

// Setters for the properties
void LaserScanner::setRange(float range)
{
    mRange = range;
}

void LaserScanner::setSpeed(int speed)
{
    mSpeed = speed;
}

void LaserScanner::setAngleStart(int angleStart)
{
    mAngleStart = angleStart;
}

void LaserScanner::setAngleStop(int angleStop)
{
    mAngleStop = angleStop;
}

void LaserScanner::setAngleStep(float angleStep)
{
    mAngleStep = angleStep;
}

void LaserScanner::setPosition(const Ogre::Vector3 &position)
{
    mScannerNode->setPosition(position);
}

void LaserScanner::setOrientation(const Ogre::Quaternion &orientation)
{
    mScannerNode->setOrientation(orientation);
}

Ogre::Vector3 LaserScanner::getPosition(void)
{
    return mScannerNode->getPosition();
}

Ogre::Quaternion LaserScanner::getOrientation(void)
{
    return mScannerNode->getOrientation();
}

