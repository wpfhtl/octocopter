#include "laserscanner.h"

LaserScanner::LaserScanner(
        Simulator* simulator,
        OgreWidget* ogreWidget,
        const float range,
        const int speed,
        const int angleStart,
        const int angleStop,
        const float angleStep)
{
    Q_ASSERT(angleStart < angleStop);

    mSimulator = simulator;
    mOgreWidget = ogreWidget;
    mRange = range;
    mSpeed = (float)speed;
    mAngleStart = angleStart;
    mAngleStop = angleStop;
    mAngleStep = angleStep;
    mTimeFactor = mSimulator->getTimeFactor();

    // Create a scene node in ogre that is attached to the vehicle. We give
    // out this node to others when they want to move/rotate us.
    mScannerNode = mOgreWidget->createScannerNode();

    mCurrentScanAngle = mAngleStart;

    mRaySceneQuery = mOgreWidget->createRaySceneQuery();
    mRaySceneQuery->setSortByDistance(true, 1); // just return the first 1 intersecting element.
    // Not sure on this, see http://www.ogre3d.org/wiki/index.php/Intermediate_Tutorial_3#Query_Masks
    mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK | Ogre::SceneManager::ENTITY_TYPE_MASK | Ogre::SceneManager::STATICGEOMETRY_TYPE_MASK);

    // When we emit a scan, make mSimulator receive it
    connect(this, SIGNAL(scanFinished(LaserScanner*, QList<long>)), mSimulator, SLOT(slotScanFinished(LaserScanner*, QList<long>)));
}

void LaserScanner::slotDoScanStep(void)
{
     // Set up scene query.
//    Feb 10 [23:03:51] <kernelpanic_> Excuse me, even after reading the Quaternion primer, I can't figure out how to create a new Ray(mySceneNode.getPosition(), mySceneNode.getOrientation()). ::getOrientation() returns a Quaternion (I understand that), but Ray wants a Vector3 as direction. Why is that and how do I convert from Quat to Vector3?
//    Feb 10 [23:04:55] <-- Brendan123 (~asdf@c-24-131-233-38.hsd1.pa.comcast.net) has quit
//    Feb 10 [23:06:37] <Tenttu> multiple NEGATIVE_UNIT_Z with the quaternion
//    Feb 10 [23:06:48] <Landon> kernelpanic_: how about this: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=46750&p=321002&hilit=quaternion
//    Feb 10 [23:07:22] <kernelpanic_> Landon: yeah, that sounds like my problem. Thank you, I'm reading
//    Feb 10 [23:16:52] <kernelpanic_> Tenttu: why exactly the Z-axis?
//    Feb 10 [23:17:44] <Tenttu> it's forward vector
//    Feb 10 [23:18:07] <Tenttu> and when you rotate it with the quaternion you get the direction
//    Feb 10 [23:18:50] <Tenttu> but camera does have getDirection() though
//    Feb 10 [23:19:14] <kernelpanic_> I'm afraid I probably haven't really understood quaternions yet :| I guess I'll read the links referenced in that forum thread.
    Ogre::Ray laserBeam(mScannerNode->getPosition(), mScannerNode->_getDerivedOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z);
    mRaySceneQuery->setRay(laserBeam);

    // TODO: visualize the ray in Ogre? Fuck yeah!

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

    if(mCurrentScanAngle > mAngleStop)
    {
        // This scan is finished, emit it...
        emit scanFinished(mScanData);

        // ...and schedule the next scan. The time to next scan is inferred from
        // rotational speed and angle between mCurrentScanAngle and angleStart.
        const float degreesToNextScan = 360.0 - mCurrentScanAngle + mAngleStart;

        // 6000 == 360.0[degress/round] / 60.0[seconds/minute] * 1000.0[milliseconds/second]
        const int milliSecsToNextScan = (int)(degreesToNextScan / mSpeed * 6000.0 * mTimeFactor);

        // Set mCurrentScanAngle for the next scan to mAngleStart
        mCurrentScanAngle = mAngleStart;

        QTimer::singleShot(milliSecsToNextScan, this, SLOT(slotDoScanStep()));
    }
    else
    {
        // Scan is not finished, schedule the next ray
        // 6000 == 360.0[degress/round] / 60.0[seconds/minute] * 1000.0[milliseconds/second]
        const int milliSecsToNextScan = (int)(mAngleStep / mSpeed * 6000.0 * mTimeFactor);
        QTimer::singleShot(milliSecsToNextScan, this, SLOT(slotDoScanStep()));
    }
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
