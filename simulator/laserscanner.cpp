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
    qDebug() << "LaserScanner::LaserScanner()";
    Q_ASSERT(angleStart < angleStop);

    mSimulator = simulator;
    mOgreWidget = ogreWidget;
    setRange(range); // to also set mRangeSquared
    mSpeed = (float)speed;
    mAngleStart = angleStart;
    mAngleStop = angleStop;
    mAngleStep = angleStep;
    mTimeFactor = mSimulator->getTimeFactor();

    // We need to register what we want to emit.
    qRegisterMetaType<QList<CoordinateGps> >("QList<CoordinateGps>");

    // We'll need a unique name, so why not use our own address...
    char address[50];
    sprintf(address, "%p", (void*)this);
    setObjectName("LaserScanner_at_" + QString(address));

    // Material name for the ray-visualization
    mMaterialName = QString("RayFrom_" + objectName() + "_material").toStdString();

    // Create a scene node in ogre that is attached to the vehicle.
    mScannerNode = mOgreWidget->createScanner(objectName());

    mLaserBeam.setOrigin(mScannerNode->_getDerivedPosition());
//    mLaserBeam.setDirection(mScannerNode->getOrientation());

    mCurrentScanAngle = mAngleStart;

//    qDebug() << "LaserScanner::LaserScanner(): creating manualObjet for ray, my address is" << this;
    mOgreWidget->createManualObject("Ray_From_" + objectName(), &mRayObject, &mRayNode, mRayMaterial);
    mRayObject->setDynamic(true);
    mRayObject->setQueryFlags(0);
    mRayMaterial->setReceiveShadows(false);
    mRayMaterial->getTechnique(0)->setLightingEnabled(true);
    mRayMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
    mRayMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
    mRayMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);
    mRayNode->attachObject(mRayObject);

    mNumberOfRaySceneQueries = 0;

    // make the scanner appear
    mOgreWidget->update();
}

void LaserScanner::testRsqPerformance()
{
    struct timeval timeStart;
    gettimeofday(&timeStart, NULL);

    const int iterations = 1000000;

//    for(int i=0;i<iterations;i++)
//    {
//        slotDoScanStep(false);
//    }

    struct timeval timeStop;
    gettimeofday(&timeStop, NULL);

    const long long timeDiff = (timeStop.tv_sec - timeStart.tv_sec) * 1000000 + (timeStop.tv_usec - timeStart.tv_usec);

    qDebug() << "LaserScanner::testRsqPerformance():" << iterations << "RSQs took" << timeDiff << "ms, giving" << (int)(iterations * (1.0 / ((float)timeDiff / 1000000.0)) / 1000.0) << "kRSQ/s";

}

LaserScanner::~LaserScanner()
{
    qDebug() << "LaserScanner::~LaserScanner()";
    mOgreWidget->destroyScanner(objectName());
    mOgreWidget->destroyManualObject(mRayObject, mRayNode);
    mUdpSocket->deleteLater();
    mTimerScan->deleteLater();
}

void LaserScanner::run()
{
    mUdpSocket = new QUdpSocket;
    mUdpSocket->bind();

    mTimerScan = new QTimer;
    mTimerScan->setInterval(0);
    connect(mTimerScan, SIGNAL(timeout()), SLOT(slotDoScan()));

    if(!mSimulator->isPaused()) mTimerScan->start();

    qDebug() << "LaserScanner::run(): starting LaserScanner eventloop in threadid" << currentThreadId();
    exec();
    qDebug() << "LaserScanner::run(): eventloop finished in threadid" << currentThreadId();
}

void LaserScanner::slotDoScan()
{
    // Theoretically, there is no reason to emit complete scans instead of "streaming"
    // single world coordinates. I suspect that fewer callbacks, fewer and bigger
    // network-packets will be more performant, though. It might actually be even
    // smarter to emit every n CoordinateGps, when n marshalled CoordinateGps reach
    // the interface's MTU. But thats for later, we're still simulating right now...

    if(mScannerOrientationPrevious == mScannerOrientation && mScannerPositionPrevious == mScannerPosition)
    {
        // No need to scan, we'll get the same results as previously. IF THE REST OF
        // THE WORLD IS STATIC, that is. Sleep for one scan, then try again.
        qDebug() << "LaserScanner::slotDoScan(): vehicle hasn't moved, sleeping scan";
        usleep(1000000 / (mSpeed / 60) * (1.0 / mTimeFactor));
        return;
    }
    else
        qDebug() << "LaserScanner::slotDoScan(): vehicle has moved, scanning";

    const long long realTimeBetweenRaysUS = (1000000.0 / 6.0 / mSpeed * mAngleStep) * (1.0 / mTimeFactor);

    struct timeval timeStart;
    gettimeofday(&timeStart, NULL);

    struct timeval timeNow;

    int numberOfRays = 0;

    while(mCurrentScanAngle <= mAngleStop)
    {
        numberOfRays++;

        mNumberOfRaySceneQueries++;

        //qDebug() << "LaserScanner::slotDoScan(): next ray:" << mCurrentScanAngle;
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
        QMutexLocker locker(&mMutex);
        mLaserBeam.setOrigin(mScannerPosition/* - Ogre::Vector3(0.0, 0.1, 0.0)*/);
        mLaserBeam.setDirection(mScannerOrientation * quatBeamRotation * Ogre::Vector3::NEGATIVE_UNIT_Z);
        locker.unlock();

        float closestDistanceToEntity = -1.0f;

        // We once used a rayscenequery from the scenemanager for querying meshes, but that was slow and not thread-safe.
        // Instead, each laserscanner now keeps a copy of all meshes' information. Meshes are read once in the c'tor from
        // OgreWidget::mEntities and then a pointer is kept. When scanning, a corresponding MeshInformation-Pointer is
        // filled with information for that mesh once and used subsequently. No more RaySceneQuery needed. HAH!
        QMapIterator<Ogre::Entity*, OgreWidget::MeshInformation*> i(mOgreWidget->mEntities);
        while(i.hasNext())
        {
            i.next();
            Ogre::Entity* currentEntity = i.key();
            OgreWidget::MeshInformation* currentMeshInformation = i.value();

            std::pair<bool,Ogre::Real> result = mLaserBeam.intersects(currentMeshInformation->aabb);
            if(result.first)
            {
                // stop checking if we have found a raycast hit that is closer than all remaining entities
                if(closestDistanceToEntity >= 0.0f && closestDistanceToEntity < result.second) break;

                // also stop checking if the distance to this object's bbox is out of this lidar's range
                if(mRange < result.second) break;


                // test for hitting individual triangles on the mesh
                for(size_t i = 0; i < currentMeshInformation->index_count; i += 3)
                {
                    // check for a hit against this triangle
                    const std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(
                                mLaserBeam,
                                currentMeshInformation->vertices[currentMeshInformation->indices[i]],
                                currentMeshInformation->vertices[currentMeshInformation->indices[i+1]],
                                currentMeshInformation->vertices[currentMeshInformation->indices[i+2]],
                                true, // positiveSide
                                false // negativeSide
                                );

                    // if it was a hit, check if its the closest
                    if(hit.first)
                    {
                        if((closestDistanceToEntity < 0.0f) || (hit.second < closestDistanceToEntity))
                        {
                            // this is the closest so far, save it off
                            closestDistanceToEntity = hit.second;
                        }
                    }
                }
                qDebug() << objectName() << QString::fromStdString(currentEntity->getName()) << "at" << closestDistanceToEntity;
            }
        }

        // Do a RSQ against the terrain.
        // http://www.ogre3d.org/docs/api/html/classOgre_1_1TerrainGroup.html says about rayIntersects:
        // This can be called from any thread as long as no parallel write to the terrain data occurs.
        Ogre::TerrainGroup::RayResult rayResultTerrain = mOgreWidget->mTerrainGroup->rayIntersects(mLaserBeam);

        float distanceValidEntity  = 99999.9;
        float distanceValidTerrain = 99999.9;

        if(closestDistanceToEntity > 0)
            distanceValidEntity = closestDistanceToEntity;

        if(rayResultTerrain.hit)
            distanceValidTerrain = mLaserBeam.getOrigin().distance(rayResultTerrain.position);

        const float distanceFinal = std::min(distanceValidEntity, distanceValidTerrain);

        qDebug() << objectName() << distanceValidEntity << distanceValidTerrain << distanceFinal;

        if(distanceFinal < mRange)
        {
            const Ogre::Vector3 point = mLaserBeam.getPoint(distanceFinal);
            mScanData << QVector3D(point.x, point.y, point.z);
        }

        // Increase mCurrentScanAngle by mAngleStep for the next laserBeam
        mCurrentScanAngle += mAngleStep;

        gettimeofday(&timeNow, NULL);

        const long long scanTimeElapsed = (timeNow.tv_sec - timeStart.tv_sec) * 1000000 + (timeNow.tv_usec - timeStart.tv_usec);
        const long long scanTimeAtNextRay = realTimeBetweenRaysUS * (numberOfRays+1);

//        if(mNumberOfRaySceneQueries%10000 == 0) qDebug() << "Number of RSQs:" << mNumberOfRaySceneQueries;

        usleep(std::max(0, (int)(scanTimeAtNextRay - scanTimeElapsed)));
    }

    gettimeofday(&timeNow, NULL);

//    const long long timeDiff = (timeNow.tv_sec - timeStart.tv_sec) * 1000000 + (timeNow.tv_usec - timeStart.tv_usec);
//    qDebug() << "LaserScanner::slotDoScan(): took" << timeDiff << "us, should have been" << (long long)(realTimeBetweenRaysUS * ((mAngleStop - mAngleStart)/mAngleStep));

    // This scan is finished, emit it...
//    emit scanFinished(mScanData);
    QByteArray datagram;
    QDataStream stream(&datagram, QIODevice::WriteOnly);

    // How many hits?
    stream << (qint32)mScanData.size();
    // From which position?
    // Assuming the laserscanner's position stays pretty much the same within one scan, there's no need
    // to transmit the direction to the scanner in every hit-packet. So we only send the positions.
    stream << QVector3D(mScannerPosition.x-180, mScannerPosition.y, mScannerPosition.z-120);
    // Stream the hits
    stream << mScanData;

//    qDebug() << "LaserScanner::slotDoScan(): List size" << mScanData.size() << "UDP data size:" << datagram.size() << "first 20 bytes:" << datagram.left(20);
//    <aep> kernelpanic: and read the hint in the flush() docs.  if it returns false, you need to wait and flush again
//    <aep> krunk-: ah waitForBytesWritten() works
//    <aep> kernelpanic: ^
//    <kernelpanic> aep: ok. But I send 40 udp packets per second anyway, could I just write() the next packet and flush() again instead of waiting?
//    <aep> kernelpanic: you could, but you dont get any gurantees when the packet is delivered
//    <aep> kernelpanic: and if you're unlucky, you're getting out of sync
//    <kernelpanic> aep: what does that mean? I'm just filling and flushing a buffer, no?
//    <aep> kernelpanic: flush() does only write as much as the OS accepts.  usually thats plenty, but it may under rare conditions get stuck
//    <aep> kernelpanic: yes. but you only call flush on write, so there is more data comming
//    <aep> with an eventloop you get free "unsticking" since the eventloop will interupt YOUR work and make sure stuff is sent when the queue is free
//    <aep> kernelpanic: very likely it will work just fine, just remember the caveats ;)
//    <kernelpanic> aep: i don't understand. I'm doing write()/flush() with 40Hz, so the data will get written to the OS eventually. And I prefer high throughput to low latency. Or am I completely missing you?
//    <aep> yes you are, that was not the point
//    <aep> flush() only writes a specific amount of data, not ALL of it
//    <aep> kernelpanic: for(;;){send() flush();}  only works as long as send() is smaller then the OS write buffer. otherwise you build up a backlog
    mUdpSocket->writeDatagram(datagram, QHostAddress::Broadcast, 45454);
    mUdpSocket->flush();
//    mUdpSocket->waitForBytesWritten();
    mScanData.clear();

    // Set mCurrentScanAngle for the next scan to mAngleStart
    mCurrentScanAngle = mAngleStart;
    mScannerOrientationPrevious = mScannerOrientation;
    mScannerPositionPrevious = mScannerPosition;

    // Sleep for degreesToNextScan = 360.0 - mAngleStop + mAngleStart
    gettimeofday(&timeNow, NULL);
    const long long scanTimeElapsed = (timeNow.tv_sec - timeStart.tv_sec) * 1000000 + (timeNow.tv_usec - timeStart.tv_usec);
    const long long timeRest = (1000000/(mSpeed/60)) * (1.0 / mTimeFactor) - scanTimeElapsed;
//    qDebug() << "LaserScanner::slotDoScan(): emitted results, resting" << timeRest << "us after scan.";
    usleep(std::max(0, (int)timeRest));


}


void LaserScanner::slotSetScannerPose(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
    // This slot is called whenever the vehicle's position changes. Thus, we
    // should be called at least 25, at most mSpeed/60 times per second.
    QMutexLocker locker(&mMutex);
    mScannerPosition = position;
    mScannerOrientation = orientation;
}

void LaserScanner::slotPause(void)
{
    qDebug() << "LaserScanner::slotPause(): stopping scanner";
    mTimerScan->stop();
//    mContinue = false;
}

void LaserScanner::slotStart(void)
{
    qDebug() << "LaserScanner::slotStart(): starting scanner timer in thread" << currentThreadId();
    mTimerScan->start();
}

Ogre::SceneNode* LaserScanner::getSceneNode(void)
{
    return mScannerNode;
}

void LaserScanner::slotSetTimeFactor(double timeFactor)
{
    QMutexLocker locker(&mMutex);
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
    mRangeSquared = pow(mRange, 2.0);
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
    QMutexLocker locker(&mMutex);
    mScannerNode->setPosition(position);

    // Configuration widget calls this method, so update the beam too.
    mLaserBeam.setOrigin(mScannerNode->_getDerivedPosition());
    locker.unlock();
}

void LaserScanner::setOrientation(const Ogre::Quaternion &orientation)
{
    QMutexLocker locker(&mMutex);
    mScannerNode->setOrientation(orientation);

    // Configuration widget calls this method, so update the beam too.
    Ogre::Quaternion quatBeamRotation(Ogre::Degree(mCurrentScanAngle), Ogre::Vector3::UNIT_Y);
    mLaserBeam.setDirection(mScannerNode->_getDerivedOrientation() * quatBeamRotation * Ogre::Vector3::NEGATIVE_UNIT_Z);
    locker.unlock();
}

Ogre::Vector3 LaserScanner::getPosition(void)
{
    return mScannerNode->getPosition();
}

Ogre::Quaternion LaserScanner::getOrientation(void)
{
    return mScannerNode->getOrientation();
}

Ogre::Ray LaserScanner::getCurrentLaserBeam(void)
{
    QMutexLocker locker(&mMutex);
    return mLaserBeam;
//    return Ogre::Ray(mLaserBeam.getOrigin(), mLaserBeam.getDirection());
}
