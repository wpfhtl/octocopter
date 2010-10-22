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
    mTimerScan->deleteLater();
}

void LaserScanner::run()
{
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
//        qDebug() << "LaserScanner::slotDoScan(): vehicle hasn't moved, sleeping scan";
        usleep(1000000 / (mSpeed / 60) * (1.0 / mTimeFactor));
        return;
    }

    const long long realTimeBetweenRaysUS = (1000000.0 / 6.0 / mSpeed * mAngleStep) * (1.0 / mTimeFactor);

    struct timeval timeStart;
    gettimeofday(&timeStart, NULL);

    struct timeval timeNow;

    int numberOfRays = 0;

    // a container for collected rays, or rather the world coordinates of where they ended
    QList<QVector3D> scanData;

    while(mCurrentScanAngle <= mAngleStop)
    {
        numberOfRays++;

        mNumberOfRaySceneQueries++;

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

        if(distanceFinal < mRange)
        {
            const Ogre::Vector3 point = mLaserBeam.getPoint(distanceFinal);
            scanData << QVector3D(point.x-180, point.y, point.z-120);
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

    // This scan is finished, send it to our baseconnection
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);

    // set packet type.
    stream << QString("points");

    // How many hits?
    stream << (quint32)scanData.size();

    // From which position?
    // Assuming the laserscanner's position stays pretty much the same within one scan, there's no need
    // to transmit the direction to the scanner in every hit-packet. So we only send the positions.
    stream << QVector3D(mScannerPosition.x, mScannerPosition.y, mScannerPosition.z);

    // Stream the hits
    stream << scanData;

//    qDebug() << "lidar sending bytes:" << data.length();

    mSimulator->mBaseConnection->slotSendData(data);

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
}
