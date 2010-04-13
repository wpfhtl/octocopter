#include "laserscanner.h"

LaserScanner::LaserScanner(
        Simulator* simulator,
        OgreWidget* ogreWidget,
        const float range,
        const int speed,
        const int angleStart,
        const int angleStop,
        const float angleStep) :
        mRaySceneQuery(0)
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

    // We need to register what we want to emit.
    qRegisterMetaType<QList<CoordinateGps> >("QList<CoordinateGps>");

    // We'll need a unique name, so why not use our own address...
    char address[50];
    sprintf(address, "%p", (void*)this);
    setObjectName("LaserScanner_at_" + QString(address));

    // Material name for the ray-visualization
    mMaterialName = QString("RayFrom_" + objectName() + "_material").toStdString();

    // Create a scene node in ogre that is attached to the vehicle.
    mScannerNode = mOgreWidget->createScannerNode(objectName());

    mLaserBeam.setOrigin(mScannerNode->_getDerivedPosition());
//    mLaserBeam.setDirection(mScannerNode->getOrientation());

    mCurrentScanAngle = mAngleStart;

    // Get the RaySceneQuery from Ogre
    mRaySceneQuery = mOgreWidget->createRaySceneQuery();
//    mRaySceneQuery->setSortByDistance(true, 1); // just return the first 1 intersecting element.
    mRaySceneQuery->setSortByDistance(true);
//    mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);
    // Not sure on this, see http://www.ogre3d.org/wiki/index.php/Intermediate_Tutorial_3#Query_Masks
//    mRaySceneQuery->setQueryMask(0);
//    mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK | Ogre::SceneManager::ENTITY_TYPE_MASK | Ogre::SceneManager::STATICGEOMETRY_TYPE_MASK);
    mRaySceneQuery->setQueryTypeMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);


    for(std::set<Ogre::SceneQuery::WorldFragmentType>::const_iterator it=mRaySceneQuery->getSupportedWorldFragmentTypes()->begin();it!=mRaySceneQuery->getSupportedWorldFragmentTypes()->end();it++)
    {
        qDebug() << "LaserScanner::LaserScanner(): supported queries:" << *it;
    }


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
    // TODO: delete/unregister scannerNode, mRayMaterial etc.
//    mRayObject->clear();
//    delete mRayObject;
//    mTimerScanStep->deleteLater();
    //mOgreWidget->sceneManager()->destoryQuery();
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
        qDebug() << "LaserScanner::slotDoScan(): next ray:" << mCurrentScanAngle;
        slotDoScanStep();

        gettimeofday(&timeNow, NULL);

        const long long scanTimeElapsed = (timeNow.tv_sec - timeStart.tv_sec) * 1000000 + (timeNow.tv_usec - timeStart.tv_usec);
        const long long scanTimeAtNextRay = realTimeBetweenRaysUS * (numberOfRays+1);

        usleep(std::max(0, (int)(scanTimeAtNextRay - scanTimeElapsed)));
    }

    gettimeofday(&timeNow, NULL);

    const long long timeDiff = (timeNow.tv_sec - timeStart.tv_sec) * 1000000 + (timeNow.tv_usec - timeStart.tv_usec);
    qDebug() << "LaserScanner::slotDoScan(): took" << timeDiff << "us, should have been" << (long long)(realTimeBetweenRaysUS * ((mAngleStop - mAngleStart)/mAngleStep));

    // This scan is finished, emit it...
//    emit scanFinished(mScanData);
    QByteArray datagram;
    QDataStream stream(&datagram, QIODevice::WriteOnly);
    stream << "test";
    stream << &mScanData;
    qDebug() << "LaserScanner::slotDoScan(): List size" << mScanData.size() << "UDP packet size:" << datagram.size();
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
    qDebug() << "LaserScanner::slotDoScan(): emitted results, resting" << timeRest << "us after scan.";
    usleep(std::max(0, (int)timeRest));
}

void LaserScanner::slotDoScanStep()
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
    QMutexLocker locker(&mMutex);
    mLaserBeam.setOrigin(mScannerPosition - Ogre::Vector3(0.0, 0.1, 0.0));
    mLaserBeam.setDirection(mScannerOrientation * quatBeamRotation * Ogre::Vector3::NEGATIVE_UNIT_Z);
    locker.unlock();

    Ogre::Vector3 result;
    if(raycastFromPoint(mScannerPosition, mLaserBeam.getDirection(), result))
    {
        const Ogre::Real distance = mScannerPosition.distance(result);
        qDebug() << "LaserScanner::slotDoScanStep(): I hit a mesh, distance" << distance;
    }

//    mRaySceneQuery->setRay(mLaserBeam);

//    mRaySceneQuery->setQueryTypeMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK );
//    mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);

    // Perform the scene query
//    mRaySceneQuery->execute(this);

//    Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute();
//    Ogre::RaySceneQueryResult::iterator itr = result.begin();

//    if(itr != result.end())
//    {
//        if(itr->worldFragment)
//        qDebug() << "ben:" << itr->distance;

//        mCurrentObject->setPosition(itr->worldFragment->singleIntersection);
//    }


    // Increase mCurrentScanAngle by mAngleStep for the next laserBeam
    mCurrentScanAngle += mAngleStep;
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
}

void LaserScanner::slotSetScannerPose(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
    // This slot is called whenever the vehicle's position changes. Thus, we
    // should be called at least 25, at most mSpeed/60 times per second.
    QMutexLocker locker(&mMutex);
    mScannerPosition = position;
    mScannerOrientation = orientation;
}

bool LaserScanner::queryResult(Ogre::SceneQuery::WorldFragment* /*fragment*/, Ogre::Real distance)
{
    // A callback for when the ray hits worldgeometry

    // TODO: This only gets the bounding-box. Extend to the polygons by using
    // http://www.ogre3d.org/wiki/index.php/Raycasting_to_the_polygon_level

    // Don't save relative coordinates, the scanner's position will change
    // until the very next iteration. Thus, save world (=gps) coordinates.
Q_ASSERT(false);
    qDebug() << "LaserScanner::queryResult(): hit wft at distance of" << distance;

    if(distance > 0.1 && distance <= mRange)
        mScanData << mCoordinateConverter.convert(mLaserBeam.getPoint(distance));
    else if(distance < 0.0001)
        printf(".");

    return true;
}

bool LaserScanner::queryResult(Ogre::MovableObject* obj, Ogre::Real distance)
{
    // A callback for when the ray hits movable objects. Behave as for worldgeometry
    std::string bop = obj->getName();
    QString foo(bop.c_str());
    qDebug() << "LaserScanner::queryResult(): hit mov" << foo << "at distance of" << distance;

    if(distance > 0.1 && distance <= mRange)
        mScanData << mCoordinateConverter.convert(mLaserBeam.getPoint(distance));
    else if(distance < 0.0001)
        printf(":");

    return true;
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

void LaserScanner::setTimeFactor(float timeFactor)
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

Ogre::Ray LaserScanner::getCurrentLaserBeam(void)
{
    QMutexLocker locker(&mMutex);
    return mLaserBeam;
}









// Raycasts from a point in to the scene. Returns success or failure.
// On success the point is returned in the result.
bool LaserScanner::raycastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 &result)
{
    // create the ray to test
    Ogre::Ray ray(Ogre::Vector3(point.x, point.y, point.z),
                  Ogre::Vector3(normal.x, normal.y, normal.z));

//    // check we are initialised
//    if (mRaySceneQuery != NULL)
//    {
//        // create a query object
        mRaySceneQuery->setRay(ray);

        // execute the query, returns a vector of hits
        if (mRaySceneQuery->execute().size() <= 0)
        {
            // raycast did not hit an objects bounding box
            qDebug() << "LaserScanner::raycastFromPoint(): RSQ didn't hit anything, not inspecting any meshes.";
            return (false);
        }
//    }
//    else
//    {
//        LOG_ERROR << "Cannot raycast without RaySceneQuery instance" << ENDLOG;
//        return (false);
//    }

    // at this point we have raycast to a series of different objects bounding boxes.
    // we need to test these different objects to see which is the first polygon hit.
    // there are some minor optimizations (distance based) that mean we wont have to
    // check all of the objects most of the time, but the worst case scenario is that
    // we need to test every triangle of every object.
    Ogre::Real closest_distance = -1.0f;
    Ogre::Vector3 closest_result;
    Ogre::RaySceneQueryResult &query_result = mRaySceneQuery->getLastResults();
    for (size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++)
    {
        qDebug() << "LaserScanner::raycastFromPoint(): Now checking RSQ-result number" << qr_idx;
        // stop checking if we have found a raycast hit that is closer than all remaining entities
        if ((closest_distance >= 0.0f) && (closest_distance < query_result[qr_idx].distance))
        {
            qDebug() << "LaserScanner::raycastFromPoint(): we have already found the closest hit at" << closest_distance;
            break;
        }

        if(query_result[qr_idx].worldFragment)
        {
            Ogre::SceneQuery::WorldFragment* wft = query_result[qr_idx].worldFragment;
            qDebug() << "LaserScanner::raycastFromPoint(): result" << qr_idx << "is a world fragment!";

        }

        Ogre::Entity* ent = static_cast<Ogre::Entity*>(query_result[qr_idx].movable);
        const Ogre::MeshPtr ptr1 = ent->getMesh();
        Ogre::String foo = ent->getMovableType();
        qDebug() << "LaserScanner::raycastFromPoint(): Now checking movable object" << ent->getMovableType().c_str()<< "with name" << ent->getName().c_str() << "parentNode" << query_result[qr_idx].movable->getParentNode()->getName().c_str() << "parentNode" << query_result[qr_idx].movable->getParentNode()->getParent()->getName().c_str() << "parentNode" << query_result[qr_idx].movable->getParentNode()->getParent()->getParent()->getName().c_str() << "parentNode" << query_result[qr_idx].movable->getParentNode()->getParent()->getParent()->getParent()->getName().c_str();


        Ogre::SceneNode* scn = mOgreWidget->sceneManager()->getSceneNode(ent->getName());
        for(int i=0;i<scn->numAttachedObjects();i++)
        {
            Ogre::MovableObject* obj1 = scn->getAttachedObject(i);
            qDebug() << "number"<<i<<"of"<<scn->numAttachedObjects()<<"attached to" << ent->getName().c_str() << "is" << obj1->getName().c_str();

            Ogre::Entity* ent2 = static_cast<Ogre::Entity*>(obj1);
            qDebug() << "number of subentities" << ent2->getNumSubEntities();
            const Ogre::MeshPtr ptr = ent2->getMesh();
            Q_ASSERT(!ptr.isNull());

        }
        // only check this result if its a hit against an entity
        if((query_result[qr_idx].movable != NULL) && (query_result[qr_idx].movable->getMovableType().compare("TerrainMipMap") == 17/*0*/))
        {
            // get the entity to check
            Ogre::Entity *pentity = ent;//static_cast<Ogre::Entity*>(query_result[qr_idx].movable);

            // mesh data to retrieve
            size_t vertex_count;
            size_t index_count;
            Ogre::Vector3 *vertices;
            unsigned long *indices;

            // get the mesh information
//         getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
//                              pentity->getParentNode()->getWorldPosition(),
//                              pentity->getParentNode()->getWorldOrientation(),
//                              pentity->getParentNode()->_getDerivedScale());

            qDebug() << "LaserScanner::raycastFromPoint(): downloading mesh information";
            getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                              pentity->getParentNode()->_getDerivedPosition(),
                              pentity->getParentNode()->_getDerivedOrientation(),
                              pentity->getParentNode()->_getDerivedScale());

            // test for hitting individual triangles on the mesh
            bool new_closest_found = false;
            for (int i = 0; i < static_cast<int>(index_count); i += 3)
            {
                // check for a hit against this triangle
                std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
                    vertices[indices[i+1]], vertices[indices[i+2]], true, false);

                // if it was a hit check if its the closest
                if (hit.first)
                {
                    if ((closest_distance < 0.0f) || (hit.second < closest_distance))
                    {
                        // this is the closest so far, save it off
                        qDebug() << "LaserScanner::raycastFromPoint(): found new closest hit at" << hit.second;
                        closest_distance = hit.second;
                        new_closest_found = true;
                    }
                }
            }

         // free the verticies and indicies memory
            delete[] vertices;
            delete[] indices;

            // if we found a new closest raycast for this object, update the
            // closest_result before moving on to the next object.
            if (new_closest_found)
            {
                closest_result = ray.getPoint(closest_distance);
            }
        }
        else
        {
            qDebug() << "LaserScanner::raycastFromPoint(): current RSQ result is either not movable or not an entity.";
        }
    }

    // return the result
    if(closest_distance >= 0.0f)
    {
        // raycast success
        result = closest_result;
        return true;
    }
    else
    {
        // raycast failed
        return false;
    }
}




// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
void LaserScanner::getMeshInformation(
        const Ogre::MeshPtr mesh,
        size_t &vertex_count,
        Ogre::Vector3* &vertices,
        size_t &index_count,
        unsigned long* &indices,
        const Ogre::Vector3 &position,
        const Ogre::Quaternion &orient,
        const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;

    vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }


    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }


        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        // Ogre 1.6 patch (commenting the static_cast...) - index offsets start from 0 for each submesh
        if ( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] /*+ static_cast<unsigned long>(offset)*/;
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) /*+
                    static_cast<unsigned long>(offset)*/;
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
}
