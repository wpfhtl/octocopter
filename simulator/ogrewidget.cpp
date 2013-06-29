#include "ogrewidget.h"

#include <QStatusBar>
#include <QtX11Extras/QX11Info>

const QPoint     OgreWidget::invalidMousePoint(-1,-1);
const Ogre::Real OgreWidget::turboModifier(200);

OgreWidget::OgreWidget(Simulator *simulator) :
    QWidget((QWidget*)simulator),
    mSimulator(simulator),
    mOgreRoot(0),
    mSceneManager(0),
    mOgreRenderWindow(0),
    mOgreViewport(0),
    mCamera(0),
    mVehicleNode(0),
    oldPosL(invalidMousePoint),
    oldPosR(invalidMousePoint),
    btnL(false),
    btnR(false),
    selectedNode(0),
    mFrameCount(0),
    mTerrainsImported(false),
    mMutex(QMutex::NonRecursive)
{
    qDebug() << "OgreWidget::OgreWidget()";

    setAttribute(Qt::WA_OpaquePaintEvent);

    // Do NOT set this, it will give you a black window!
    //setAttribute(Qt::WA_PaintOnScreen);

    setMinimumSize(240,240);
    setFocusPolicy(Qt::ClickFocus);
}

OgreWidget::~OgreWidget()
{
    QMutexLocker locker(&mMutex);

    // free the verticies and indicies memory
    QMapIterator<Ogre::Entity*, MeshInformation*> i(mEntities);
    while(i.hasNext())
    {
        i.next();
        MeshInformation* currentMeshInformation = i.value();
        //        delete[] currentMeshInformation->vertices;
        //        delete[] currentMeshInformation->indices;

        delete currentMeshInformation;
    }

    if(mOgreRenderWindow)
    {
        mOgreRenderWindow->removeAllViewports();
    }

    delete mTerrainGroup;
    delete mTerrainGlobals;

    if(mOgreRoot)
    {
        mOgreRoot->detachRenderTarget(mOgreRenderWindow);

        if(mSceneManager)
        {
            mOgreRoot->destroySceneManager(mSceneManager);
        }
    }

    mOgreRoot->shutdown();

    delete mOgreRoot;
}


void OgreWidget::initializeOgre()
{
    mOgreRoot = new Ogre::Root;

    Ogre::RenderSystem *renderSystem = mOgreRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
    mOgreRoot->setRenderSystem(renderSystem);
    mOgreRoot->initialise(false);

    Ogre::NameValuePairList viewConfig;
    QWidget *parentWidget = dynamic_cast <QWidget *>(parent());

    Ogre::String windowHandle = Ogre::StringConverter::toString ((unsigned long)QX11Info::display()) +
            ":" + Ogre::StringConverter::toString ((unsigned int)QX11Info::appScreen()) +
            ":" + Ogre::StringConverter::toString ((unsigned long)parentWidget->winId());


    qDebug() << __PRETTY_FUNCTION__ << "using window handle" << QString::fromStdString(windowHandle);
    viewConfig["parentWindowHandle"] = windowHandle;
    viewConfig["vsync"] = "true"; // this actually works on linux/nvidia-blob/thinkpad!

    mOgreRenderWindow = mOgreRoot->createRenderWindow("OgreRenderWindow", width(), height(), false, &viewConfig);

    mOgreRenderWindow->setActive(true);

    Ogre::SceneManagerEnumerator::MetaDataIterator iter = Ogre::SceneManagerEnumerator::getSingleton().getMetaDataIterator();
    while( iter.hasMoreElements() )
    {
        Ogre::String st = iter.getNext()->typeName;
        qDebug() << "Scene manager type available:" << QString::fromStdString(st);
    }

    mSceneManager = mOgreRoot->createSceneManager(Ogre::ST_GENERIC);
    //    ogreSceneManager = ogreRoot->createSceneManager("TerrainSceneManager"/*Ogre::ST_EXTERIOR_CLOSE*/);
    //    ogreSceneManager->showBoundingBoxes(true);

    // By default, entities cannot be found using a RSQ. This is to make sure that the
    // Lidar's don't catch the rotors, cameras etc. It also means we'll have to set the
    // flags to nonzero for any mesh that should be scannable.
    Ogre::MovableObject::setDefaultQueryFlags(0x00000000);

    mCamera = mSceneManager->createCamera("camera");
    mCamera->setNearClipDistance(.1);
    //mCamera->setPolygonMode(Ogre::PM_WIREFRAME);     /* wireframe */
    mCamera->setPolygonMode(Ogre::PM_SOLID);         /* solid */

    mCameraNode = mSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode", Ogre::Vector3(0, 12, 15));
    mCameraNode->attachObject(mCamera);
    mCamera->lookAt(0,8,0);

    mOgreViewport = mOgreRenderWindow->addViewport(mCamera);
    mOgreViewport->setBackgroundColour(Ogre::ColourValue(0.7, 0.7, 0.7));
    mCamera->setAspectRatio(Ogre::Real(width()) / Ogre::Real(height()));

    if(mOgreRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
    {
        mCamera->setFarClipDistance(0);   // enable infinite far clip distance if we can
    }

    // Initialize shader generator.
    // Must be before resource loading in order to allow parsing extended material attributes.
    if(!initializeRTShaderSystem(mSceneManager))
    {
        OGRE_EXCEPT(Ogre::Exception::ERR_FILE_NOT_FOUND,
                    "Shader Generator Initialization failed - Core shader libs path not found",
                    "SdkSample::_setup");
    }

    setupTerrain();

    // initialize trajectory line
    mTrajectoryLine = mSceneManager->createManualObject(QString("trajectoryLine_manualobject").toStdString());
    Ogre::SceneNode* mTrajectoryLineNode = mSceneManager->getRootSceneNode()->createChildSceneNode("trajectoryLine_node");

    Ogre::MaterialPtr trajectoryLineMaterial = Ogre::MaterialManager::getSingleton().create("trajectoryLineMaterial", "General");
    trajectoryLineMaterial->setReceiveShadows(false);
    trajectoryLineMaterial->getTechnique(0)->setLightingEnabled(true);
    trajectoryLineMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
    trajectoryLineMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
    trajectoryLineMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);
    //    trajectoryLineMaterial->dispose();  // dispose pointer, not the material
    mTrajectoryLineNode->attachObject(mTrajectoryLine);

    emit setupFinished();

    qDebug() << __PRETTY_FUNCTION__ << "done.";
}

void OgreWidget::keyPressEvent(QKeyEvent *e)
{
    QMutexLocker locker(&mMutex);

    // Only act if the pressed key is a movement key
    if(e->key() == Qt::Key_Space)
    {
        mCamera->lookAt(mSceneManager->getSceneNode("vehicleNode")->getPosition());
        update();
        e->accept();
    }
    else if(e->key() == Qt::Key_M)
    {
        if(mCamera->getPolygonMode() == Ogre::PM_SOLID)
            mCamera->setPolygonMode(Ogre::PM_WIREFRAME);
        else
            mCamera->setPolygonMode(Ogre::PM_SOLID);

        update();
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mouseMoveEvent(QMouseEvent *e)
{
    QMutexLocker locker(&mMutex);
    if(e->buttons().testFlag(Qt::LeftButton) && oldPosL != invalidMousePoint && btnL)
    {
        const QPoint &pos = e->pos();
        Ogre::Real deltaX = pos.x() - oldPosL.x();
        Ogre::Real deltaY = pos.y() - oldPosL.y();
        Ogre::Real deltaZ = pos.y() - oldPosL.y();

        // Switch the mouse-y axis between Ogre Y and Z when toggling shift
        if(e->modifiers().testFlag(Qt::ShiftModifier))
            deltaY = 0;
        else
            deltaZ = 0;

        if(e->modifiers().testFlag(Qt::ControlModifier))
        {
            deltaX *= turboModifier;
            deltaY *= turboModifier;
            deltaZ *= turboModifier;
        }

        mCameraNode->translate(mCamera->getOrientation() * Ogre::Vector3(deltaX/50, -deltaY/50, deltaZ/50), Ogre::Node::TS_LOCAL);
        oldPosL = pos;
        e->accept();
        update();
    }
    else if(e->buttons().testFlag(Qt::RightButton) && oldPosR != invalidMousePoint && btnR)
    {
        // rotate the camera
        const QPoint &pos = e->pos();
        const float diffX = (pos.x() - oldPosR.x()) / 10.0;
        const float diffY = (pos.y() - oldPosR.y()) / 10.0;
        //        qDebug() << "OgreWidget::mouseMoveEvent(): rotating camera:" << diffX << diffY;
        mCamera->yaw(Ogre::Degree(-diffX));
        mCamera->pitch(Ogre::Degree(-diffY));
        oldPosR = pos;
        e->accept();
        update();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mousePressEvent(QMouseEvent *e)
{
    qDebug() << "OgreWidget::mousePressEvent()";

    QMutexLocker locker(&mMutex);
    if(e->buttons().testFlag(Qt::LeftButton))
    {
        btnL = true;
        oldPosL = e->pos();
        e->accept();
    }
    else if(e->buttons().testFlag(Qt::RightButton))
    {
        btnR = true;
        oldPosR = e->pos();
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mouseReleaseEvent(QMouseEvent *e)
{
    qDebug() << "OgreWidget::mouseReleaseEvent()";

    QMutexLocker locker(&mMutex);
    if(!e->buttons().testFlag(Qt::LeftButton) && btnL)
    {
        oldPosL = QPoint(invalidMousePoint);
        btnL = false;
        e->accept();
    }
    else if(!e->buttons().testFlag(Qt::RightButton) && btnR)
    {
        oldPosR = QPoint(invalidMousePoint);
        btnR = false;
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::moveEvent(QMoveEvent *e)
{
    qDebug() << "OgreWidget::moveEvent()";

    QMutexLocker locker(&mMutex);

    QWidget::moveEvent(e);

    if(e->isAccepted() && mOgreRenderWindow)
    {
        qDebug() << "OgreWidget::moveEvent(): ogreRenderWindow->windowMovedOrResized()";
        mOgreRenderWindow->windowMovedOrResized();
        update();
    }
}

void OgreWidget::paintEvent(QPaintEvent* event)
{
//    qDebug() << __PRETTY_FUNCTION__;

    if(!mOgreRenderWindow)
    {
        initializeOgre();
    }

    QMutexLocker locker(&mMutex);
    if(!mOgreRoot) initializeOgre();

    if(mTerrainGroup->isDerivedDataUpdateInProgress())
    {
        if(mTerrainsImported)
        {
            qDebug() << "OgreWidget::paintEvent(): DerivedDataUpdateInProgress, building terrain, please wait";
            mSimulator->statusBar()->showMessage("Building terrain, please wait...");
        }
        else
        {
            qDebug() << "OgreWidget::paintEvent(): DerivedDataUpdateInProgress, updating textures, please wait";
            mSimulator->statusBar()->showMessage("Updating textures, patience...");
        }
    }
    else
    {
        if(mTerrainsImported)
        {
            qDebug() << "OgreWidget::paintEvent(): saving terrains    ";
            mSimulator->statusBar()->showMessage("Saving all terrains.");
            mTerrainGroup->saveAllTerrains(true);
            mTerrainsImported = false;
            //mSimulator->statusBar()->hide();
        }
    }

    mOgreRoot->_fireFrameStarted();

    // this should be IN a frame listener! animation currently unused
    //    if(mVehicleAnimationState) mVehicleAnimationState->addTime(0.1);

    //    qDebug() << "now rendering";

    // Construct all laserscanner rays before rendering
    QList<LaserScanner*> *laserScanners = mSimulator->mLaserScanners;
    for(int i = 0; i < laserScanners->size(); ++i)
    {
        LaserScanner* scanner = laserScanners->at(i);
        Ogre::Ray beam = scanner->getCurrentLaserBeam();

        Q_ASSERT(scanner->getSceneNode()->getParent() == mVehicleNode);

        //        scanner->getSceneNode()->_update(false, true);

        scanner->mRayObject->clear();

        if(scanner->isScanning())
        {
            scanner->mRayObject->begin(QString("RayFrom_" + scanner->objectName() + "_material").toStdString(), Ogre::RenderOperation::OT_LINE_LIST);
            //        scanner->mRayObject->position(mVehicleNode->_getDerivedPosition() + mVehicleNode->getOrientation() * scanner->getSceneNode()->getPosition());
            scanner->mRayObject->position(scanner->getSceneNode()->_getDerivedPosition());
            //        scanner->mRayObject->position(beam.getPoint(0.0));
            scanner->mRayObject->position(beam.getPoint(scanner->range()));
            scanner->mRayObject->end();
        }
    }

    mOgreRenderWindow->update();
    //    qDebug() << "rendering done";

    mOgreRoot->_fireFrameEnded();

    if(mFrameCount++ % 25 == 0)
        emit currentRenderStatistics(size(), mOgreRenderWindow->getTriangleCount(), mOgreRenderWindow->getLastFPS());

    event->accept();
}

void OgreWidget::resizeEvent(QResizeEvent* event)
{
    QMutexLocker locker(&mMutex);
    qDebug() << __PRETTY_FUNCTION__;

    QWidget::resizeEvent(event);
    if (event->isAccepted())
    {
        const QSize& newSize = event->size();
        if (mCamera)
        {
            Ogre::Real aspectRatio = Ogre::Real(newSize.width()) / Ogre::Real(newSize.height());
            mCamera->setAspectRatio(aspectRatio);
        }
        if (mOgreRenderWindow)
        {
            mOgreRenderWindow->resize(newSize.width(), newSize.height());
            mOgreRenderWindow->windowMovedOrResized();
            //mOgreRenderWindow->reposition(pos().x(), pos().y());
            //mOgreRenderWindow->resize(width, height);
        }
    }
}

void OgreWidget::wheelEvent(QWheelEvent *e)
{
    //    qDebug() << "OgreWidget::wheelEvent(): " << -e->delta() / 60;

    QMutexLocker locker(&mMutex);

    Ogre::Vector3 zTranslation(0,0, -e->delta() / 60);

    if(e->modifiers().testFlag(Qt::ControlModifier))
        mCameraNode->translate(zTranslation * turboModifier, Ogre::Node::TS_LOCAL);
    else
        mCameraNode->translate(zTranslation * Ogre::Vector3::NEGATIVE_UNIT_Z, Ogre::Node::TS_LOCAL);

    update();

    e->accept();
}

void OgreWidget::loadResources()
{
    qDebug() << "OgreWidget::loadResources()";
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load("resources.cfg");

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }

    // Initialise, parse scripts etc
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    qDebug() << "OgreWidget::loadResources(): done.";
}

Ogre::SceneNode* OgreWidget::createScanner(const QString name, const Ogre::Vector3 &relativePosition, const Ogre::Quaternion &relativeRotation)
{
    qDebug() << "OgreWidget::createScannerNode()";

    QMutexLocker locker(&mMutex);

    Ogre::Entity *scannerEntity = mSceneManager->createEntity(QString(name + "_entity").toStdString(), "hokuyoutm30lx.mesh");
    Ogre::SceneNode *scannerNode = mVehicleNode/*mSceneManager->getSceneNode("vehicleNode")*/->createChildSceneNode(QString(name + "_node").toStdString(), relativePosition, relativeRotation);
    scannerNode->attachObject(scannerEntity);
    qDebug() << "OgreWidget::createScannerNode(): done, returning";

    return scannerNode;
}

void OgreWidget::slotVisualizeTrajectory(const QVector3D& start, const QList<WayPoint>& waypoints)
{
    mTrajectoryLine->clear();
    mTrajectoryLine->begin(QString("trajectoryLineMaterial").toStdString(), Ogre::RenderOperation::OT_LINE_LIST);

    for(int i=0;i<waypoints.size();i++)
    {
        if(i==0)
            mTrajectoryLine->position(Ogre::Vector3(start.x(), start.y(), start.z()));
        else
            mTrajectoryLine->position(Ogre::Vector3(waypoints.at(i-1).x(), waypoints.at(i-1).y(), waypoints.at(i-1).z()));

        mTrajectoryLine->position(Ogre::Vector3(waypoints.at(i).x(), waypoints.at(i).y(), waypoints.at(i).z()));
    }

    mTrajectoryLine->end();
}

void OgreWidget::destroyScanner(const QString name)
{
    mSceneManager->destroyEntity(QString(name + "_entity").toStdString());
    mSceneManager->destroySceneNode(QString(name + "_node").toStdString());
}

void OgreWidget::createManualObject(const QString &name, Ogre::ManualObject** manualObject, Ogre::SceneNode** sceneNode, Ogre::MaterialPtr &material)
{
    qDebug() << "OgreWidget::createManualObject(), my address is" << this;

    QMutexLocker locker(&mMutex);

    *manualObject =  mSceneManager->createManualObject(QString(name+"_manualobject").toStdString());
    *sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(QString(name+"_scenenode").toStdString());
    material = Ogre::MaterialManager::getSingleton().createOrRetrieve(QString("LaserScannerMaterial").toStdString(), "General").first;
    qDebug() << "OgreWidget::createManualObject(): done, name of SceneNode is" << QString::fromStdString((*sceneNode)->getName());
}

void OgreWidget::destroyManualObject(Ogre::ManualObject* manualObject, Ogre::SceneNode* sceneNode)
{
    mSceneManager->getRootSceneNode()->removeAndDestroyChild(sceneNode->getName());

    // The ManualObject will automatically detach itself from any nodes on destruction.
    //    FIXME: this crashes.
    //    ogreSceneManager->destroyManualObject(manualObject);

    // Must not destroy material, it is shared between all scanners.
}

void OgreWidget::createRttCamera(Ogre::Camera** camera, Ogre::RenderTarget** renderTarget, Ogre::SceneNode ** sceneNode, const QString name, const QSize size)
{
    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().createManual(
                QString(name+"_texture").toStdString(),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME/*"general"*/,
                Ogre::TEX_TYPE_2D,
                size.width(),
                size.height(),
                32,
                0,
                Ogre::PF_R8G8B8,
                Ogre::TU_RENDERTARGET);

    *renderTarget = tex->getBuffer()->getRenderTarget();
    tex.setNull();

    *camera = mSceneManager->createCamera(QString(name+"_camera").toStdString());
    Ogre::Viewport* viewPort = (*renderTarget)->addViewport(*camera);
    viewPort->setBackgroundColour(Ogre::ColourValue::Red);

    // From: http://www.ogre3d.org/docs/api/html/classOgre_1_1Camera.html:
    // Note that a Camera can be attached to a SceneNode, using the method SceneNode::attachObject.
    // If this is done the Camera will combine it's own position/orientation settings with it's parent
    // SceneNode. This is useful for implementing more complex Camera / object relationships i.e.
    // having a camera attached to a world object.
    //
    // Cameras are attached to the camera-scenenode, which is attached to the vehicle-scenenode
    Ogre::Entity *cameraEntity = mSceneManager->createEntity(QString(name + "_entity").toStdString(), "camera.mesh");
    *sceneNode = mSceneManager->getSceneNode("vehicleNode")->createChildSceneNode(QString(name + "_node").toStdString());
    (*sceneNode)->attachObject(cameraEntity);
    (*sceneNode)->attachObject(*camera);
}

void OgreWidget::destroyRttCamera(const QString name/*, Ogre::RenderTarget* renderTarget, Ogre::Camera* camera*/)
{
    mSceneManager->getEntity(QString(name + "_entity").toStdString())->detachFromParent();
    mSceneManager->destroyEntity(QString(name + "_entity").toStdString());

    mSceneManager->getCamera(QString(name+"_camera").toStdString())->detachFromParent();
    mSceneManager->destroyCamera(QString(name+"_camera").toStdString());

    mSceneManager->destroySceneNode(QString(name + "_node").toStdString());

    // We remove the texture, and hopefully that means we'll also remove the renderTarget
    Ogre::TextureManager::getSingleton().remove(QString(name+"_texture").toStdString());
}

Ogre::SceneManager* OgreWidget::sceneManager()
{
    return mSceneManager;
}

void OgreWidget::setupTerrain()
{
    //    bool blankTerrain = false;


    mTerrainPos = Ogre::Vector3(1.0, -75.1, 1.0);

    //    mEditMarker = ogreSceneManager->createEntity("editMarker", "sphere.mesh");
    //    mEditNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode();
    //    mEditNode->attachObject(mEditMarker);
    //    mEditNode->setScale(0.05, 0.05, 0.05);

    // These two lines makes the terrain textures look nicer:
    Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
    Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(7);

    //mSceneManager->setFog(Ogre::FOG_EXP, Ogre::ColourValue(0.1, 0.1, 0.1), 0.01, 500, 1000);

    Ogre::LogManager::getSingleton().setLogDetail(Ogre::LL_BOREME);

    // Set up directional and ambient lights
    // THis first light is used for terrain shadows
    Ogre::Vector3 lightdir(-0.55, -0.3, 0.75);
    lightdir.normalise();
    Ogre::Light* light = mSceneManager->createLight("tstLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDirection(lightdir);
    light->setDiffuseColour(Ogre::ColourValue(1.0, 1.0, 1.0));
    light->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

    mSceneManager->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));

    Ogre::Vector3 lightdir2(-0.55, -0.3, -0.75);
    lightdir2.normalise();
    Ogre::Light* light2 = mSceneManager->createLight("tstLight2");
    light2->setType(Ogre::Light::LT_DIRECTIONAL);
    light2->setDirection(lightdir2);
    light2->setDiffuseColour(Ogre::ColourValue(0.4, 0.4, 0.4));
    light2->setSpecularColour(Ogre::ColourValue(0.2, 0.2, 0.2));

    Ogre::Vector3 lightdir3(0.55, -0.2, -0.75);
    lightdir3.normalise();
    Ogre::Light* light3 = mSceneManager->createLight("tstLight3");
    light3->setType(Ogre::Light::LT_DIRECTIONAL);
    light3->setDirection(lightdir3);
    light3->setDiffuseColour(Ogre::ColourValue(0.5, 0.5, 0.5));
    light3->setSpecularColour(Ogre::ColourValue(0.1, 0.1, 0.1));

    Ogre::Vector3 lightdir4(-0.55, -0.2, 0.75);
    lightdir4.normalise();
    Ogre::Light* light4 = mSceneManager->createLight("tstLight4");
    light4->setType(Ogre::Light::LT_DIRECTIONAL);
    light4->setDirection(lightdir4);
    light4->setDiffuseColour(Ogre::ColourValue(0.3, 0.3, 0.3));
    light4->setSpecularColour(Ogre::ColourValue(0.2, 0.2, 0.2));


    mTerrainGroup = new Ogre::TerrainGroup(mSceneManager, Ogre::Terrain::ALIGN_X_Z, TERRAIN_SIZE, TERRAIN_WORLD_SIZE);
    mTerrainGroup->setFilenameConvention(Ogre::String("cachedterrain"), Ogre::String("dat"));
    mTerrainGroup->setOrigin(mTerrainPos);

    mTerrainGlobals = new Ogre::TerrainGlobalOptions();
    configureTerrainDefaults(light);

    for(long x = TERRAIN_PAGE_MIN_X; x <= TERRAIN_PAGE_MAX_X; ++x)
        for (long y = TERRAIN_PAGE_MIN_Y; y <= TERRAIN_PAGE_MAX_Y; ++y)
            defineTerrain(x, y);

    // sync load since we NEED everything in place when we start
    mTerrainGroup->loadAllTerrains(true);

    // calculate the blend maps of all terrains
    if(mTerrainsImported)
    {
        Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
        while(ti.hasMoreElements())
        {
            Ogre::Terrain* t = ti.getNext()->instance;
            initBlendMaps(t);
        }
    }

    mTerrainGroup->freeTemporaryResources();

    // create a few entities on the terrain
    Ogre::Entity* entity;
    Ogre::SceneNode* sceneNode;
    Ogre::Quaternion rotation;
    const Ogre::Vector3 position(0, 0, -3);

    entity = mSceneManager->createEntity("tudorHouse", "tudorhouse.mesh");
    entity->setQueryFlags(0xFFFFFFFF);
    rotation.FromAngleAxis(Ogre::Degree(Ogre::Math::RangeRandom(-180, 180)-20), Ogre::Vector3::UNIT_Y);
    sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(
                position + Ogre::Vector3(
                    10,
                    mTerrainGroup->getHeightAtWorldPosition(position) + mTerrainPos.y + 6.4,
                    -10),
                rotation);
    sceneNode->setScale(Ogre::Vector3(0.012, 0.012, 0.012));
    sceneNode->attachObject(entity);
    addMeshInformation(entity,sceneNode);

    entity = mSceneManager->createEntity("church", "church.mesh");
    entity->setQueryFlags(0xFFFFFFFF);
    rotation.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);
    sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(
                position + Ogre::Vector3(
                    -7,
                    mTerrainGroup->getHeightAtWorldPosition(position) + mTerrainPos.y + 0.0,
                    22),
                rotation);
    sceneNode->attachObject(entity);
    addMeshInformation(entity,sceneNode);

    entity = mSceneManager->createEntity("house1", "house1.mesh");
    entity->setQueryFlags(0xFFFFFFFF);
    rotation.FromAngleAxis(Ogre::Degree(20), Ogre::Vector3::UNIT_Y);
    sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(
                position + Ogre::Vector3(
                    -23,
                    mTerrainGroup->getHeightAtWorldPosition(position) + mTerrainPos.y + 0.0,
                    -10),
                rotation);
    sceneNode->attachObject(entity);
    addMeshInformation(entity,sceneNode);

    entity = mSceneManager->createEntity("house2", "house2.mesh");
    entity->setMaterialName("house2/8_-_Default");
    entity->setQueryFlags(0xFFFFFFFF);
    rotation.FromAngleAxis(Ogre::Degree(-120), Ogre::Vector3::UNIT_Y);
    sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(
                position + Ogre::Vector3(
                    22,
                    mTerrainGroup->getHeightAtWorldPosition(position) + mTerrainPos.y + 3.0,
                    20),
                rotation);
    sceneNode->attachObject(entity);
    addMeshInformation(entity,sceneNode);

    entity = mSceneManager->createEntity("windmill", "windmill.mesh");
    entity->setQueryFlags(0xFFFFFFFF);
    rotation.FromAngleAxis(Ogre::Degree(120), Ogre::Vector3::UNIT_Y);
    sceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode(
                position + Ogre::Vector3(
                    -35,
                    mTerrainGroup->getHeightAtWorldPosition(position) + mTerrainPos.y - 0.2,
                    20),
                rotation);
    sceneNode->attachObject(entity);
    addMeshInformation(entity,sceneNode);

    //    qDebug() << "number of entities:" << mEntities.size();

    mSceneManager->setSkyBox(true, "Examples/CloudyNoonSkyBox");
}

void OgreWidget::addMeshInformation(Ogre::Entity* entity, Ogre::SceneNode* node)
{
    MeshInformation* mi = new MeshInformation;

    mi->sceneNode = node;

    getMeshInformation(
                entity->getMesh(),
                mi->vertex_count,
                mi->vertices,
                mi->index_count,
                mi->indices,
                node->_getDerivedPosition(),
                node->_getDerivedOrientation(),
                node->_getDerivedScale()
                );

    mi->aabb = entity->getBoundingBox();
    mi->aabb.transform(node->_getFullTransform());

    mi->valid = true;

    mEntities.insert(entity, mi);
}

void OgreWidget::configureTerrainDefaults(Ogre::Light* light)
{
    // Configure global
    // MaxPixelError decides how precise our terrain is going to be. A lower number will mean
    // a more accurate terrain, at the cost of performance (because of more vertices).
    mTerrainGlobals->setMaxPixelError(8);

    // CompositeMapDistance decides how far the Ogre terrain will render the lightmapped terrain.
    mTerrainGlobals->setCompositeMapDistance(3000);

    //    mTerrainGlobals->setUseRayBoxDistanceCalculation(true);
    //mTerrainGlobals->getDefaultMaterialGenerator()->setDebugLevel(1);
    //mTerrainGlobals->setLightMapSize(256);

    //matProfile->setLightmapEnabled(false);
    // Important to set these so that the terrain knows what to use for derived (non-realtime) data
    mTerrainGlobals->setLightMapDirection(light->getDerivedDirection());
    //    mTerrainGlobals->setCompositeMapAmbient(mSceneManager->getAmbientLight());
    mTerrainGlobals->setCompositeMapDiffuse(light->getDiffuseColour());

    // Configure default import settings for if we use imported image
    Ogre::Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
    defaultimp.terrainSize = TERRAIN_SIZE;
    defaultimp.worldSize = TERRAIN_WORLD_SIZE;
    defaultimp.inputBias = 75; // tune for world-size!
    defaultimp.inputScale = 60; // tune for world-size!
    defaultimp.minBatchSize = 33;
    defaultimp.maxBatchSize = 65;

    // Set up the textures
    defaultimp.layerList.resize(3);
    defaultimp.layerList[0].worldSize = 100;
    defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
    defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
    defaultimp.layerList[1].worldSize = 30;
    defaultimp.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
    defaultimp.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
    defaultimp.layerList[2].worldSize = 100;
    defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
    defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");
}

void OgreWidget::defineTerrain(long x, long y, bool flat)
{
    // First, we ask our TerrainGroup what file name it would use to generate the terrain. Then we check if
    // there is a file by that name in our resource group. If there is, it means that we generated a binary
    // terrain data file already, and thus there is no need to import it from an image. If there isn't a data
    // file present, it means we have to generate our terrain, and we load the image and uses that to define it.

    if(flat)
    {
        qDebug() << "OgreWidget::defineTerrain(): flat";
        mTerrainGroup->defineTerrain(x, y, 0.0f);
    }
    else
    {
        Ogre::String filename = mTerrainGroup->generateFilename(x, y);
        qDebug() << "OgreWidget::defineTerrain(): filename" << QString::fromStdString((std::string)filename) << "resourceGroup" << QString::fromStdString((std::string)mTerrainGroup->getResourceGroup());
        if(Ogre::ResourceGroupManager::getSingleton().resourceExists(mTerrainGroup->getResourceGroup(), filename))
        {
            qDebug() << "OgreWidget::defineTerrain(): resource exists, calling TerrainGroup::defineTerrain with x y" << x << y;
            mTerrainGroup->defineTerrain(x, y);
        }
        else
        {
            qDebug() << "OgreWidget::defineTerrain(): resource absent, calling TerrainGroup::defineTerrain with image for x y" << x << y;
            Ogre::Image img;
            getTerrainImage(x % 2 != 0, y % 2 != 0, img);
            img.save("test2.png");
            mTerrainGroup->defineTerrain(x, y, &img);
            mTerrainGroup->getTerrainDefinition(x, y)->importData->inputImage->save("test3.png");

            mTerrainsImported = true;
        }
    }
}

void OgreWidget::getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
    img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    qDebug() << "OgreWidget::getTerrainImage(): loaded terrain heightmap, width is" << img.getWidth();
    if(flipX)
        img.flipAroundY();
    if(flipY)
        img.flipAroundX();

}

void OgreWidget::initBlendMaps(Ogre::Terrain* terrain)
{
    // Let's just say that it uses the terrain height to splat the three layers
    // on the terrain. Notice the use of getLayerBlendMap and getBlendPointer.
    Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
    Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);
    Ogre::Real minHeight0 = 70;
    Ogre::Real fadeDist0 = 40;
    Ogre::Real minHeight1 = 70;
    Ogre::Real fadeDist1 = 15;
    float* pBlend1 = blendMap1->getBlendPointer();
    for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y)
    {
        for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x)
        {
            Ogre::Real tx, ty;

            blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
            Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
            Ogre::Real val = (height - minHeight0) / fadeDist0;
            val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
            //*pBlend0++ = val;

            val = (height - minHeight1) / fadeDist1;
            val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
            *pBlend1++ = val;
        }
    }
    blendMap0->dirty();
    blendMap1->dirty();
    blendMap0->update();
    blendMap1->update();
}

bool OgreWidget::initializeRTShaderSystem(Ogre::SceneManager* sceneMgr)
{
    if(Ogre::RTShader::ShaderGenerator::initialize())
    {
        loadResources();
        mShaderGenerator = Ogre::RTShader::ShaderGenerator::getSingletonPtr();

        mShaderGenerator->addSceneManager(sceneMgr);

        // Setup core libraries and shader cache path.
        qDebug() << "number of resource groups:" << Ogre::ResourceGroupManager::getSingleton().getResourceGroups().size();
        Ogre::StringVector groupVector = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
        Ogre::StringVector::iterator itGroup = groupVector.begin();
        Ogre::StringVector::iterator itGroupEnd = groupVector.end();
        Ogre::String shaderCoreLibsPath;
        Ogre::String shaderCachePath;

        for (; itGroup != itGroupEnd; ++itGroup)
        {
            qDebug() << "now looking in resource group" << QString::fromStdString(*itGroup);
            Ogre::ResourceGroupManager::LocationList resLocationsList = Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(*itGroup);
            Ogre::ResourceGroupManager::LocationList::iterator it = resLocationsList.begin();
            Ogre::ResourceGroupManager::LocationList::iterator itEnd = resLocationsList.end();
            bool coreLibsFound = false;

            // Try to find the location of the core shader lib functions and use it
            // as shader cache path as well - this will reduce the number of generated files
            // when running from different directories.
            for (; it != itEnd; ++it)
            {
                std::string archiveName = (*it)->archive->getName();
                qDebug() << "Now looking for RTShaderLib in" << QString::fromStdString(archiveName);
                if((*it)->archive->getName().find("RTShaderLib") != Ogre::String::npos)
                {
                    shaderCoreLibsPath = (*it)->archive->getName() + "/";
                    shaderCachePath = shaderCoreLibsPath;
                    coreLibsFound = true;
                    break;
                }
            }
            // Core libs path found in the current group.
            if(coreLibsFound)
                break;
        }

        // Core shader libs not found -> shader generating will fail.
        if(shaderCoreLibsPath.empty())
        {
            qDebug() << "shaderCoreLibsPath.empty()!";
            return false;
        }

#ifdef _RTSS_WRITE_SHADERS_TO_DISK
        // Set shader cache path.
        mShaderGenerator->setShaderCachePath(shaderCachePath);
#endif
        // Create and register the material manager listener.
        mMaterialMgrListener = new ShaderGeneratorTechniqueResolverListener(mShaderGenerator);
        Ogre::MaterialManager::getSingleton().addListener(mMaterialMgrListener);
    }
    else
    {
        qDebug() << "Ogre::RTShader::ShaderGenerator::initialize() failed.";
    }

    return true;
}

Ogre::SceneNode* OgreWidget::createVehicleNode(const Ogre::String name, Ogre::Entity** entity, const Ogre::Vector3 position, const Ogre::Quaternion orientation)
{
    *entity = mSceneManager->createEntity("vehicleEntity", "oktokopter.mesh");
    mVehicleNode = mSceneManager->getRootSceneNode()->createChildSceneNode(name, position, orientation);
    mSceneManager->getRootSceneNode()->removeChild(mCameraNode);
    mVehicleNode->addChild(mCameraNode);
    //    mCameraNode->translate(Ogre::Vector3(0, 0, 5));
    mCameraNode->setPosition(/*mVehicleNode->_getDerivedPosition() +*/ Ogre::Vector3(0, 5, 15));
    //    mCameraNode->setInheritOrientation(false);
    mCamera->lookAt(mVehicleNode->_getDerivedPosition());
    qDebug() << "OgreWidget::createVehicleNode(): vehicle is at" << mVehicleNode->_getDerivedPosition().x << mVehicleNode->_getDerivedPosition().y << mVehicleNode->_getDerivedPosition().z;
    qDebug() << "OgreWidget::createVehicleNode(): camera  is at" << mCameraNode->_getDerivedPosition().x << mCameraNode->_getDerivedPosition().y << mCameraNode->_getDerivedPosition().z;
    return mVehicleNode;
}


// Get the mesh information for the given mesh.
// Code found on this forum link: http://www.ogre3d.org/wiki/index.php/RetrieveVertexData
void OgreWidget::getMeshInformation(const Ogre::MeshPtr mesh,
                                    size_t &vertex_count,
                                    Ogre::Vector3* &vertices,
                                    size_t &index_count,
                                    Ogre::uint32* &indices,
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
            if(!added_shared)
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
    indices = new Ogre::uint32[index_count];

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

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Ogre::Real* pOgre::Real;
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

        Ogre::uint32*  pLong = static_cast<Ogre::uint32*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        if( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<Ogre::uint32>(offset);
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<Ogre::uint32>(pShort[k]) + static_cast<Ogre::uint32>(offset);
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
}
