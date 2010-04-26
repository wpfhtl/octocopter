#include "ogrewidget.h"

const QPoint     OgreWidget::invalidMousePoint(-1,-1);
const Ogre::Real OgreWidget::turboModifier(10);

OgreWidget::OgreWidget(Simulator *simulator) :
        QWidget((QWidget*)simulator),
        mSimulator(simulator),
        ogreRoot(0),
        ogreSceneManager(0),
        ogreRenderWindow(0),
        ogreViewport(0),
        mCamera(0),
        oldPosL(invalidMousePoint),
        oldPosR(invalidMousePoint),
        btnL(false),
        btnR(false),
        selectedNode(0),
        mFrameCount(0),
        mUpdateTimerId(0),
        mMutex(QMutex::NonRecursive)
{
    qDebug() << "OgreWidget::OgreWidget()";
    QMutexLocker locker(&mMutex);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_PaintOnScreen);
    setMinimumSize(240,240);
    setFocusPolicy(Qt::ClickFocus);

    mKeyCoordinateMapping[Qt::Key_W] = Ogre::Vector3( 0, 0,-1);
    mKeyCoordinateMapping[Qt::Key_S] = Ogre::Vector3( 0, 0, 1);
    mKeyCoordinateMapping[Qt::Key_A] = Ogre::Vector3(-1, 0, 0);
    mKeyCoordinateMapping[Qt::Key_D] = Ogre::Vector3( 1, 0, 0);
    mKeyCoordinateMapping[Qt::Key_E] = Ogre::Vector3( 0, 1, 0);
    mKeyCoordinateMapping[Qt::Key_Q] = Ogre::Vector3( 0,-1, 0);
}

OgreWidget::~OgreWidget()
{
    QMutexLocker locker(&mMutex);

    if(ogreRenderWindow)
    {
        ogreRenderWindow->removeAllViewports();
    }

    if(ogreRoot)
    {
        ogreRoot->detachRenderTarget(ogreRenderWindow);

        if(ogreSceneManager)
        {
            ogreRoot->destroySceneManager(ogreSceneManager);
        }
    }

    delete ogreRoot;
}

void OgreWidget::setBackgroundColor(QColor c)
{
    qDebug() << "OgreWidget::setBackgroundColor()";
    QMutexLocker locker(&mMutex);
    if(ogreViewport)
    {
        Ogre::ColourValue ogreColour;
        ogreColour.setAsARGB(c.rgba());
        ogreViewport->setBackgroundColour(ogreColour);
    }
}

void OgreWidget::setCameraPosition(const Ogre::Vector3 &vector, const TranslationMode &mode, const Ogre::Node::TransformSpace &transformSpace, const Ogre::SceneNode* lookAt)
{
    qDebug() << "OgreWidget::setCameraPosition()";
    QMutexLocker locker(&mMutex);

    if(mode == OgreWidget::TRANSLATION_RELATIVE)
        mCameraNode->translate(vector, transformSpace);
    else if(mode == OgreWidget::TRANSLATION_ABSOLUTE)
        mCameraNode->setPosition(vector);

    if(lookAt) mCamera->lookAt(lookAt->getPosition());

    update();

    emit cameraPositionChanged(mCameraNode->getPosition());
}

Ogre::Vector3 OgreWidget::getVehiclePosition(void) const
{
    Q_ASSERT(false);
    QMutexLocker locker(&mMutex);
//    qDebug() << "OgreWidget::getVehiclePosition()";
    return mVehicleNode->getPosition();
}

void OgreWidget::setVehiclePosition(const Ogre::Vector3 &vector, const TranslationMode &mode, const Ogre::Node::TransformSpace &transformSpace, const bool reAimCamera)
{
    Q_ASSERT(false);
    qDebug() << "OgreWidget::setVehiclePosition()";
    QMutexLocker locker(&mMutex);

    if(mode == OgreWidget::TRANSLATION_RELATIVE)
        mVehicleNode->translate(vector, transformSpace);
    else if(mode == OgreWidget::TRANSLATION_ABSOLUTE)
        mVehicleNode->setPosition(vector);

    if(reAimCamera)
    {
        mCamera->lookAt(mVehicleNode->getPosition());
        emit cameraPositionChanged(mCameraNode->getPosition());
    }

    update();
}

void OgreWidget::keyPressEvent(QKeyEvent *e)
{
    QMutexLocker locker(&mMutex);

    // Only act if the pressed key is a movement key
    if(mKeyCoordinateMapping.find(e->key()) != mKeyCoordinateMapping.end())
    {
        e->accept();

        // A movement-key has been pressed, remember that and start the timer if its not already running.
        // In timerEvent(), the camera will be moved whenever the timer fires.
        mKeysPressed << e->key();
        if(! mUpdateTimerId) mUpdateTimerId = startTimer(1000/25);
    }
    else if(e->key() == Qt::ControlModifier)
    {
        // ControlModifier speeds up movement, but don't activate the timer for it
        // because by itself, it doesn't cause any movement
        qDebug() << "OgreWidget::keyPressEvent(): CTRL pressed";
        mKeysPressed << e->key();
    }
    else if(e->key() == Qt::Key_Space)
    {
        mCamera->lookAt(ogreSceneManager->getSceneNode("vehicleNode")->getPosition());
        update();
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::keyReleaseEvent(QKeyEvent *e)
{
    QMutexLocker locker(&mMutex);

    mKeysPressed.removeOne(e->key());

    if(mKeysPressed.empty() || (mKeysPressed.size() == 1 && mKeysPressed.contains(Qt::ControlModifier)))
    {
        killTimer(mUpdateTimerId);
        mUpdateTimerId = 0;
    }

    e->accept();
}

void OgreWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
    qDebug() << "OgreWidget::mouseDoubleClickEvent()";
    QMutexLocker locker(&mMutex);
    if(e->buttons().testFlag(Qt::LeftButton))
    {
        qDebug() << "OgreWidget::mouseDoubleClickEvent(): lmb";
        Ogre::Real x = e->pos().x() / (float)width();
        Ogre::Real y = e->pos().y() / (float)height();

        Ogre::Ray ray = mCamera->getCameraToViewportRay(x, y);
        Ogre::RaySceneQuery *query = ogreSceneManager->createRayQuery(ray);
        Ogre::RaySceneQueryResult &queryResult = query->execute();
        Ogre::RaySceneQueryResult::iterator queryResultIterator = queryResult.begin();

        if(queryResultIterator != queryResult.end())
        {
            if(queryResultIterator->movable)
            {
                selectedNode = queryResultIterator->movable->getParentSceneNode();
                selectedNode->showBoundingBox(true);
            }
        }
        else if(selectedNode)
        {
            selectedNode->showBoundingBox(false);
            selectedNode = 0;
        }

        ogreSceneManager->destroyQuery(query);

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
//    qDebug() << "OgreWidget::mouseMoveEvent()";
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
        update();

        oldPosL = pos;
        e->accept();
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
        update();

        oldPosR = pos;
        e->accept();
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

    if(e->isAccepted() && ogreRenderWindow)
    {
        ogreRenderWindow->windowMovedOrResized();
        update();
    }
}

QPaintEngine* OgreWidget::paintEngine() const
{
//    qDebug() << "OgreWidget::paintEngine()";

//    QMutexLocker locker(&mMutex);

    // We don't want another paint engine to get in the way for our Ogre based paint engine.
    // So we return nothing.
    return NULL;
}

void OgreWidget::timerEvent(QTimerEvent * e)
{
    QMutexLocker locker(&mMutex);
    qDebug() << "OgreWidget::timerEvent(): start";

    Q_ASSERT(! mKeysPressed.empty());

    for(int i=0; i < mKeysPressed.size(); ++i)
    {
        QMap<int, Ogre::Vector3>::iterator keyPressed = mKeyCoordinateMapping.find(mKeysPressed.at(i));
        if(keyPressed != mKeyCoordinateMapping.end() && mCamera)
        {
            if(QApplication::keyboardModifiers().testFlag(Qt::ControlModifier))
                mCameraNode->translate(keyPressed.value() * turboModifier);
            else
                mCameraNode->translate(keyPressed.value());

            update();
        }
    }

    e->accept();
//    qDebug() << "OgreWidget::timerEvent(): end";
}

void OgreWidget::paintEvent(QPaintEvent *e)
{
    QMutexLocker locker(&mMutex);
    if(!ogreRoot)
    {
        initOgreSystem();
    }

    // Construct all laserscanner rays before rendering
    QList<LaserScanner*> *laserScanners = mSimulator->getLaserScannerList();
    for(int i = 0; i < laserScanners->size(); ++i)
    {
        LaserScanner* scanner = laserScanners->at(i);
        Ogre::Ray beam = scanner->getCurrentLaserBeam();

        scanner->mRayObject->clear();
        scanner->mRayObject->begin(QString("RayFrom_" + scanner->objectName() + "_material").toStdString(), Ogre::RenderOperation::OT_LINE_LIST);
        scanner->mRayObject->position(beam.getPoint(0.0));
        scanner->mRayObject->position(beam.getPoint(scanner->range()));
        scanner->mRayObject->end();
    }

    ogreRoot->_fireFrameStarted();

    ogreRenderWindow->update();

    ogreRoot->_fireFrameEnded();

    if(mFrameCount++ % 25 == 0)
        emit currentRenderStatistics(size(), ogreRenderWindow->getTriangleCount(), ogreRenderWindow->getLastFPS());

    e->accept();
}

void OgreWidget::resizeEvent(QResizeEvent *e)
{
    QMutexLocker locker(&mMutex);
    qDebug() << "OgreWidget::resizeEvent()";
    QWidget::resizeEvent(e);

    if(e->isAccepted())
    {
        const QSize &newSize = e->size();
        if(ogreRenderWindow)
        {
            ogreRenderWindow->resize(newSize.width(), newSize.height());
            ogreRenderWindow->windowMovedOrResized();
        }
        if(mCamera)
        {
            Ogre::Real aspectRatio = Ogre::Real(newSize.width()) / Ogre::Real(newSize.height());
            mCamera->setAspectRatio(aspectRatio);
        }
    }
}

void OgreWidget::showEvent(QShowEvent *e)
{
    qDebug() << "OgreWidget::showEvent()";

    QMutexLocker locker(&mMutex);

    QWidget::showEvent(e);
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

void OgreWidget::initOgreSystem()
{
    qDebug() << "OgreWidget::initOgreSystem()";
    ogreRoot = new Ogre::Root();

    Ogre::RenderSystem *renderSystem = ogreRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
    ogreRoot->setRenderSystem(renderSystem);
    ogreRoot->initialise(false);

    Ogre::NameValuePairList viewConfig;
    Ogre::String widgetHandle;
#ifdef Q_WS_WIN
    widgetHandle = Ogre::StringConverter::toString((size_t)((HWND)winId()));
#else
    QWidget *q_parent = dynamic_cast <QWidget *> (parent());
    QX11Info xInfo = x11Info();

    widgetHandle = Ogre::StringConverter::toString ((unsigned long)xInfo.display()) +
        ":" + Ogre::StringConverter::toString ((unsigned int)xInfo.screen()) +
        ":" + Ogre::StringConverter::toString ((unsigned long)q_parent->winId());
#endif
    viewConfig["parentWindowHandle"] = widgetHandle;

    ogreRenderWindow = ogreRoot->createRenderWindow("Ogre rendering window", width(), height(), false, &viewConfig);

    Ogre::SceneManagerEnumerator::MetaDataIterator iter = Ogre::SceneManagerEnumerator::getSingleton().getMetaDataIterator();
    while( iter.hasMoreElements() )
    {
        Ogre::String st = iter.getNext()->typeName;
        printf("Scene manager type available: %s\n\n",st.c_str());
    }

    ogreSceneManager = ogreRoot->createSceneManager(Ogre::ST_GENERIC);
//    ogreSceneManager = ogreRoot->createSceneManager("TerrainSceneManager"/*Ogre::ST_EXTERIOR_CLOSE*/);
//    ogreSceneManager->showBoundingBoxes(true);

    mCamera = ogreSceneManager->createCamera("camera");
    mCamera->setNearClipDistance(.1);
    mCamera->setPolygonMode(Ogre::PM_WIREFRAME);     /* wireframe */
    mCamera->setPolygonMode(Ogre::PM_SOLID);         /* solid */


    mCameraNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode", Ogre::Vector3(0, 12, 15));
    mCameraNode->attachObject(mCamera);
    mCamera->lookAt(0,8,0);

    ogreViewport = ogreRenderWindow->addViewport(mCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(0.7, 0.7, 0.7));
    mCamera->setAspectRatio(Ogre::Real(width()) / Ogre::Real(height()));

    loadResources();

    // Initialize shader generator.
    // Must be before resource loading in order to allow parsing extended material attributes.
    bool success = initializeRTShaderSystem(ogreSceneManager);
    if (!success)
    {
        OGRE_EXCEPT(Ogre::Exception::ERR_FILE_NOT_FOUND,
                    "Shader Generator Initialization failed - Core shader libs path not found",
                    "SdkSample::_setup");
    }


    createScene();

    setupTerrain();

    emit setupFinished();

    qDebug() << "OgreWidget::initOgreSystem(): done";
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
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
            // OS X does not set the working directory relative to the app,
            // In order to make things portable on OS X we need to provide
            // the loading with it's own bundle path location
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(Ogre::String(macBundlePath() + "/" + archName), typeName, secName);
#else
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
#endif
        }
    }

    // Initialise, parse scripts etc
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    qDebug() << "OgreWidget::loadResources(): done.";
}

void OgreWidget::createScene()
{
    qDebug() << "OgreWidget::createScene()";
//    ogreSceneManager->setAmbientLight(Ogre::ColourValue(1,1,1));

//    ogreSceneManager->setWorldGeometry("media/terrain.cfg");
//    ogreSceneManager->getSceneNode("Terrain")->showBoundingBox(true);

    qDebug() << "OgreWidget::createScene(): done.";
}

Ogre::RaySceneQuery* OgreWidget::createRaySceneQuery(void)
{
    qDebug() << "OgreWidget::createRaySceneQuery(): returning pointer.";

//    QMutexLocker locker(&mMutex);
    return ogreSceneManager->createRayQuery(Ogre::Ray());
}

Ogre::SceneNode* OgreWidget::createScannerNode(const QString name, const Ogre::Vector3 &relativePosition, const Ogre::Quaternion &relativeRotation)
{
    qDebug() << "OgreWidget::createScannerNode()";

    QMutexLocker locker(&mMutex);

    // We don't need names, so we just create something random
    Ogre::Entity *scannerEntity = ogreSceneManager->createEntity(QString(name + "_entity").toStdString(), "hokuyoutm30lx.mesh");
    Ogre::SceneNode *scannerNode = ogreSceneManager->getSceneNode("vehicleNode")->createChildSceneNode(QString(name + "_node").toStdString(), relativePosition, relativeRotation);
    scannerNode->attachObject(scannerEntity);
    qDebug() << "OgreWidget::createScannerNode(): done, returning";

    return scannerNode;
}

void OgreWidget::createManualObject(const QString &name, Ogre::ManualObject** manualObject, Ogre::SceneNode** sceneNode, Ogre::MaterialPtr &material)
{
    qDebug() << "OgreWidget::createManualObject(), my address is" << this;

    QMutexLocker locker(&mMutex);

    *manualObject =  ogreSceneManager->createManualObject(QString(name+"_manualobject").toStdString());
    *sceneNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode(QString(name+"_scenenode").toStdString());
    material = Ogre::MaterialManager::getSingleton().create(QString(name+"_material").toStdString(), "debuggergroup");
    qDebug() << "OgreWidget::createManualObject(): done, name of SceneNode is" << QString::fromStdString((*sceneNode)->getName());
}

Ogre::SceneManager* OgreWidget::sceneManager()
{
    return ogreSceneManager;
}



#define TERRAIN_PAGE_MIN_X 0
#define TERRAIN_PAGE_MIN_Y 0
#define TERRAIN_PAGE_MAX_X 0
#define TERRAIN_PAGE_MAX_Y 0

#include "Terrain/OgreTerrain.h"
#include "Terrain/OgreTerrainGroup.h"
#include "Terrain/OgreTerrainQuadTreeNode.h"
#include "Terrain/OgreTerrainMaterialGeneratorA.h"
#include "Terrain/OgreTerrainPaging.h"

#include "Paging/OgrePagedWorldSection.h"
#include <RTShaderSystem/OgreShaderGenerator.h>

#define TERRAIN_FILE_PREFIX String("testTerrain")
#define TERRAIN_FILE_SUFFIX String("dat")
#define TERRAIN_WORLD_SIZE 12000.0f
#define TERRAIN_SIZE 513


void OgreWidget::setupTerrain()
{
        bool blankTerrain = false;
        //blankTerrain = true;

        mTerrainGlobals = new Ogre::TerrainGlobalOptions();

        mEditMarker = ogreSceneManager->createEntity("editMarker", "sphere.mesh");
        mEditNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode();
        mEditNode->attachObject(mEditMarker);
        mEditNode->setScale(0.05, 0.05, 0.05);

//        setupControls();

//        mCameraMan->setTopSpeed(50);

//        setDragLook(true);

        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(7);

        ogreSceneManager->setFog(Ogre::FOG_LINEAR, Ogre::ColourValue(0.7, 0.7, 0.8), 0, 10000, 25000);

        Ogre::LogManager::getSingleton().setLogDetail(Ogre::LL_BOREME);

        Ogre::Vector3 lightdir(0.55, -0.3, 0.75);
        lightdir.normalise();


        Ogre::Light* l = ogreSceneManager->createLight("tstLight");
        l->setType(Ogre::Light::LT_DIRECTIONAL);
        l->setDirection(lightdir);
        l->setDiffuseColour(Ogre::ColourValue::White);
        l->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

        ogreSceneManager->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));


        mTerrainGroup = new Ogre::TerrainGroup(ogreSceneManager, Ogre::Terrain::ALIGN_X_Z, TERRAIN_SIZE, TERRAIN_WORLD_SIZE);
        mTerrainGroup->setFilenameConvention(Ogre::TERRAIN_FILE_PREFIX, Ogre::TERRAIN_FILE_SUFFIX);
        mTerrainGroup->setOrigin(mTerrainPos);

        configureTerrainDefaults(l);
#ifdef PAGING
        // Paging setup
        mPageManager = new PageManager();
        // Since we're not loading any pages from .page files, we need a way just
        // to say we've loaded them without them actually being loaded
        mPageManager->setPageProvider(&mDummyPageProvider);
        mPageManager->addCamera(mCamera);
        mTerrainPaging = new Ogre::TerrainPaging(mPageManager);
        PagedWorld* world = mPageManager->createWorld();
        mTerrainPaging->createWorldSection(world, mTerrainGroup, 2000, 3000,
                TERRAIN_PAGE_MIN_X, TERRAIN_PAGE_MIN_Y,
                TERRAIN_PAGE_MAX_X, TERRAIN_PAGE_MAX_Y);
#else
        for (long x = TERRAIN_PAGE_MIN_X; x <= TERRAIN_PAGE_MAX_X; ++x)
                for (long y = TERRAIN_PAGE_MIN_Y; y <= TERRAIN_PAGE_MAX_Y; ++y)
                        defineTerrain(x, y, blankTerrain);
        // sync load since we want everything in place when we start
        mTerrainGroup->loadAllTerrains(true);
#endif

        if (mTerrainsImported)
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
        Ogre::Entity* e = ogreSceneManager->createEntity("tudorhouse.mesh");
        Ogre::Vector3 entPos(mTerrainPos.x + 2043, 0, mTerrainPos.z + 1715);
        Ogre::Quaternion rot;
        entPos.y = mTerrainGroup->getHeightAtWorldPosition(entPos) + 65.5 + mTerrainPos.y;
        rot.FromAngleAxis(Ogre::Degree(Ogre::Math::RangeRandom(-180, 180)), Ogre::Vector3::UNIT_Y);
        Ogre::SceneNode* sn = ogreSceneManager->getRootSceneNode()->createChildSceneNode(entPos, rot);
        sn->setScale(Ogre::Vector3(0.12, 0.12, 0.12));
        sn->attachObject(e);
        mHouseList.push_back(e);

        ogreSceneManager->setSkyBox(true, "Examples/CloudyNoonSkyBox");


}

void OgreWidget::configureTerrainDefaults(Ogre::Light* l)
        {
                // Configure global
                mTerrainGlobals->setMaxPixelError(8);
                // testing composite map
                mTerrainGlobals->setCompositeMapDistance(3000);
                //mTerrainGlobals->setUseRayBoxDistanceCalculation(true);
                //mTerrainGlobals->getDefaultMaterialGenerator()->setDebugLevel(1);
                //mTerrainGlobals->setLightMapSize(256);

                //matProfile->setLightmapEnabled(false);
                // Important to set these so that the terrain knows what to use for derived (non-realtime) data
                mTerrainGlobals->setLightMapDirection(l->getDerivedDirection());
                mTerrainGlobals->setCompositeMapAmbient(ogreSceneManager->getAmbientLight());
                //mTerrainGlobals->setCompositeMapAmbient(ColourValue::Red);
                mTerrainGlobals->setCompositeMapDiffuse(l->getDiffuseColour());

                // Configure default import settings for if we use imported image
                Ogre::Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
                defaultimp.terrainSize = TERRAIN_SIZE;
                defaultimp.worldSize = TERRAIN_WORLD_SIZE;
                defaultimp.inputScale = 600;
                defaultimp.minBatchSize = 33;
                defaultimp.maxBatchSize = 65;
                // textures
                defaultimp.layerList.resize(3);
                defaultimp.layerList[0].worldSize = 100;
                defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
                defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
                defaultimp.layerList[1].worldSize = 30;
                defaultimp.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
                defaultimp.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
                defaultimp.layerList[2].worldSize = 200;
                defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
                defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");


        }

void OgreWidget::defineTerrain(long x, long y, bool flat)
        {
                // if a file is available, use it
                // if not, generate file from import

                // Usually in a real project you'll know whether the compact terrain data is
                // available or not; I'm doing it this way to save distribution size

                if (flat)
                {
                        mTerrainGroup->defineTerrain(x, y, 0.0f);
                }
                else
                {
                    Ogre::String filename = mTerrainGroup->generateFilename(x, y);
                        if (Ogre::ResourceGroupManager::getSingleton().resourceExists(mTerrainGroup->getResourceGroup(), filename))
                        {
                                mTerrainGroup->defineTerrain(x, y);
                        }
                        else
                        {
                                Ogre::Image img;
                                getTerrainImage(x % 2 != 0, y % 2 != 0, img);
                                mTerrainGroup->defineTerrain(x, y, &img);
                                mTerrainsImported = true;
                        }

                }
        }

void OgreWidget::getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
    img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        if (flipX)
                img.flipAroundY();
        if (flipY)
                img.flipAroundX();

}

void OgreWidget::initBlendMaps(Ogre::Terrain* terrain)
{
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
        //blendMap0->loadImage("blendmap1.png", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        blendMap0->update();
        blendMap1->update();

        // set up a colour map
        /*
        if (!terrain->getGlobalColourMapEnabled())
        {
                terrain->setGlobalColourMapEnabled(true);
                Image colourMap;
                colourMap.load("testcolourmap.jpg", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                terrain->getGlobalColourMap()->loadImage(colourMap);
        }
        */

}

/*-----------------------------------------------------------------------------
| Initialize the RT Shader system.
-----------------------------------------------------------------------------*/
/*virtual*/ bool OgreWidget::initializeRTShaderSystem(Ogre::SceneManager* sceneMgr)
{
        if (Ogre::RTShader::ShaderGenerator::initialize())
        {
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
                                if ((*it)->archive->getName().find("RTShaderLib") != Ogre::String::npos)
                                {
                                        shaderCoreLibsPath = (*it)->archive->getName() + "/";
                                        shaderCachePath = shaderCoreLibsPath;
                                        coreLibsFound = true;
                                        break;
                                }
                        }
                        // Core libs path found in the current group.
                        if (coreLibsFound)
                                break;
                }

                // Core shader libs not found -> shader generating will fail.
                if (shaderCoreLibsPath.empty())
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
