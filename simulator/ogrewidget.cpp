#include "ogrewidget.h"

const QPoint     OgreWidget::invalidMousePoint(-1,-1);
const Ogre::Real OgreWidget::turboModifier(10);

OgreWidget::OgreWidget(QWidget *parent) : QWidget(parent), ogreRoot(0), ogreSceneManager(0), ogreRenderWindow(0), ogreViewport(0), ogreCamera(0), oldPos(invalidMousePoint), selectedNode(0)
{
    qDebug() << "OgreWidget::OgreWidget()";
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_PaintOnScreen);
    setMinimumSize(240,240);
    setFocusPolicy(Qt::ClickFocus);
}

OgreWidget::~OgreWidget()
{
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
    if(ogreViewport)
    {
        Ogre::ColourValue ogreColour;
        ogreColour.setAsARGB(c.rgba());
        ogreViewport->setBackgroundColour(ogreColour);
    }
}

void OgreWidget::setCameraPosition(const Ogre::Vector3 &vector, const TranslationMode &mode, const Ogre::Node::TransformSpace &transformSpace, const Ogre::SceneNode* lookAt)
{
//    qDebug() << "OgreWidget::setCameraPosition()";

    if(mode == OgreWidget::TRANSLATION_RELATIVE)
        mCameraNode->translate(vector, transformSpace);
    else if(mode == OgreWidget::TRANSLATION_ABSOLUTE)
        mCameraNode->setPosition(vector);

    if(lookAt) ogreCamera->lookAt(lookAt->getPosition());

    update();

    emit cameraPositionChanged(mCameraNode->getPosition());
}

void OgreWidget::setVehiclePosition(const Ogre::Vector3 &vector, const TranslationMode &mode, const Ogre::Node::TransformSpace &transformSpace, const bool reAimCamera)
{
//    qDebug() << "OgreWidget::setVehiclePosition()";

    if(mode == OgreWidget::TRANSLATION_RELATIVE)
        mVehicleNode->translate(vector, transformSpace);
    else if(mode == OgreWidget::TRANSLATION_ABSOLUTE)
        mVehicleNode->setPosition(vector);

    if(reAimCamera)
    {
        ogreCamera->lookAt(mVehicleNode->getPosition());
        emit cameraPositionChanged(mCameraNode->getPosition());
    }

    update();
}

void OgreWidget::keyPressEvent(QKeyEvent *e)
{
    static QMap<int, Ogre::Vector3> keyCoordModificationMapping;
    static bool mappingInitialised = false;

    if(!mappingInitialised)
    {
        keyCoordModificationMapping[Qt::Key_W] = Ogre::Vector3( 0, 0,-5);
        keyCoordModificationMapping[Qt::Key_S] = Ogre::Vector3( 0, 0, 5);
        keyCoordModificationMapping[Qt::Key_A] = Ogre::Vector3(-5, 0, 0);
        keyCoordModificationMapping[Qt::Key_D] = Ogre::Vector3( 5, 0, 0);

        keyCoordModificationMapping[Qt::Key_E] = Ogre::Vector3( 0, 5, 0);
        keyCoordModificationMapping[Qt::Key_Q] = Ogre::Vector3( 0,-5, 0);
        mappingInitialised = true;
    }

    QMap<int, Ogre::Vector3>::iterator keyPressed = keyCoordModificationMapping.find(e->key());
    if(keyPressed != keyCoordModificationMapping.end() && ogreCamera)
    {
        if(e->modifiers().testFlag(Qt::ControlModifier))
            setCameraPosition(keyPressed.value() * turboModifier);
        else
            setCameraPosition(keyPressed.value());

        e->accept();
    }
    else if(e->key() == Qt::Key_Space)
    {
        ogreCamera->lookAt(mVehicleNode->getPosition());
        update();
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
    if(e->buttons().testFlag(Qt::LeftButton))
    {
        qDebug() << "OgreWidget::mouseDoubleClickEvent(): lmb";
        Ogre::Real x = e->pos().x() / (float)width();
        Ogre::Real y = e->pos().y() / (float)height();

        Ogre::Ray ray = ogreCamera->getCameraToViewportRay(x, y);
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
        else
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
    if(e->buttons().testFlag(Qt::LeftButton) && oldPos != invalidMousePoint)
    {
        const QPoint &pos = e->pos();
        Ogre::Real deltaX = pos.x() - oldPos.x();
        Ogre::Real deltaY = pos.y() - oldPos.y();
        Ogre::Real deltaZ = pos.y() - oldPos.y();

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

        setCameraPosition(Ogre::Vector3(deltaX, -deltaY, deltaZ));

        oldPos = pos;
        e->accept();
    }
    else if(e->buttons().testFlag(Qt::RightButton) && oldPos != invalidMousePoint)
    {
        // rotate the camera
        const QPoint &pos = e->pos();
        qDebug() << "OgreWidget::mouseMoveEvent(): rotating camera by" << pos.x() - oldPos.x() / 5.0 << pos.x() - oldPos.x() / 5.0;
        ogreCamera->yaw(Ogre::Degree(pos.x() - oldPos.x() / 5.0));
        ogreCamera->pitch(Ogre::Degree(pos.y() - oldPos.y() / 5.0));

        oldPos = pos;
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mousePressEvent(QMouseEvent *e)
{
    if(e->buttons().testFlag(Qt::LeftButton))
    {
        oldPos = e->pos();
        e->accept();
    }
    else
    {
        e->ignore();
    }
}

void OgreWidget::mouseReleaseEvent(QMouseEvent *e)
{
    if(!e->buttons().testFlag(Qt::LeftButton))
    {
        oldPos = QPoint(invalidMousePoint);
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
    QWidget::moveEvent(e);

    if(e->isAccepted() && ogreRenderWindow)
    {
        ogreRenderWindow->windowMovedOrResized();
        update();
    }
}

QPaintEngine* OgreWidget::paintEngine() const
{
    qDebug() << "OgreWidget::paintEngine()";
    // We don't want another paint engine to get in the way for our Ogre based paint engine.
    // So we return nothing.
    return NULL;
}

void OgreWidget::timerEvent( QTimerEvent * e)
{
//    qDebug() << "OgreWidget::timerEvent(): start";
    ogreRoot->renderOneFrame();

    e->accept();
//    qDebug() << "OgreWidget::timerEvent(): end";
}

void OgreWidget::paintEvent(QPaintEvent *e)
{
    if(!ogreRoot)
    {
        initOgreSystem();
    }

//    qDebug() << "OgreWidget::paintEvent(): start";
    //ogreRoot->renderOneFrame();
    ogreRoot->_fireFrameStarted();
        ogreRenderWindow->update();
    ogreRoot->_fireFrameEnded();

    emit currentRenderStatistics(size(), ogreRenderWindow->getTriangleCount(), ogreRenderWindow->getLastFPS());

    e->accept();
//    qDebug() << "OgreWidget::paintEvent(): end";
}

void OgreWidget::resizeEvent(QResizeEvent *e)
{
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
        if(ogreCamera)
        {
            Ogre::Real aspectRatio = Ogre::Real(newSize.width()) / Ogre::Real(newSize.height());
            ogreCamera->setAspectRatio(aspectRatio);
        }
    }
}

void OgreWidget::showEvent(QShowEvent *e)
{
    qDebug() << "OgreWidget::showEvent()";
//    if(!ogreRoot)
//    {
//        initOgreSystem();
//    }

    QWidget::showEvent(e);
}

void OgreWidget::wheelEvent(QWheelEvent *e)
{
    qDebug() << "OgreWidget::wheelEvent(): " << -e->delta() / 60;

    Ogre::Vector3 zTranslation(0,0, -e->delta() / 60);

    if(e->modifiers().testFlag(Qt::ControlModifier))
        setCameraPosition(zTranslation * turboModifier);
    else
        setCameraPosition(zTranslation * turboModifier);

    e->accept();
}

void OgreWidget::initOgreSystem()
{
    qDebug() << "OgreWidget::initOgreSystem()";
    ogreRoot = new Ogre::Root();

    Ogre::RenderSystem *renderSystem = ogreRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
    ogreRoot->setRenderSystem(renderSystem);
    ogreRoot->initialise(false);
//    ogreRoot->initialise(true, "test");

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
//    ogreRenderWindow = ogreRoot->getAutoCreatedWindow();

    Ogre::SceneManagerEnumerator::MetaDataIterator iter = Ogre::SceneManagerEnumerator::getSingleton().getMetaDataIterator();
    while( iter.hasMoreElements() )
    {
        Ogre::String st = iter.getNext()->typeName;
        printf("Scene manager type available: %s\n\n",st.c_str());
    }

    ogreSceneManager = ogreRoot->createSceneManager("TerrainSceneManager"/*Ogre::ST_EXTERIOR_CLOSE*/);

    ogreCamera = ogreSceneManager->createCamera("camera");
    ogreCamera->setNearClipDistance(5);

    mCameraNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode", Ogre::Vector3(0, 0, 100));
    mCameraNode->attachObject(ogreCamera);
//    ogreCamera->lookAt(0,50,0);

    ogreViewport = ogreRenderWindow->addViewport(ogreCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(0,0,0));
    ogreCamera->setAspectRatio(Ogre::Real(width()) / Ogre::Real(height()));

    loadResources();
    createScene();

    startTimer(50);
}

void OgreWidget::loadResources()
{
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
}

void OgreWidget::createScene()
{
    qDebug() << "OgreWidget::createScene()";
    ogreSceneManager->setAmbientLight(Ogre::ColourValue(1,1,1));

    ogreSceneManager->setWorldGeometry("media/terrain.cfg");

    mVehicleEntity = ogreSceneManager->createEntity("Robot", "quad.mesh");
    mVehicleNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode("VehicleNode");
    mVehicleNode->attachObject(mVehicleEntity);
    mVehicleNode->setPosition(0.0, 0.0, 0.0);
    //mCopterNode->yaw(Ogre::Radian(Ogre::Degree(-90)));
}

Ogre::SceneNode* OgreWidget::getVehicleNode(void)
{
    //fixme: we return an invalid pointer, as initialization is still ahead of us.
    return mVehicleNode;
}

Ogre::RaySceneQuery* OgreWidget::createRaySceneQuery(void)
{
    return ogreSceneManager->createRayQuery(Ogre::Ray());
}

Ogre::SceneNode* OgreWidget::createScannerNode()
{
    // We don't need names, so we just create something random
    Ogre::Entity *scannerEntity = ogreSceneManager->createEntity(QDateTime::currentDateTime().toString("sszzz").toStdString(), "laserscanner.mesh");
    Ogre::SceneNode *scannerNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode("VehicleNode");
    scannerNode->attachObject(scannerEntity);
    scannerNode->setPosition(0.0, 0.0, 0.0);

    return scannerNode;
}
