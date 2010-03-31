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
        ogreCamera(0),
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

    if(lookAt) ogreCamera->lookAt(lookAt->getPosition());

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
        ogreCamera->lookAt(mVehicleNode->getPosition());
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
        ogreCamera->lookAt(ogreSceneManager->getSceneNode("vehicleNode")->getPosition());
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

        mCameraNode->translate(ogreCamera->getOrientation() * Ogre::Vector3(deltaX/50, -deltaY/50, deltaZ/50), Ogre::Node::TS_LOCAL);
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
        ogreCamera->yaw(Ogre::Degree(-diffX));
        ogreCamera->pitch(Ogre::Degree(-diffY));
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
        if(keyPressed != mKeyCoordinateMapping.end() && ogreCamera)
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

    ogreSceneManager = ogreRoot->createSceneManager("TerrainSceneManager"/*Ogre::ST_EXTERIOR_CLOSE*/);

    ogreCamera = ogreSceneManager->createCamera("camera");
    ogreCamera->setNearClipDistance(.1);

    mCameraNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode", Ogre::Vector3(0, 12, 15));
    mCameraNode->attachObject(ogreCamera);
    ogreCamera->lookAt(0,8,0);

    ogreViewport = ogreRenderWindow->addViewport(ogreCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(0.7, 0.7, 0.7));
    ogreCamera->setAspectRatio(Ogre::Real(width()) / Ogre::Real(height()));

    loadResources();
    createScene();

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

    ogreSceneManager->setWorldGeometry("media/terrain.cfg");
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
