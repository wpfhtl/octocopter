#include "camera.h"

Camera::Camera(Simulator* simulator, OgreWidget* ogreWidget, const int width, const int height, const int interval) :
        mWidth(width), mHeight(height), mInterval(interval)
{
    qDebug() << "Camera::Camera()";

    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    // We'll need a unique name, so why not use our own address...
    char address[50];
    sprintf(address, "%p", (void*)this);
    setObjectName("Camera_at_" + QString(address));

    mOgreWidget->createRttCamera(&mCamera, &mRenderTarget, objectName(), 800, 600);
}


Camera::~Camera()
{
    qDebug() << "Camera::~Camera()";
    mOgreWidget->destroyCamera(mRenderTarget, mCamera);
//    mOgreWidget->destroyManualObject(mRayObject, mRayNode);
    mUdpSocket->deleteLater();
    mTimerShutter->deleteLater();
}

void Camera::run()
{
    mUdpSocket = new QUdpSocket;
    mUdpSocket->bind();

    mTimerShutter = new QTimer;
    mTimerShutter->setInterval(mInterval);
    connect(mTimerShutter, SIGNAL(timeout()), SLOT(slotRecordImage()));

//    mOgreWidget->createRttCamera(&mCamera, &mRenderTarget, objectName(), 800, 600);

    if(!mSimulator->isPaused()) mTimerShutter->start();

    qDebug() << "Camera::run(): starting Camera eventloop in threadid" << currentThreadId();
    exec();
    qDebug() << "Camera::run(): eventloop finished in threadid" << currentThreadId();
}

void Camera::slotRecordImage()
{
    qDebug() << "Camera::slotRecordImage(): updating renderTarget and saving in threadid" << currentThreadId();
    mRenderTarget->update();
    mRenderTarget->writeContentsToFile("rt.png");
    // This scan is finished, emit it...
//    emit scanFinished(mScanData);
//    QByteArray datagram;
//    QDataStream stream(&datagram, QIODevice::WriteOnly);

    // How many hits?
//    stream << (qint32)mScanData.size();
    // From which position?
//    stream << QVector3D(mCameraPosition.x, mCameraPosition.y, mCameraPosition.z);
    // Stream the hits


//    mUdpSocket->writeDatagram(datagram, QHostAddress::Broadcast, 45454);
//    mUdpSocket->flush();
}

void Camera::slotPause(void)
{
    qDebug() << "Camera::slotPause(): stopping Camera";
    mTimerShutter->stop();
//    mContinue = false;
}

void Camera::slotStart(void)
{
    qDebug() << "Camera::slotStart(): starting Camera timer in thread" << currentThreadId();
    mTimerShutter->start();
}

//Ogre::SceneNode* Camera::getSceneNode(void)
//{
//    return mCameraNode;
//}

void Camera::setTimeFactor(float timeFactor)
{
    QMutexLocker locker(&mMutex);
    mTimeFactor = timeFactor;
}

void Camera::setPosition(const Ogre::Vector3 &position)
{
    mCamera->setPosition(position);
}

void Camera::setDirection(const Ogre::Vector3 &direction)
{
    mCamera->setDirection(direction);
}

Ogre::Vector3 Camera::getPosition(void) const
{
    return mCamera->getPosition();
}

Ogre::Vector3 Camera::getDirection(void) const
{
    return mCamera->getDirection();
}

int Camera::width(void) const { return mWidth; }
int Camera::height(void) const { return mHeight; }
int Camera::interval(void) const { return mInterval; }

void Camera::setWidth(const int width) { mWidth = width; }
void Camera::setHeight(const int height) { mHeight = height; }
void Camera::setInterval(const int interval) {mInterval = interval; }
