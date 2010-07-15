#include "camera.h"

Camera::Camera(Simulator* simulator, OgreWidget* ogreWidget, const QSize size, const float fovY, const int interval) :
        mFovY(fovY), mPixelBox(0), mImage(0)
{

    mSimulator = simulator;
    mOgreWidget = ogreWidget;

    qDebug() << "Camera::Camera(): setting camera timer to" << mInterval * (1.0 / mSimulator->getTimeFactor());

    // We'll need a unique name, so why not use our own address...
    char address[50];
    sprintf(address, "%p", (void*)this);
    setObjectName("Camera_at_" + QString(address));

    mUdpSocket = new QUdpSocket;
    mUdpSocket->bind();

    mTimerShutter = new QTimer;
    connect(mTimerShutter, SIGNAL(timeout()), SLOT(slotRecordImage()));

    mOgreWidget->createRttCamera(&mCamera, &mRenderTarget, &mCameraNode, objectName(), size);
    setFovY(mFovY);
    mCamera->setNearClipDistance(0.01);

    setImageSize(size);
    setInterval(interval);

    // Make sure the camera is attached (to mCameraNode), so that it inherits the sceneNodes (=the vehicle's) motions
    Q_ASSERT(mCamera->isAttached() && mCamera->getParentSceneNode() == mCameraNode);

    if(!mSimulator->isPaused()) mTimerShutter->start();
}


Camera::~Camera()
{
    qDebug() << "Camera::~Camera()";
    mOgreWidget->destroyRttCamera(objectName()); // are we leaking the rendertarget?

    mUdpSocket->deleteLater();
    mTimerShutter->deleteLater();
    delete mImage;
    delete mPixelBox;
}
/*
void Camera::run()
{
    mUdpSocket = new QUdpSocket;
    mUdpSocket->bind();

    mTimerShutter = new QTimer;
    mTimerShutter->setInterval(mInterval);
    connect(mTimerShutter, SIGNAL(timeout()), SLOT(slotRecordImage()));

//    mOgreWidget->createRttCamera(&mCamera, &mRenderTarget, objectName(), 800, 600);

    if(!mSimulator->isPaused()) mTimerShutter->start();

//    qDebug() << "Camera::run(): starting Camera eventloop in threadid" << currentThreadId();
//    exec();
//    qDebug() << "Camera::run(): eventloop finished in threadid" << currentThreadId();
}
*/

void Camera::slotRecordImage()
{
    qDebug() << "Camera::slotRecordImage(): updating renderTarget from" << mCamera->getDerivedPosition().x << mCamera->getDerivedPosition().y << mCamera->getDerivedPosition().z;
    mRenderTarget->update();
//    mRenderTarget->writeContentsToFile("rt.png");
    // Copy to PixelBox, which is backed by mImageBuffer, which backs mImage
    mRenderTarget->copyContentsToMemory(*mPixelBox, Ogre::RenderTarget::FB_AUTO);

    QByteArray datagram;
    QDataStream stream(&datagram, QIODevice::WriteOnly);

    // Stream camera name
    stream << objectName();

    // Stream the pose
    stream << QVector3D(mCameraNode->_getDerivedPosition().x, mCameraNode->_getDerivedPosition().y, mCameraNode->_getDerivedPosition().z);
    stream << QQuaternion(mCameraNode->_getDerivedOrientation().w, mCameraNode->_getDerivedOrientation().x, mCameraNode->_getDerivedOrientation().y, mCameraNode->_getDerivedOrientation().z);

    // Stream the image
    QByteArray imageArray;
    QBuffer buffer(&imageArray);
    mImage->save(&buffer, "jpg", 90);
    datagram.append(imageArray);
    qDebug() << "Camera::slotRecordImage(): number of bytes for image" << imageArray.size();

    // Now set the length of the datagram
    QByteArray datagramLengthArray;
    QDataStream streamLength(&datagramLengthArray, QIODevice::WriteOnly);
    streamLength << (qint32)(datagram.length() + sizeof(qint32));
    datagram.prepend(datagramLengthArray);

    qDebug() << "Camera::slotRecordImage(): datagram size:" << datagram.size();

    mUdpSocket->writeDatagram(datagram, QHostAddress::Broadcast, 11111);
    mUdpSocket->flush();
}

void Camera::slotPause(void)
{
    qDebug() << "Camera::slotPause(): stopping Camera";
    mTimerShutter->stop();
//    mContinue = false;
}

void Camera::slotStart(void)
{
    qDebug() << "Camera::slotStart(): starting Camera timer";// in thread" << currentThreadId();
    mTimerShutter->start();
}

//Ogre::SceneNode* Camera::getSceneNode(void)
//{
//    return mCameraNode;
//}

void Camera::slotSetTimeFactor(double timeFactor)
{
    QMutexLocker locker(&mMutex);
    mTimeFactor = timeFactor;
    mTimerShutter->setInterval(mInterval * (1.0 / mSimulator->getTimeFactor()));
}

void Camera::setPosition(const Ogre::Vector3 &position)
{
    // NOTTRUEANYMORE: The position is meant relative to the vehicle-scenenode, which is pointed to by mCameraNode
    qDebug() << "Camera::setPosition() derived orientation" << mCamera->getDerivedOrientation().getPitch().valueDegrees() << mCamera->getDerivedOrientation().getRoll().valueDegrees() << mCamera->getDerivedOrientation().getYaw().valueDegrees();
    qDebug() << "Camera::setPosition() setting position to" << position.x << position.y << position.z;
    Q_ASSERT(mCamera->isAttached());
    mCameraNode->setPosition(position);
    qDebug() << "Camera::setPosition() derived position is" << mCamera->getDerivedPosition().x << mCamera->getDerivedPosition().y << mCamera->getDerivedPosition().z;
    qDebug() << "Camera::setPosition() own position is" << mCamera->getPosition().x << mCamera->getPosition().y << mCamera->getPosition().z;
}

void Camera::setOrientation(const Ogre::Quaternion &orientation)
{
    qDebug() << "Camera::setOrientation() derived orientation is" << mCameraNode->_getDerivedOrientation().getPitch().valueDegrees() << mCameraNode->_getDerivedOrientation().getRoll().valueDegrees() << mCameraNode->_getDerivedOrientation().getYaw().valueDegrees();
    qDebug() << "Camera::setOrientation() setting orientation true  to" << orientation.getPitch().valueDegrees() << orientation.getRoll().valueDegrees() << orientation.getYaw().valueDegrees();
    qDebug() << "Camera::setOrientation() setting orientation false to" << orientation.getPitch(false).valueDegrees() << orientation.getRoll(false).valueDegrees() << orientation.getYaw(false).valueDegrees();
    Q_ASSERT(mCamera->isAttached());
    mCameraNode->setOrientation(orientation);
    qDebug() << "Camera::setOrientation() derived orientation is" << mCameraNode->_getDerivedOrientation().getPitch().valueDegrees() << mCameraNode->_getDerivedOrientation().getRoll().valueDegrees() << mCameraNode->_getDerivedOrientation().getYaw().valueDegrees();
}

Ogre::Vector3 Camera::getPosition(void) const
{
    return mCameraNode->getPosition();// - mCameraNode->_getDerivedPosition();
}

Ogre::Quaternion Camera::getOrientation(void) const
{
    return mCameraNode->getOrientation();
}

QSize Camera::imageSize(void) const { return mSize; }
float Camera::fovY(void) const { return mFovY; }
int Camera::interval(void) const { return mInterval; }

void Camera::setImageSize(const QSize size)
{
    mSize = size;
    mImageBuffer.resize(size.width() * size.height() * 3); // 3 bytes per pixel, see format below
    if(mPixelBox) delete mPixelBox;
    // Construct a new PixelBox, and let it write into our mImageBuffer QByteArray
    mPixelBox = new Ogre::PixelBox(size.width(), size.height(), 1, Ogre::PF_R8G8B8, mImageBuffer.data());
    if(mImage) delete mImage;
    mImage = new QImage((const uchar*)mImageBuffer.data(), mSize.width(), mSize.height(), QImage::Format_RGB888);
}
void Camera::setFovY(const float fovY) { mFovY = fovY; mCamera->setFOVy(Ogre::Degree(mFovY)); }
void Camera::setInterval(const int interval)
{
    mInterval = interval;
    mTimerShutter->setInterval(mInterval * (1.0 / mSimulator->getTimeFactor()));
    qDebug() << "Camera::setInterval(): setiing camera timer to" << mInterval * (1.0 / mSimulator->getTimeFactor());
}
