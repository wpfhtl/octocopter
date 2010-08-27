#ifndef CAMERA_H
#define CAMERA_H

#include <QtCore>
#include <QtNetwork>
#include <Ogre.h>
#include <OgreTerrainGroup.h>

#include "simulator.h"
#include "ogrewidget.h"
#include "coordinateconverter.h"

class Simulator;
class OgreWidget;

class Camera : public QObject//QThread
{
    Q_OBJECT

private:
    Simulator *mSimulator;

    QMutex mMutex;

    QTcpSocket* mTcpSocket;
    QImage* mImage;
    QByteArray mImageBuffer, mNetworkPayload;

    QSize mSize;
    int mInterval; // the interval in realtime-milliseconds. mTimerShutter will be set to respect simulators current timeFactor
    float mFovY;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerShutter;

    double mTimeFactor;

    // We need to be careful with this, as the targets live in other threads
    Ogre::SceneNode *mCameraNode;
    Ogre::Camera *mCamera;
    Ogre::PixelBox *mPixelBox;
    OgreWidget *mOgreWidget;

    Ogre::RenderTarget *mRenderTarget;

private slots:
    void slotConnectToServer(void);
    void slotSendImage(void);
    void slotBytesWritten(qint64 bytes);
    void slotNetworkError(QAbstractSocket::SocketError socketError);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    Camera(Simulator* simulator, OgreWidget* ogreWidget, const QSize size, const float fovY, const int interval);

    ~Camera();

    Ogre::SceneNode* getSceneNode(void);

    // Getters for the properties
    Ogre::Vector3 getPosition(void) const;
    Ogre::Quaternion getOrientation(void) const;
    QSize imageSize() const;
    float fovY(void) const;
    int interval(void) const;

    // Setters for the properties
    void setPosition(const Ogre::Vector3 &position);
    void setOrientation(const Ogre::Quaternion &orientation);
    void setImageSize(const QSize size);
    void setFovY(const float fovY);
    void setInterval(const int interval);

//    void run(void);

public slots:
    void slotStart(void);
    void slotPause(void);
    void slotSetTimeFactor(double timeFactor);
//    void slotSetCameraPose(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation);
};

#endif
