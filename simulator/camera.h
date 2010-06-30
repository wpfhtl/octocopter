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

class Camera : public QThread
{
    Q_OBJECT

private:
    Simulator *mSimulator;

    QMutex mMutex;

    QUdpSocket* mUdpSocket;

    int mWidth, mHeight, mInterval;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerShutter;

    float mTimeFactor;

    // We need to be careful with this, as the targets live in other threads
//    Ogre::SceneNode *mCameraNode;
    Ogre::Camera *mCamera;
    OgreWidget *mOgreWidget;

    Ogre::RenderTarget *mRenderTarget;

private slots:
    void slotRecordImage(void);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    Camera(Simulator* simulator, OgreWidget* ogreWidget, const int width, const int height, const int interval);

    ~Camera();

    Ogre::SceneNode* getSceneNode(void);

    // Getters for the properties
    Ogre::Vector3 getPosition(void) const;
    Ogre::Vector3 getDirection(void) const;
    int width(void) const;
    int height(void) const;
    int interval(void) const;

    // Setters for the properties
    void setPosition(const Ogre::Vector3 &position);
    void setDirection(const Ogre::Vector3 &direction);
    void setWidth(const int width);
    void setHeight(const int height);
    void setInterval(const int interval);

    void setTimeFactor(float);
    void run(void);

public slots:
    void slotStart(void);
    void slotPause(void);
//    void slotSetCameraPose(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation);
};

#endif
