#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
//#include <QObject>
//#include <QTimer>
#include <Ogre.h>

#include "simulator.h"
#include "ogrewidget.h"
#include "coordinateconverter.h"

class Simulator;

class LaserScanner : public QObject
{
    Q_OBJECT

private:
    Simulator *mSimulator;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerScanStep;

    // the scanner's range in meters. This abstract the function between scan-target's size and range
    float mRange;

    // how fast does the laser rotate in rounds per minute? Storing a float makes following calculations easier.
    float mSpeed;

    // 0 deg is the rear, 180deg is the front. E.g. hokuyo utm30lx goes from 45 to 315 deg.
    int mAngleStart, mAngleStop;

    // members for visualizing the ray in ogre
    Ogre::ManualObject* mRayObject;
    Ogre::SceneNode* mRayNode;
    Ogre::MaterialPtr mRayMaterial;

    // how many degrees between two rays?
    float mAngleStep;

    // current angle/status of the current scan. Valid between mAngleStart and mAngleStop
    float mCurrentScanAngle;

    // a container for collected rays, or rather the world coordinates of where they ended
    QList<CoordinateGps> mScanData;

    float mTimeFactor;

    Ogre::RaySceneQuery *mRaySceneQuery;
    Ogre::SceneNode *mScannerNode;
    OgreWidget *mOgreWidget;

    CoordinateConverter mCoordinateConverter;

    void testRsqPerformance();

private slots:
    void slotDoScanStep(bool visualize = true, bool scheduleNextScan = true);

public:
    // Laser rotation is always CCW, angleStart < angleStop
    LaserScanner(
            Simulator* simulator,
            OgreWidget* ogreWidget,
            const float range,
            const int speed,
            //const RotationDirection direction,
            const int angleStart,
            const int angleStop,
            const float angleStep);

    ~LaserScanner();

    Ogre::SceneNode* getSceneNode(void);

    // Getters for the properties
    float range(void) const;
    int speed(void) const;
    int angleStart(void) const;
    int angleStop(void) const;
    float angleStep(void) const;
    Ogre::Vector3 getPosition(void);
    Ogre::Quaternion getOrientation(void);

    // Setters for the properties
    void setRange(float range);
    void setSpeed(int speed);
    void setAngleStart(int angleStart);
    void setAngleStop(int angleStop);
    void setAngleStep(float angleStep);
    void setPosition(const Ogre::Vector3 &position);
    void setOrientation(const Ogre::Quaternion &orientation);

    void setTimeFactor(float);

signals:
    void scanFinished(QList<CoordinateGps>);


public slots:
    void slotStart(void);
    void slotPause(void);
};

#endif
