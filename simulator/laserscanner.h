#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <QtCore>
#include <QtNetwork>
#include <Ogre.h>
#include <OgreTerrainGroup.h>

#include "simulator.h"
#include "ogrewidget.h"
#include "coordinateconverter.h"

class Simulator;
class OgreWidget;

class LaserScanner : public QThread
{
    Q_OBJECT

private:
    Simulator *mSimulator;

    QMutex mMutex;

    QUdpSocket* mUdpSocket;

    // We need singleShot functionality, but we also need to be able to pause when simulation is paused.
    QTimer* mTimerScan;

    // the scanner's range in meters. This abstract the function between scan-target's size and range
    float mRange, mRangeSquared;

    // how fast does the laser rotate in rounds per minute? Storing a float makes following calculations easier.
    float mSpeed;

    // 0 deg is the rear, 180deg is the front. E.g. hokuyo utm30lx goes from 45 to 315 deg.
    int mAngleStart, mAngleStop;

    // how many degrees between two rays?
    float mAngleStep;

    // current angle/status of the current scan. Valid between mAngleStart and mAngleStop
    float mCurrentScanAngle;

    std::string mMaterialName;

    // a container for collected rays, or rather the world coordinates of where they ended
    QList<QVector3D> mScanData;

    double mTimeFactor;

    // We need to be careful with this, as the targets live in other threads
//    Ogre::RaySceneQuery *mRaySceneQuery;
    Ogre::SceneNode *mScannerNode;
    OgreWidget *mOgreWidget;

    // This scanner's beam, used for the RSQ
    Ogre::Ray mLaserBeam;

//    CoordinateConverter *mCoordinateConverter;

    // Cache the scanner position. If it hasn't changed, there's no need to scan again.
    Ogre::Vector3 mScannerPosition, mScannerPositionPrevious;
    Ogre::Quaternion mScannerOrientation, mScannerOrientationPrevious;

    void testRsqPerformance();

    // unused for now
    bool queryResult(Ogre::SceneQuery::WorldFragment *fragment, Ogre::Real distance);
    bool queryResult(Ogre::MovableObject *obj, Ogre::Real distance);

    // http://www.ogre3d.org/wiki/index.php/Raycasting_to_the_polygon_level
    bool raycastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 &result);

    void getMeshInformation(
            const Ogre::MeshPtr mesh,
            size_t &vertex_count,
            Ogre::Vector3* &vertices,
            size_t &index_count,
            unsigned long* &indices,
            const Ogre::Vector3 &position,
            const Ogre::Quaternion &orient,
            const Ogre::Vector3 &scale);

private slots:
    void slotDoScan(void);

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

    Ogre::Ray getCurrentLaserBeam(void);

    // Members for visualizing the ray in ogre, used directly by ogrewidget
    Ogre::ManualObject* mRayObject;
    Ogre::SceneNode* mRayNode;
    Ogre::MaterialPtr mRayMaterial;

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

    void run(void);

public slots:
    void slotStart(void);
    void slotPause(void);
    void slotSetTimeFactor(double);
    void slotSetScannerPose(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation);
};

#endif
