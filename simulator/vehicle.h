#ifndef VEHICLE_H
#define VEHICLE_H

#include <QThread>
#include <QTimer>
#include <Ogre.h>

#include "simulator.h"
#include "coordinateconverter.h"

class Simulator;

class Vehicle : public QThread
{
    Q_OBJECT
private:
    QTimer *mTimerUpdatePosition;
    int mTimeOfLastUpdate;
    Simulator *mSimulator;
    CoordinateConverter mCoordinateConverter;
    Ogre::Vector3 mNextWayPoint;
    Ogre::SceneNode *mVehicle;
//    bool mShuttingDown;


public:
    Vehicle(Simulator *simulator, Ogre::SceneNode *vehicle);
    void run(void);

signals:

public slots:
    void slotSetNextWayPoint(const CoordinateGps &wayPoint);
    void slotShutDown(void);

private slots:
    void slotUpdatePosition(void);
};

#endif
