#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QVector3D>

#include "simulator.h"
#include "joystick.h"
#include "vehicle.h"

class Simulator;
class Vehicle;

class FlightController : public QObject
{
    Q_OBJECT

private:
    Joystick *mJoystick;
    Vehicle* mVehicle;
    Simulator* mSimulator;
    BtOgre::RigidBodyState* mMotionState;
    Ogre::Vector3 mNextWayPoint;

    int mTimeOfLastUpdate;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;

protected:

public:
    FlightController(Simulator* simulator, Vehicle* vehicle, BtOgre::RigidBodyState* motionState);
    ~FlightController();

    void getEngineSpeeds(int &f, int &b, int &l, int &r);

private:
    void getJoystickValues(int &f, int &b, int &l, int &r);

public slots:
    void slotSetNextWayPoint(const Ogre::Vector3 &wayPoint);

    /*
signals:
    void newPose(const Ogre::Vector3 pos, const Ogre::Quaternion rot);

    void run(void);
    void start(void);
    void stop(void);
    void slotSetMotorSpeeds(const QList<int> &speeds);
    void slotShutDown(void);

private slots:
    void slotUpdatePosition(void);
    void slotUpdatePhysics(void);
    */
};

#endif
