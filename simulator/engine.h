#ifndef ENGINE_H
#define ENGINE_H

#include <QObject>
#include <QMap>
#include <QString>

#include "btBulletDynamicsCommon.h"

class Engine : public QObject
{
Q_OBJECT
private:
    struct Propeller
    {
        Propeller(const double p_c1, const double p_c2, const double p_c3, const int p_rpmMin, const int p_rpmMax, const float p_diameter, const float p_pitch)
            : c1(p_c1), c2(p_c2), c3(p_c3), rpmMin(p_rpmMin), rpmMax(p_rpmMax), diameter(p_diameter), pitch(p_pitch) {}
        Propeller() // default c'tor for QMap
            : c1(0), c2(0), c3(0), rpmMin(0), rpmMax(0), diameter(0), pitch(0) {}
        double c1, c2, c3;
        int rpmMin, rpmMax;
        float diameter, pitch;
    };

    Propeller mCurrentPropeller;
    QMap<QString, Propeller> mPropellers;
    void initializePropellers();

    btTransform mPose;
//    RotationDirection mRotationDirection;

public:
//    enum RotationDirection
//    {
//        CW = -1,
//        CCW = 1
//    };

    // Creates an engine with an offset and a rotation. With this, we can
    // calculate both thrust and torque caused on the mount (=vehicle)
    Engine(const btTransform &pose = btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), QObject *parent = 0);

    // choose a propeller!
    bool setPropeller(const QString &propeller);

    // returns thrust in newton for a given rpm
    btVector3 calculateThrust(const int rpm);

    // returns torque in Nm.
    btVector3 calculateTorque(const int rpm);

signals:

public slots:

};

#endif
