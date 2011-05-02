#ifndef ENGINE_H
#define ENGINE_H

#include <QObject>
#include <QMap>
#include <QString>
#include <QDebug>

#include "btBulletDynamicsCommon.h"

// probably not the right place to put this
#define deg2rad(x) (x)*M_PI/180.0
#define rad2deg(x) (x)*180.0/M_PI

// Read http://www.mikrokopter.de/ucwiki/FlugZeit#Flugzeit_.2BAPw-ber_Nutzlast

class Engine : public QObject
{
Q_OBJECT
private:
    struct Propeller
    {
        Propeller(const double p_c1, const double p_c2, const double p_c3, const int p_rpmMin, const int p_rpmMax, const float p_diameter, const float p_pitch)
            : c1(p_c1), c2(p_c2), c3(p_c3), rpmMin(p_rpmMin), rpmMax(p_rpmMax), diameter(inch2meter(p_diameter)), pitch(p_pitch) {}

        Propeller() // default c'tor for QMap
            : c1(0), c2(0), c3(0), rpmMin(0), rpmMax(0), diameter(0), pitch(0) {}

        float inch2meter(float inches) {return inches * 0.0254;}

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
//    Engine(const btTransform &pose = btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), QObject *parent = 0);

    // Construct an engine from which we just want thrust (scalar) and torque (also scalar?)
    Engine(void);
    Engine(const Engine &other);

    Engine& operator=(const Engine &other);

    btVector3 getPosition(void) const;

    // choose a propeller!
    bool setPropeller(const QString &propeller);

    // returns thrust in newton for a given rpm
//    btVector3 calculateThrust(const int rpm) const;
//    double calculateThrust(const int rpm) const;


    // This method was added later, upon realization that we don't care about RPM. Instead,
    // we use http://www.mikrocontroller.com/files/Schubdiagramm_Roxxy2827-35.jpg to correlate
    // thrust and current. This also makes the battery-simulation work better.
    // Returns a thrust force in newton for a given current flowing through a single motor.
    double calculateThrust(const float& current);

    // returns torque in Nm.
//    btVector3 calculateTorque(const int rpm) const;
    double calculateTorque(const int rpm) const;

signals:

public slots:

};

#endif
