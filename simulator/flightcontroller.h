#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QVector3D>

//#include <qwt_plot.h>
//#include <qwt_plot_marker.h>
//#include <qwt_plot_curve.h>
//#include <qwt_legend.h>
//#include <qwt_data.h>
//#include <qwt_text.h>
//#include <qwt_math.h>

#include "simulator.h"
#include "joystick.h"
#include "vehicle.h"
#include "common.h"

class Simulator;
class Vehicle;

class FlightController : public QObject
{
    Q_OBJECT

public:
    FlightController(Simulator* simulator, Vehicle* vehicle, BtOgre::RigidBodyState* motionState);
    ~FlightController();

    enum FlightState
    {
        ManualControl,
        ApproachingNextWayPoint,
//        Landing, // Landing is like Approaching a wpt on the ground.
        Idle
    };

    void getEngineSpeeds(int &f, int &b, int &l, int &r);

    FlightState getFlightState(void) const;
    QString getFlightStateString(void) const;

    QVector3D getPosition() const;
    QQuaternion getOrientation();
    QList<QVector3D> getWayPoints();
    void clearWayPoints();

    static int wayPointComponentsEqual(const QVector3D &wpt1, const QVector3D &wpt2);

private:
    Joystick *mJoystick;
    Vehicle* mVehicle;
    Simulator* mSimulator;
    BtOgre::RigidBodyState* mMotionState;
    QList<QVector3D> mWayPoints, mWayPointsPassed;

    FlightState mFlightState;

//    QwtPlot* mPlot;
//    QwtPlotCurve *curvePitch, *curveRoll, *curveYaw;

    int mTimeOfLastUpdate;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;
    QVector3D mLastPosition;

    // Upon reaching a waypoint, we rotate. This helps to keep track of that rotation
    float mDesiredYaw;

    void wayPointReached();
    QVector3D getLandingWayPoint() const;

signals:
        void wayPointReached(QVector3D);
        void currentWayPoints(QList<QVector3D>);
        void message(const QString);

private:
    void getJoystickValues(int &f, int &b, int &l, int &r);

private slots:
    void slotJoystickButtonStateChanged(unsigned char, bool);

public slots:
    void slotWayPointInsert(const QString &hash, const int index, const QVector3D &wayPoint);
    void slotWayPointDelete(const QString &hash, const int index);
};

#endif
