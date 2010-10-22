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
    QList<QVector3D> mWayPoints;

//    QwtPlot* mPlot;
//    QwtPlotCurve *curvePitch, *curveRoll, *curveYaw;

    bool mAutoPilot;
    int mTimeOfLastUpdate;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;

    // Upon reaching a waypoint, we rotate. This helps to keep track of that rotation
    float mDesiredYaw;

protected:

public:
    FlightController(Simulator* simulator, Vehicle* vehicle, BtOgre::RigidBodyState* motionState);
    ~FlightController();

    void getEngineSpeeds(int &f, int &b, int &l, int &r);

    QVector3D getPosition();
    QQuaternion getOrientation();
    QList<QVector3D> getWayPoints();
    void clearWayPoints();

signals:
        void wayPointReached(QVector3D);
        void message(const QString);

private:
    void getJoystickValues(int &f, int &b, int &l, int &r);

private slots:
    void slotJoystickButtonStateChanged(unsigned char, bool);

public slots:
    void slotSetNextWayPoint(const QVector3D &wayPoint);
};

#endif
