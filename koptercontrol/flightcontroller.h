#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QVector3D>
#include <QTime>

//#include "common.h"
#include "waypoint.h"

class FlightController : public QObject
{
    Q_OBJECT

public:
    FlightController();
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
    QList<QVector3D> mWayPoints, mWayPointsPassed;

    FlightState mFlightState;

    QTime mTimeOfLastUpdate;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;
    QVector3D mLastPosition;

    // Upon reaching a waypoint, we rotate. This helps to keep track of that rotation
    float mDesiredYaw;

    void wayPointReached();
    QVector3D getLandingWayPoint() const;

signals:
    // used to set the motor-speeds
//    void speeds(const quint16& m1, const quint16& m2, const quint16& m3, const quint16& m4, const quint16& m5, const quint16& m6, const quint16& m7, const quint16& m8);
    void motion(const quint8& thrust, const qint8& nick, const qint8& roll, const qint8& yaw, const qint8& height);

    // emitted when a waypoint is reached
    void wayPointReached(const WayPoint&);

    void currentWayPoints(QVector<WayPoint>);

    // log/status messages
    void message(const QString);

public slots:
    void slotWayPointInsert(const QString &hash, const int index, const QVector3D &wayPoint);
    void slotWayPointDelete(const QString &hash, const int index);
};

#endif
