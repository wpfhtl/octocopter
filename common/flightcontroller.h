#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QList>
#include <QVector3D>
#include <QTime>

#include <common.h>
#include "waypoint.h"
#include "laserscanner.h"
#include "pose.h"

/*
  The Flightcontroller is responsible for controlling the kopter's flight. Using either
  its own timer or a separate thread (to be determined), it emits its motion(..) signal
  perdiodically. These values are then supposed to be sent to the kopter.

  The Flightcontroller has a list of waypoints that can be manipulated using its API.
  When the list is empty, it will land (by appending another waypoint on the ground
  right below its current position) and then idle.

  The Flightcontroller simply approaches its next waypoint on the shortest path and does
  NOT do collision-avoidance - use slotFreeze() to halt in mid-air (=hover).
*/

class FlightController : public QObject
{
    Q_OBJECT

public:
    FlightController(LaserScanner* const);
    ~FlightController();

    enum FlightState
    {
        ManualControl,
        ApproachingNextWayPoint,
        Freezing,
        Idle
    };

    LaserScanner* mLaserScanner;


    FlightState getFlightState(void) const;
    QString getFlightStateString(void) const;

    QVector3D getPosition() const;
    QQuaternion getOrientation();
    QList<WayPoint> getWayPoints();
    void clearWayPoints();

    static int wayPointComponentsEqual(const QVector3D &wpt1, const QVector3D &wpt2);

private:
    QList<WayPoint> mWayPoints, mWayPointsPassed;

    FlightState mFlightState;

    QTime mTimeOfLastUpdate;

    quint32 mGpsTimeOfLastScan;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;

    Pose mLastKnownVehiclePose;

    // Upon reaching a waypoint, we rotate. This helps to keep track of that rotation
    float mDesiredYaw;

    void wayPointReached();
    WayPoint getLandingWayPoint() const;
    void setFlightState(const FlightState&);

signals:
    // used to set the motor-speeds
    void motion(const quint8& thrust, const qint8& nick, const qint8& roll, const qint8& yaw, const qint8& height);

    // emitted when a waypoint is reached
    void wayPointReached(const WayPoint&);

    // emitted when the flightState changed
    void flightStateChanged(FlightState);

    void currentWayPoints(QList<WayPoint>);

    // log/status messages
    void message(const QString&, const LogImportance& importance, const QString& message);

public slots:
    void slotSetVehiclePose(Pose*);
    void slotWayPointInsert(const QString &hash, const int index, const QVector3D &wayPoint);
    void slotWayPointDelete(const QString &hash, const int index);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Why does the FlightController give a &%$§ whether the laserscanner is active? While scanning,
    // we want to yaw constantly, so that the lidar can see more terrain (similar to creating
    // panorama photos). Unfortunately, this constant rotating confuses the human pilot, so we only
    // yaw when the lidar actually needs this.
    // If this slot is NOT called for a specified period of time (in the order of e.g. 500ms), we
    // stop rotating.
    void slotScanningInProgress(const quint32& timestamp);

    void slotFreeze();
};

#endif
