#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QList>
#include <QVector3D>
#include <QTime>

#include <math.h> // ceil and floor

#include <common.h>
#include "waypoint.h"
#include <laserscanner.h>
#include "pose.h"

/*
  The Flightcontroller is responsible for controlling the kopter's flight. Using either
  its own timer or a separate thread (to be determined), it emits its motion(..) signal
  perdiodically. These values are then supposed to be sent to the kopter.

  The Flightcontroller has a list of waypoints that can be manipulated using its API.
  When the list is empty, it will land (by appending another waypoint on the ground
  right below its current position) and then idle.

  The Flightcontroller simply approaches its next waypoint on the shortest path and does
  NOT do collision-avoidance - use slotHoldPosition() to halt in mid-air (=hover).
*/

class FlightController : public QObject
{
    Q_OBJECT

public:
    FlightController();
    ~FlightController();

    FlightState getFlightState(void) const;
    Pose getLastKnownPose(void) const;
    QList<WayPoint> getWayPoints();
    // ??? static int wayPointComponentsEqual(const QVector3D &wpt1, const QVector3D &wpt2);

private:
    // Motion is computed whenever a new pose comes in, so we don't need a timer - except
    // when the GPS board fails to deliver (even useless) poses, we'll need to compute
    // safe values to emit. Thus, when Poses come in every 40ms, we start this timer with
    // 40ms+20%, and it will call slotComputeMotionCommands() at an interval of 48ms when
    // the GPS board fails.
    QTimer *mBackupTimerComputeMotion;
    QList<WayPoint> mWayPoints, mWayPointsPassed;

    FlightState mFlightState;

    QTime mTimeOfLastControllerUpdate, mTimeOfLastLaserScan;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;

    Pose mLastKnownVehiclePose;

    QTime mLastKnownHeightOverGroundTimestamp;
    float mLastKnownHeightOverGround;

    bool isHeightOverGroundValueRecent() const;

    void nextWayPointReached();
//    WayPoint getLandingWayPoint() const;
    void setFlightState(FlightState);

    // In the first controller iteration, we don't want to build derivatives, they'd be waaayy off and destabilize the controller
    bool mFirstControllerRun;

    // Just in case we reach an undefined state, we emit safe control values. Used instead of asserting.
    void emitSafeControlValues();

    void ensureSafeFlightAfterWaypointsChanged();

private slots:
    void slotComputeBackupMotion();

signals:
    // used to set the motor-speeds
    void motion(const quint8& thrust, const qint8& yaw, const qint8& pitch, const qint8& roll, const qint8& height);

    void debugValues(
        const Pose& usedPose,
        const quint8& thrust,
        const qint8& yaw,
        const qint8& pitch,
        const qint8& roll,
        const qint8& height);

    // emitted when a waypoint is reached
    void wayPointReached(const WayPoint&);

    // emitted when flightcontroller inserts a waypoint. Can be at the end (for landing) or in between
    // for collision avoidance etc.
    void wayPointInserted(quint16 index, const WayPoint& wayPoint);

    // emitted when the flightState changed
    void flightStateChanged(FlightState);

    void currentWayPoints(QList<WayPoint>);

    // log/status messages
    void message(const QString&, const LogImportance& importance, const QString& message);

public slots:
    void slotNewVehiclePose(const Pose&);
    void slotWayPointInsert(const quint16& index, const WayPoint& wayPoint);
    void slotWayPointDelete(const quint16& index);
    void slotSetWayPoints(const QList<WayPoint>&);

    void slotEmitFlightState();

    // This signal comes from Kopter (the MK's serial connection), and we use it only to derive the flightstate from the RemoteControl's externalControl-switch
    void slotExternalControlStatusChanged(bool externalControl);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Called by LaserScanner to set the last known distance of the ray pointing down in vehicle frame
    void slotSetHeightOverGround(const float&);

    void slotHoldPosition();
};

#endif
