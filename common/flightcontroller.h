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
  NOT do collision-avoidance - use slotFreeze() to halt in mid-air (=hover).
*/

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
        Freezing,
        Idle
    };

    FlightState getFlightState(void) const;
    QString getFlightStateString(void) const;

    Pose getLastKnownPose(void) const;
    QList<WayPoint> getWayPoints();
    void clearWayPoints();

    static int wayPointComponentsEqual(const QVector3D &wpt1, const QVector3D &wpt2);

private:
    QList<WayPoint> mWayPoints, mWayPointsPassed;

    FlightState mFlightState;

    QTime mTimeOfLastControllerUpdate, mTimeOfLastLaserScan;

    float mPrevErrorPitch, mPrevErrorRoll, mPrevErrorYaw, mPrevErrorHeight;
    float mErrorIntegralPitch, mErrorIntegralRoll, mErrorIntegralYaw, mErrorIntegralHeight;

    Pose mLastKnownVehiclePose;

    QTime mLastKnownBottomBeamLengthTimestamp;
    float mLastKnownBottomBeamLength;

    void wayPointReached();
    WayPoint getLandingWayPoint() const;
    void setFlightState(const FlightState&);

    // In the first controller iteration, we don't want to build derivatives, they'd be waaayy off and destabilize the controller
    bool mFirstControllerRun;

    // Just in case we reach an undefined state, we emit safe control values. Used instead of asserting.
    void emitSafeControlValues();

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

    // This signal comes from Kopter (the MK's serial connection), and we use it only to derive the flightstate from the RemoteControl's externalControl-switch
    void slotNewPpmChannelValues(const quint8 thrust, const qint8 yaw, const qint8 pitch, const qint8 roll, const bool motorSafety, const bool externalControl);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Called by LaserScanner to set the last known distance of the ray pointing down in vehicle frame
    void slotSetHeightOverGround(const float&);

    void slotFreeze();
};

#endif
