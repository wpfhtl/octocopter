#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QList>
#include <QVector3D>
#include <QTime>

#include <common.h>
#include "flightcontrollervalues.h"
#include <laserscanner.h>
#include "pose.h"

class MotionCommand;

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
    FlightController(const QString& logFilePrefix = QString());
    ~FlightController();

    FlightState getFlightState(void) const;
    Pose getLastKnownPose(void) const;
    QList<WayPoint> getWayPoints();
    // ??? static int wayPointComponentsEqual(const QVector3D &wpt1, const QVector3D &wpt2);

private:
    QFile* mLogFile;

    // Motion is computed whenever a new pose comes in, so we don't need a timer - except
    // when the GPS board fails to deliver useful poses, we'll need to compute safe values
    // to emit. Thus, when Poses are planned to come in every 100ms, we start this timer
    // 150ms, and it will call slotComputeMotionCommands() regularly when GNSS board fails.
    static const quint16 backupTimerIntervalFast = 150;
    static const quint16 backupTimerIntervalSlow = 500;
    QTimer *mBackupTimerComputeMotion;

    QList<WayPoint> mWayPoints, mWayPointsPassed;

    struct ImuOffsets {
        float pitch;
        float roll;
    };

    ImuOffsets mImuOffsets;

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

    void logFlightControllerValues(const FlightControllerValues&);

    // In the first controller iteration, we don't want to build derivatives, they'd be waaayy off and destabilize the controller
    bool mFirstControllerRun;

    void ensureSafeFlightAfterWaypointsChanged();

private slots:
    void slotComputeBackupMotion();

signals:
    // used to set the motor-speeds
    void motion(const MotionCommand&);

    // used for debugging
    void flightControllerValues(const FlightControllerValues&);

    // emitted when a waypoint is reached
    void wayPointReached(const WayPoint&);

    // emitted when flightcontroller inserts a waypoint. Can be at the end (for landing) or in between
    // for collision avoidance etc.
    void wayPointInserted(quint16 index, const WayPoint& wayPoint);

    // emitted when the flightState changed
    void flightStateChanged(FlightState);

    void currentWayPoints(QList<WayPoint>);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);

public slots:
    void slotNewVehiclePose(const Pose&);
    void slotWayPointInsert(const quint16& index, const WayPoint& wayPoint);
    void slotWayPointDelete(const quint16& index);
    void slotSetWayPoints(const QList<WayPoint>&);

    void slotEmitFlightState();

    // The IMU is not mounted perfectly straight on the helicopter. Even if it was, the helicopter is a bent mess by now.
    // This means that reading IMU values and then setting the kopter's pitch/roll from there leads to big drifting. To
    // counter this, we fly manually and tune pitch/roll to have it balanced, then toggle SW4/PB8, calling this slot.
    // FlightController then saves the currently read IMU values as offsets. Easy, huh?
    void slotCalibrateImu();

    // This signal comes from Kopter (the MK's serial connection), and we use it only to derive the flightstate from the RemoteControl's flightstate-switch
    void slotFlightStateSwitchValueChanged(const FlightStateSwitch& fssv);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Called by LaserScanner to set the last known distance of the ray pointing down in vehicle frame
    void slotSetHeightOverGround(const float&);
};

#endif
