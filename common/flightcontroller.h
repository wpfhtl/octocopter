#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QList>
#include <QVector3D>
#include <QTime>

#include <common.h>
#include "gnsstime.h"
#include "flightstate.h"
#include "flightstateswitch.h"
#include "flightcontrollervalues.h"
#include <laserscanner.h>
#include "pose.h"
#include "pidcontroller.h"

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

    const Pose* const getLastKnownPose(void) const { return &mFlightControllerValues.lastKnownPose; }

    QList<WayPoint> getWayPoints() { return mWayPoints; }

    const FlightState& getFlightState(void) const { return mFlightControllerValues.flightState; }

private:
    QFile* mLogFile;

    FlightControllerValues mFlightControllerValues;

    // When approaching waypoints, we first yaw until we point towards the target. As soon as
    // that's done, we activate the roll to keep the vehicle on the virtual line between vehicle
    // and target. If we didn't do this, wind would make the kopter drift, causing it to constantly
    // re-yaw to point to the target, making the vehicle circle around the target. See e.g.
    // kopterlog-20120813-133812-1621-bernd1 for an example of this behaviour.
    enum struct ApproachPhase
    {
        OrientTowardsTarget,
        ApproachTarget
    };

    ApproachPhase mApproachPhase;

    // Motion is computed whenever a new pose comes in, so we don't need a timer - except
    // when the GPS board fails to deliver useful poses, we'll need to compute safe values
    // to emit. Thus, when Poses are planned to come in every 100ms, we start this timer
    // 150ms, and it will call slotComputeMotionCommands() regularly when GNSS board fails.
    //
    // When used in the simulator, we want to pause this timer when the simulation is paused,
    // so this is what setPause(bool) is for - it is to be used ONLY in simulation.
    static const quint16 backupTimerIntervalFast = 150;
    static const quint16 backupTimerIntervalSlow = 500;
    QTimer *mBackupTimerComputeMotion;

    QList<WayPoint> mWayPoints, mWayPointsPassed;

    // The following method mixes the new controller outpuit with the last one to achieve smoothing
    // This is an attempt to smooth out GNSS reception problems in single GNSS-packets that would
    // otherwise lead to a bouncing vehicle while flip-flopping between safe control values and
    // hover/approachwaypoint.
    MotionCommand mLastMotionCommand;
    void smoothenControllerOutput(MotionCommand& mc);

    struct ImuOffsets {
        float pitch;
        float roll;
    };

    ImuOffsets mImuOffsets;

    void initializeControllers();

    bool isHeightOverGroundValueRecent() const;

    void nextWayPointReached();
    void setFlightState(FlightState);

    void logFlightControllerValues();

    void ensureSafeFlightAfterWaypointsChanged();

private slots:
    void slotComputeBackupMotion();

signals:
    // used to set the motor-speeds
    void motion(const MotionCommand* const mc);

    // used for debugging
    void flightControllerValues(const FlightControllerValues* const);

    // emitted when a waypoint is reached
    void wayPointReached(const WayPoint&);

    // emitted when flightcontroller inserts a waypoint. Can be at the end (for landing) or in between
    // for collision avoidance etc.
    void wayPointInserted(quint16 index, const WayPoint& wayPoint);

    // emitted when the flightState changed
    void flightStateChanged(const FlightState* const fs);

    // The weights themselves are part of FlightControllerValues
    void flightControllerWeightsChanged();

    void currentWayPoints(const QList<WayPoint>* const wpt);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);

public slots:
    void slotSetPause(bool pause) {if(pause) mBackupTimerComputeMotion->stop(); else mBackupTimerComputeMotion->start();}
    void slotNewVehiclePose(const Pose *const);
    void slotWayPointInsert(const quint16& index, const WayPoint& wayPoint);
    void slotWayPointDelete(const quint16& index);
    void slotSetWayPoints(const QList<WayPoint>&);

    void slotEmitFlightControllerInfo();

    void slotSetControllerWeights(const QString *const controllerName, const QMap<QString, float> *const controllerWeights);

    // The IMU is not mounted perfectly straight on the helicopter. Even if it was, the helicopter is a bent mess by now.
    // This means that reading IMU values and then setting the kopter's pitch/roll from there leads to big drifting. To
    // counter this, we fly manually and tune pitch/roll to have it balanced, then toggle SW4/PB8, calling this slot.
    // FlightController then saves the currently read IMU values as offsets. Easy, huh?
    void slotCalibrateImu();

    // This signal comes from Kopter (the MK's serial connection), and we use it only to derive the flightstate from the RemoteControl's flightstate-switch
    void slotFlightStateSwitchValueChanged(const FlightStateSwitch *const fssv);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Called by LaserScanner to set the last known distance of the ray pointing down in vehicle frame
    void slotSetHeightOverGround(const float&);
};

#endif
