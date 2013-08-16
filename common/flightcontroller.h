#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <QList>
#include <QVector3D>
#include <QTime>

#include <common.h>
#include "gnsstime.h"
#include "logfile.h"
#include "flightstate.h"
#include "flightstaterestriction.h"
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
    LogFile* mLogFile;

    float mMaxFlightVelPerAxis;

    FlightControllerValues mFlightControllerValues;

    // Motion is computed whenever a new pose comes in, so we don't need a timer - except
    // when the GPS board fails to deliver useful poses, we'll need to compute safe values
    // to emit. Thus, when Poses are planned to come in every 100ms, we start this timer
    // 150ms, and it will call slotComputeMotionCommands() regularly when GNSS board fails.
    //
    // When used in the simulator, we want to pause this timer when the simulation is paused,
    // so this is what setPause(bool) is for - it is to be used ONLY in simulation.
    static const quint16 backupTimerIntervalFast = 180;
    static const quint16 backupTimerIntervalSlow = 500;
    QTimer *mBackupTimerComputeMotion;

    QList<WayPoint> mWayPoints, mWayPointsPassed;

    // Where is the vehicle's position, relative to the vehicle, split up in pitch and roll axis-components
    float getLateralOffsetOnVehiclePitchAxisToPosition(const QVector3D& vehiclePosition, const float vehicleYaw, const QVector3D &desiredPosition) const;
    float getLateralOffsetOnVehicleRollAxisToPosition(const QVector3D& vehiclePosition, const float vehicleYaw, const QVector3D &desiredPosition) const;

    // When the vehicle pitches and rolls strongly, we should increase thrust to keep the height constant. The height-
    // controller will do this automatically, but with a delay. To remove this delay, we amplify thrust based on
    // pitch and roll angles.
    void adjustThrustToPitchAndRoll(MotionCommand& mc);

    // When approaching waypoints, we move the hoverpoint along the trajectory (the carrot and mule thingy)
    // In case strong wind pushes the vehicle back, we don't want the hoverPoint to slide back on the trajectory.
    // For this reason, this variable records how far we've come, preventing the hoverpoint to slide back.
    float mTrajectoryProgress;

//    QVector3D getClosestPointOnTrajectory(const QVector3D& trajectoryStart, const QVector3D& trajectoryGoal, const QVector3D& point);
    QVector3D getHoverPosition(const QVector3D& trajectoryStart, const QVector3D& trajectoryGoal, const QVector3D& vehiclePosition, const float& desiredDistanceToHoverPosition);

    /*
    class ImuOffsets {
    private:
        static const quint8 numberOfMeasurementsToAverage = 10;
        quint8 mNumberOfMeasurementsAveraged;
        float mOffsetPitch, mOffsetRoll;
        float mMeasurementSumPitch, mMeasurementSumRoll;
        bool mDoCalibration;


    public:
        ImuOffsets() : mNumberOfMeasurementsAveraged(0), mOffsetPitch(0.0f), mOffsetRoll(0.0f), mMeasurementSumPitch(0.0f), mMeasurementSumRoll(0.0f), mDoCalibration(false) {}

        void applyCorrection(float& pitch, float& roll) const
        {
            pitch -= mOffsetPitch;
            roll -= mOffsetRoll;
        }

        void doCalibration() {mDoCalibration = true;}

        void calibrate(const Pose* const p)
        {
            Q_ASSERT(needsMoreMeasurements());
            const float measuredPitch = p->getPitchDegrees();
            const float measuredRoll = p->getRollDegrees();

            mMeasurementSumPitch += measuredPitch;
            mMeasurementSumRoll += measuredRoll;

            qDebug() << "ImuOffsets::calibrate(): added IMU offsets pitch" << measuredPitch << "and roll" << measuredRoll << "from pose of TOW" << p->timestamp << "to IMU offset computation.";

            mNumberOfMeasurementsAveraged++;

            if(mNumberOfMeasurementsAveraged == numberOfMeasurementsToAverage)
            {
                mOffsetPitch = mMeasurementSumPitch / numberOfMeasurementsToAverage;
                mOffsetRoll = mMeasurementSumRoll / numberOfMeasurementsToAverage;
                qDebug() << "ImuOffsets::calibrate(): calibrated IMU offsets to pitch" << mOffsetPitch << "and roll" << mOffsetRoll << "from" << mNumberOfMeasurementsAveraged << "poses";

                // reset, so that we could calibrate again.
                mMeasurementSumPitch = mMeasurementSumRoll = 0.0f;
                mNumberOfMeasurementsAveraged = 0;
                mDoCalibration = false;
            }
        }

        bool needsMoreMeasurements() const {return mDoCalibration && mNumberOfMeasurementsAveraged < numberOfMeasurementsToAverage;}
    };

    ImuOffsets mImuOffsets;*/

    // A structure mapping for each flightstate: controller => weights.
    QMap<
        FlightState::State,
        QMap<
            PidController*,
            QMap<
                QChar /*weightName*/,
                float /*weight*/>
        >
    > mFlightControllerWeights;

    void initializeControllers();

    //bool isHeightOverGroundValueRecent() const;

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

    // emitted when the flightState changed AND on request (e.g. when a new client connects)
    void flightState(const FlightState* const fs);

    // emitted when the fsr changed AND on request (e.g. when a new client connects)
    void flightStateRestriction(const FlightStateRestriction* const fsr);

    // The weights themselves are part of FlightControllerValues
    void flightControllerWeights();

    // Emitted when FLightController changes the list. Thats currently not implemented
    void wayPoints(const QList<WayPoint>* const wpt, WayPointListSource);

    // log/status messages
    void message(const LogImportance& importance, const QString&, const QString& message);

public slots:
    void slotSetPause(bool pause) {if(pause) mBackupTimerComputeMotion->stop(); else mBackupTimerComputeMotion->start();}
    void slotNewVehiclePose(const Pose *const);
    void slotSetWayPoints(const QList<WayPoint>&, const WayPointListSource source);

    void slotEmitFlightControllerInfo();

    void slotSetFlightSpeed(const float flightSpeed);

    void slotSetControllerWeights(const QString *const controllerName, const QMap<QChar, float> *const weights);

    // The IMU is not mounted perfectly straight on the helicopter. Even if it was, the helicopter is a bent mess by now.
    // This means that reading IMU values and then setting the kopter's pitch/roll from there leads to big drifting. To
    // counter this, we fly manually and tune pitch/roll to have it balanced, then toggle SW4/PB8, calling this slot.
    // FlightController then saves the currently read IMU values as offsets. Easy, huh?
    // 2013-04-24: we don't currently use the IMU values at all, as we control based on speeds.
    // As an experiment, we use this switch on the remote control to raise the hoverPosition by one meter. This way, we
    // can have the helicopter in UserControl on the ground, then switch to hover, making it hover on the ground. Then,
    // we call this slot to raise the hoverpos, causing the helicopter to start quite furiously :)
    void slotLiftHoverPosition();

    // This signal comes from Kopter (the MK's serial connection), and we use it only to derive the flightstate from the RemoteControl's flightstate-switch
    void slotFlightStateRestrictionChanged(const FlightStateRestriction *const fsr);

    // Called regularly by our parent, we compute the motion commands then and emit motion(...).
    void slotComputeMotionCommands();

    // Called by LaserScanner to set the last known distance of the ray pointing down in vehicle frame
    void slotSetHeightOverGround(const float&);
};

#endif
