#ifndef FLIGHTCONTROLLERVALUES_H
#define FLIGHTCONTROLLERVALUES_H

#include "waypoint.h"
#include "motioncommand.h"
#include "pose.h"
#include "flightstate.h"
#include "flightstaterestriction.h"
#include "pidcontroller.h"

// This class/struct just contains a set of values that flightcontroller consumes and produces.
// Instances of this class are sent around for debugging and to the kopter for actual control.

class FlightControllerValues
{
public:
    FlightControllerValues();

    QString toString() const;

    qint32 timestamp; // Is set by flightcontroller after all values are computed
    PidController controllerThrust, controllerYaw, controllerPitch, controllerRoll;
    MotionCommand motionCommand;
    FlightState flightState;
    FlightStateRestriction flightStateRestriction;
    QVector3D trajectoryStart;
    QVector3D trajectoryGoal;
    QVector3D hoverPosition;
    Pose lastKnownPose;
    float lastKnownHeightOverGround;
    qint32 lastKnownHeightOverGroundTimestamp;
};

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv);
QDataStream& operator>>(QDataStream &in, FlightControllerValues &fcv);

#endif
