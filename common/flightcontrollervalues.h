#ifndef FLIGHTCONTROLLERVALUES_H
#define FLIGHTCONTROLLERVALUES_H

#include "waypoint.h"
#include "motioncommand.h"
#include "pose.h"
#include "flightstate.h"
#include "pidcontroller.h"

// This class/struct just contains a set of values that flightcontroller consumes and produces.
// Instances of this class are sent around for debugging and to the kopter for actual control.

class FlightControllerValues
{
public:
    FlightControllerValues();

    qint32 timestamp;
    PidController controllerThrust, controllerYaw, controllerPitch, controllerRoll;
    MotionCommand motionCommand;
    FlightState flightState;
    QVector3D targetPosition;
    Pose lastKnownPose;
    float lastKnownHeightOverGround;
    QTime lastKnownHeightOverGroundTimestamp;
};

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightControllerValues &fcv);
QDataStream& operator>>(QDataStream &in, FlightControllerValues &fcv);


#endif
