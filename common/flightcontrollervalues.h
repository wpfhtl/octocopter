#ifndef FLIGHTCONTROLLERVALUES_H
#define FLIGHTCONTROLLERVALUES_H

#include "waypoint.h"
#include "motioncommand.h"
#include "pose.h"

// This class/struct just contains a set of values that flightcontroller consumes and produces.
// Instances of this class are sent around for debugging and to the kopter for actual control.

struct FlightControllerValues
{
    MotionCommand motionCommand;
    WayPoint nextWayPoint;
    Pose lastKnownPose;
};

#endif
