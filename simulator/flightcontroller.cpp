#include "flightcontroller.h"

FlightController::FlightController(Simulator* simulator, Vehicle* vehicle, BtOgre::RigidBodyState* motionState) : QObject(simulator)
{
    mSimulator = simulator;
    mVehicle = vehicle;
    mMotionState = motionState;

    mNextWayPoint = Ogre::Vector3(0, 0, 0);
    mNextWayPoint = Ogre::Vector3(164, 10, 115);

    // create joystick
    mJoystick = new Joystick();

    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    mTimeOfLastUpdate = mSimulator->getSimulationTime()-1000; // milliseconds
}

FlightController::~FlightController()
{
    mJoystick->deleteLater();
}

void FlightController::getEngineSpeeds(int &f, int &b, int &l, int &r)
{
    const int simulationTime = mSimulator->getSimulationTime(); // milliseconds
    const double timeDiff = std::max(0.0f, (simulationTime - mTimeOfLastUpdate) / 1000.0f); // elapsed time since last call in seconds

    if(mJoystick->isButtonPressed(0) || mJoystick->isButtonPressed(1))
    {
        qDebug() << "FlightController::getEngineSpeeds(): joystick";
        getJoystickValues(f, b, l, r);
        mPrevErrorPitch = 0 - mMotionState->getOrientation().getPitch(false).valueDegrees();
        mPrevErrorRoll = 0 - mMotionState->getOrientation().getRoll(false).valueDegrees();
    }
    else
    {
//        qDebug() << "FlightController::getEngineSpeeds(): autopilot";

        // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!
        static double Kp = 5.1;
        static double Ki = 0.2;
        static double Kd = 0.6;

        double errorYaw = 0 - mMotionState->getOrientation().getYaw(false).valueDegrees();
        mErrorIntegralYaw += errorYaw*timeDiff;
        double derivativeYaw = (errorYaw - mPrevErrorYaw + 0.00001)/timeDiff;
        double outputYaw = (Kp*errorYaw) + (Ki*mErrorIntegralYaw) + (Kd*derivativeYaw);

        // adjust pitch/roll to reach target
        double desiredRoll, desiredPitch = 0.0;
        if(fabs(errorYaw) < 5.0)
        {
            if(mMotionState->getPosition().x > mNextWayPoint.x)
            {
                desiredRoll = fmin((mMotionState->getPosition().x - mNextWayPoint.x)*2, 25.0);
            }
            else
            {
                desiredRoll = -fmin((mNextWayPoint.x - mMotionState->getPosition().x)*2, 25.0);
            }
            if(mMotionState->getPosition().z > mNextWayPoint.z)
            {
                desiredPitch = -fmin((mMotionState->getPosition().z - mNextWayPoint.z)*2, 25.0);
            }
            else
            {
                desiredPitch = fmin((mNextWayPoint.z - mMotionState->getPosition().z)*2, 25.0);
            }
        }

        // base speed, keeps it in the air
        f = b = l = r = 9260;

        // try to get ourselves straight up
        double errorPitch = desiredPitch - mMotionState->getOrientation().getPitch(false).valueDegrees();
        mErrorIntegralPitch += errorPitch*timeDiff;
        double derivativePitch = (errorPitch - mPrevErrorPitch + 0.00001)/timeDiff;
        double outputPitch = (Kp*errorPitch) + (Ki*mErrorIntegralPitch) + (Kd*derivativePitch);

        double errorRoll = desiredRoll - mMotionState->getOrientation().getRoll(false).valueDegrees();
        mErrorIntegralRoll += errorRoll*timeDiff;
        double derivativeRoll = (errorRoll - mPrevErrorRoll + 0.00001)/timeDiff;
        double outputRoll = (Kp*errorRoll) + (Ki*mErrorIntegralRoll) + (Kd*derivativeRoll);

        double errorHeight = mNextWayPoint.y - mVehicle->getHeightAboveGround();
        mErrorIntegralHeight += errorHeight*timeDiff;
        double derivativeHeight = (errorHeight - mPrevErrorHeight + 0.00001)/timeDiff;
        double outputHeight = (Kp*errorHeight) + (Ki*mErrorIntegralHeight) + (Kd*derivativeHeight);

//        qDebug() << "error\t" << errorPitch << "mPrevError" << mPrevErrorPitch << "\tmErrorIntegral" << mErrorIntegralPitch << "\tderivative" << derivativePitch << "\toutput" << outputPitch;

//        qDebug() << errorPitch << (Kp*errorPitch) << (Ki*mErrorIntegralPitch) << (Kd*derivativePitch);

        mPrevErrorPitch = errorPitch;
        mPrevErrorRoll = errorRoll;
        mPrevErrorYaw = errorYaw;
        mPrevErrorHeight = errorHeight;

        // Pitch correction: When stick goes forward, joyY goes up => b- f+
        b -= outputPitch * 10;
        f += outputPitch * 10;

        // Roll correction: When stick goes right, joyX goes up => l+ r-
        l -= outputRoll * 10;
        r += outputRoll * 10;

        // Yaw correction: When stick is yawed...
        f += outputYaw * 10;
        b += outputYaw * 10;
        l -= outputYaw * 10;
        r -= outputYaw * 10;

        // if roll and pitch is as desired, activate height controller
        if(fabs(errorPitch) + fabs(errorRoll) < 5.0)
        {
            f += outputHeight * 100;
            b += outputHeight * 100;
            l += outputHeight * 100;
            r += outputHeight * 100;
        }

        // cap at max speeds.
        static const float max = 40000;
        if(f>max) f=max; if(f<-max) f=-max;
        if(b>max) b=max; if(b<-max) b=-max;
        if(l>max) l=max; if(l<-max) l=-max;
        if(r>max) r=max; if(r<-max) r=-max;


//        Ogre::Vector3 vectorToTarget = mNextWayPoint - mMotionState->getPosition();
//        qDebug() << "FlightController::getEngineSpeeds(): to target:" << vectorToTarget.length();
    }

    mTimeOfLastUpdate = simulationTime;

//    qDebug() << "FlightController::getEngineSpeeds():" << "f" << f << "b" << b << "l" << l << "r" << r;
}

void FlightController::getJoystickValues(int &f, int &b, int &l, int &r)
{
    float joyX, joyY, joyZ, joyR;
    mJoystick->getAxisValues(joyX, joyY, joyZ, joyR);

    // Set motor-base-speed according to thrust between 0 and 40000
    /*int */f = b = l = r = (-joyR + 0.5) * 20000;
//    qDebug() << "joyVals\t\t" << joyX << joyY << joyZ << joyR;
//    qDebug() << "baseSpeed\t" << f << b << l << r;

    // Steering sensitivity is higher in slow motion
    const int maxSteeringPitchRoll = 500 / mSimulator->getTimeFactor();
    const int maxSteeringYaw = 150 / mSimulator->getTimeFactor();

    // When stick goes right, joyX goes up => l+ r-
    l += (maxSteeringPitchRoll * joyX);
    r -= (maxSteeringPitchRoll * joyX);

    // When stick goes forward, joyY goes up => b- f+
    b -= (maxSteeringPitchRoll * joyY);
    f += (maxSteeringPitchRoll * joyY);

    // When stick is yawed...
    f += (maxSteeringYaw * -joyZ);
    b += (maxSteeringYaw * -joyZ);
    l -= (maxSteeringYaw * -joyZ);
    r -= (maxSteeringYaw * -joyZ);

//    qDebug() << "FlightController::getJoystickValues():\t" << "f" << f << "b" << b << "l" << l << "r" << r;
}

void FlightController::slotSetNextWayPoint(const Ogre::Vector3 &wayPoint)
{
    mNextWayPoint = wayPoint;
}
