#include "flightcontroller.h"

FlightController::FlightController(Simulator* simulator, Vehicle* vehicle, BtOgre::RigidBodyState* motionState) : QObject(simulator)
{
    mSimulator = simulator;
    mVehicle = vehicle;
    mMotionState = motionState;

    mAutoPilot = false;

//    mWayPoints.append(QVector3D(0, 0, 0));
    mWayPoints.append(QVector3D(160, 85, 115));

    // create joystick
    mJoystick = new Joystick();
    connect(mJoystick, SIGNAL(buttonStateChanged(unsigned char,bool)), SLOT(slotJoystickButtonStateChanged(unsigned char, bool)));

    mPrevErrorPitch = mPrevErrorRoll = mPrevErrorYaw = mPrevErrorHeight = 0.1;
    mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

    connect(this, SIGNAL(message(QString)), mSimulator, SLOT(slotShowMessage(QString)));

    mDesiredYaw = 180.0;

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

    if(!mAutoPilot)
    {
        qDebug() << "FlightController::getEngineSpeeds(): joystick";
        getJoystickValues(f, b, l, r);
        mPrevErrorPitch = 0 - mMotionState->getOrientation().getPitch(false).valueDegrees();
        mPrevErrorRoll = 0 - mMotionState->getOrientation().getRoll(false).valueDegrees();

        // clear the error integrals, so we don't get spikes after releasing the joystick
        mErrorIntegralPitch = mErrorIntegralRoll = mErrorIntegralYaw = mErrorIntegralHeight = 0.1;

        qDebug() << "values PRY:\t"
                 << mMotionState->getNode()->_getDerivedOrientation().getPitch(true).valueDegrees()
                 << mMotionState->getNode()->_getDerivedOrientation().getRoll(true).valueDegrees()
                 << mMotionState->getNode()->_getDerivedOrientation().getYaw(true).valueDegrees() + 180.0;

//        qDebug() << "values PRYH:" << mMotionState->getOrientation().getPitch(true).valueDegrees() << mMotionState->getOrientation().getRoll(true).valueDegrees() << mMotionState->getOrientation().getYaw(false).valueDegrees() << mMotionState->getPosition().y;
    }
    else
    {
//        qDebug() << "FlightController::getEngineSpeeds(): autopilot";

        QVector3D nextWayPoint;

        if(mWayPoints.size() > 0)
            nextWayPoint = mWayPoints.first();
        else
            nextWayPoint = getPosition() - QVector3D(0.0, mVehicle->getHeightAboveGround(), 0.0);

        // http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!
        static double Kp = 5.1;
        static double Ki = 1.2;
        static double Kd = 0.6;

        const float currentPitch = mMotionState->getOrientation().getPitch(true).valueDegrees();
        const float currentRoll = mMotionState->getOrientation().getRoll(true).valueDegrees();
        const float currentYaw = 180.0 + mMotionState->getNode()->_getDerivedOrientation().getPitch(true).valueDegrees();
        const float currentHeight = mMotionState->getPosition().y;//mVehicle->getHeightAboveGround();

        double errorYaw = mDesiredYaw - currentYaw;
        mErrorIntegralYaw += errorYaw*timeDiff;
        double derivativeYaw = (errorYaw - mPrevErrorYaw + 0.00001)/timeDiff;
        double outputYaw = (Kp*errorYaw) + (Ki*mErrorIntegralYaw) + (Kd*derivativeYaw);

        // adjust pitch/roll to reach target
        double desiredRoll, desiredPitch = 0.0;
        if(fabs(errorYaw) < 5.0)
        {
            if(mMotionState->getPosition().x > nextWayPoint.x())
            {
                desiredRoll = fmin((mMotionState->getPosition().x - nextWayPoint.x())*2, 25.0);
            }
            else
            {
                desiredRoll = -fmin((nextWayPoint.x() - mMotionState->getPosition().x)*2, 25.0);
            }
            if(mMotionState->getPosition().z > nextWayPoint.z())
            {
                desiredPitch = -fmin((mMotionState->getPosition().z - nextWayPoint.z())*2, 25.0);
            }
            else
            {
                desiredPitch = fmin((nextWayPoint.z() - mMotionState->getPosition().z)*2, 25.0);
            }
        }

        // base speed, keeps it in the air
        f = b = l = r = 9260;

        // try to get ourselves straight up
        double errorPitch = desiredPitch - currentPitch;
        mErrorIntegralPitch += errorPitch*timeDiff;
        double derivativePitch = (errorPitch - mPrevErrorPitch + 0.00001)/timeDiff;
        double outputPitch = (Kp*errorPitch) + (Ki*mErrorIntegralPitch) + (Kd*derivativePitch);

        double errorRoll = desiredRoll - currentRoll;
        mErrorIntegralRoll += errorRoll*timeDiff;
        double derivativeRoll = (errorRoll - mPrevErrorRoll + 0.00001)/timeDiff;
        double outputRoll = (Kp*errorRoll) + (Ki*mErrorIntegralRoll) + (Kd*derivativeRoll);

        double errorHeight = nextWayPoint.y() - currentHeight;
        mErrorIntegralHeight += errorHeight*timeDiff;
        double derivativeHeight = (errorHeight - mPrevErrorHeight + 0.00001)/timeDiff;
        double outputHeight = (5.0*errorHeight) + (1.2*mErrorIntegralHeight) + (0.6*derivativeHeight);

//        qDebug() << "error\t" << errorPitch << "mPrevError" << mPrevErrorPitch << "\tmErrorIntegral" << mErrorIntegralPitch << "\tderivative" << derivativePitch << "\toutput" << outputPitch;

        qDebug() << "derivt PRYH:" << QString::number(derivativePitch, 'f', 2) << "\t" << QString::number(derivativeRoll, 'f', 2) << "\t" << QString::number(derivativeYaw, 'f', 2) << "\t" << QString::number(derivativeHeight, 'f', 2);
        qDebug() << "prevEr PRYH:" << QString::number(mPrevErrorPitch, 'f', 2) << "\t" << QString::number(mPrevErrorRoll, 'f', 2) << "\t" << QString::number(mPrevErrorYaw, 'f', 2) << "\t" << QString::number(mPrevErrorHeight, 'f', 2);
        qDebug() << "inteEr PRYH:" << QString::number(mErrorIntegralPitch, 'f', 2) << "\t" << QString::number(mErrorIntegralRoll, 'f', 2) << "\t" << QString::number(mErrorIntegralYaw, 'f', 2) << "\t" << QString::number(mErrorIntegralHeight, 'f', 2);
        qDebug() << "values PRYH:" << QString::number(currentPitch, 'f', 2) << "\t" << QString::number(currentRoll, 'f', 2) << "\t" << QString::number(currentYaw, 'f', 2) << "\t" << QString::number(mMotionState->getPosition().y, 'f', 2);
        qDebug() << "should PRYH:" << QString::number(desiredPitch, 'f', 2) << "\t" << QString::number(desiredRoll, 'f', 2) << "\t" << QString::number(mDesiredYaw, 'f', 2) << "\t" << QString::number(nextWayPoint.y(), 'f', 2);
        qDebug() << "output PRYH:" << QString::number(outputPitch, 'f', 2) << "\t" << QString::number(outputRoll, 'f', 2) << "\t" << QString::number(outputYaw, 'f', 2) << "\t" << QString::number(outputHeight, 'f', 2);

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
        f -= outputYaw * 10;
        b -= outputYaw * 10;
        l += outputYaw * 10;
        r += outputYaw * 10;

        // if roll and pitch is as desired, activate height controller
        if(fabs(errorPitch) + fabs(errorRoll) < 5.0 || true)
        {
            f += outputHeight * 100;
            b += outputHeight * 100;
            l += outputHeight * 100;
            r += outputHeight * 100;
        }

        // cap at max speeds.
        static const float max = 30000;
        if(f>max) f=max; if(f<-max) f=-max;
        if(b>max) b=max; if(b<-max) b=-max;
        if(l>max) l=max; if(l<-max) l=-max;
        if(r>max) r=max; if(r<-max) r=-max;
/*
        if(mWayPoints.size()) // if its empty, this waypoint is just a parking position - don't rotate.
        {
            // If we have reached the wayPoint, rotate. When done, fly on.
            if(
                    getPosition().distanceToLine(nextWayPoint, QVector3D()) < 0.10
                    &&
                    errorPitch + errorRoll + 10*errorHeight < 10
                    )
            {
                qDebug() << "rotation" << mMotionState->getOrientation().getYaw(false).valueDegrees() << mDesiredYaw;
                mDesiredYaw += timeDiff * 10.0; // turn 10 degrees/sec

                if(mDesiredYaw > 85.0)
                {
                    if(mWayPoints.size())
                    {
                        QVector3D wpt = mWayPoints.takeFirst();
                        emit wayPointReached(wpt);
                    }
                    mDesiredYaw = 0.0;
                }
            }
        }
        else
        {
            // The waypointlist is empty, so we must be landing. If we're close to the ground,
            // switch off the engines.
            if(getPosition().distanceToLine(nextWayPoint, QVector3D()) < 0.20)
            {
                f=b=l=r= (f+b+l+r)/4.1;
            }
        }
*/
    }

    mTimeOfLastUpdate = simulationTime;

    qDebug() << "engine spds:" << "f" << f << "\tb" << b << "\tl" << l << "\tr" << r << endl;
}

void FlightController::clearWayPoints()
{
    mWayPoints.clear();
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

void FlightController::slotSetNextWayPoint(const QVector3D &wayPoint)
{
    qDebug() << "FlightController::slotSetNextWayPoint(): flying to" << wayPoint;
    mWayPoints.append(wayPoint);
}

QVector3D FlightController::getPosition()
{
    const Ogre::Vector3 pos = mMotionState->getPosition();
    return QVector3D(pos.x, pos.y, pos.z);
}

QQuaternion FlightController::getOrientation()
{
    const Ogre::Quaternion rot = mMotionState->getOrientation();
    return QQuaternion(rot.w, rot.x, rot.y, rot.z);
}

QList<QVector3D> FlightController::getWayPoints()
{
    return mWayPoints;
}

void FlightController::slotJoystickButtonStateChanged(unsigned char button, bool pressed)
{
    button++; // make it match the number on the joystick

    if(button <= 2 && pressed)
    {
        mAutoPilot = !mAutoPilot;
        if(mAutoPilot)
            emit message("AutoPilot enabled.");
        else
            emit message("AutoPilot disabled.");
    }

    if(button >= 5 && pressed)
    {
        if(mSimulator->isPaused())
        {
            mSimulator->slotSimulationStart();
            emit message("Simulation started.");
        }
        else
        {
            mSimulator->slotSimulationPause();
            emit message("Simulation paused.");
        }
    }
}
