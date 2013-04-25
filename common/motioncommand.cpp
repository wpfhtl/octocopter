#include "motioncommand.h"
#include "common.h"
#include <QVector3D>
#include <QQuaternion>
#include <math.h> // ceil and floor

MotionCommand::MotionCommand()
{
    thrust = thrustHover;
    yaw = 0;
    pitch = 0;
    roll = 0;
}

MotionCommand::MotionCommand(const quint8 thrust, const qint8 yaw, const qint8 pitch, const qint8 roll)
{
    this->thrust = thrust;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
}

MotionCommand::MotionCommand(const float thrust, const float yaw, const float pitch, const float roll)
{
    this->thrust = (quint8)qBound(0.0f, thrust, 255.0f);
    this->yaw = (qint8)qBound(-127.0f, yaw, 127.0f);
    this->pitch = (qint8)qBound(-127.0f, pitch, 127.0f);
    this->roll = (qint8)qBound(-127.0f, roll, 127.0f);
}

MotionCommand::MotionCommand(const quint8 thrust, const float yaw, const float pitch, const float roll)
{
    this->thrust = thrust;
    this->yaw = (qint8)qBound(-127.0f, yaw, 127.0f);
    this->pitch = (qint8)qBound(-127.0f, pitch, 127.0f);
    this->roll = (qint8)qBound(-127.0f, roll, 127.0f);
}

QString MotionCommand::toString() const
{
    return QString ("thrust %1 yaw %2 pitch %3 roll %4").arg(thrust).arg(yaw).arg(pitch).arg(roll);
}

QDataStream& operator<<(QDataStream &out, const MotionCommand &mc)
{
    out << mc.thrust;
    out << mc.yaw;
    out << mc.pitch;
    out << mc.roll;

    return out;
}

QDataStream& operator>>(QDataStream &in, MotionCommand &mc)
{
    in >> mc.thrust;
    in >> mc.yaw;
    in >> mc.pitch;
    in >> mc.roll;

    return in;
}

QDebug operator<<(QDebug dbg, const MotionCommand &mc)
{
    dbg << mc.toString();
    return dbg;
}

MotionCommand MotionCommand::clampedToSafeLimits() const
{
    MotionCommand clamped;

    // As documented in the header, about 127 means hovering.
    clamped.thrust = (quint8)qBound(50.0f, (float)thrust, 180.0f);

    // A yaw of 15 rotates by about 15 degrees per second, which is slooow
    clamped.yaw = (qint8)qBound(-50.0, yaw > 0.0f ? ceil(yaw) : floor(yaw), 50.0);

    // Lets limit to 25 for testing, this seems plenty
    clamped.pitch = (qint8)qBound(-25.0f, (float)pitch, 25.0f);
    clamped.roll = (qint8)qBound(-25.0f, (float)roll, 25.0f);

    return clamped;
}

void MotionCommand::adjustThrustToPitchAndRoll()
{
    const QVector3D upWorld(0,1,0);
    const QQuaternion pitchAndRoll(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), pitch) * QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), roll));
    const QVector3D upVehicle = pitchAndRoll.rotatedVector(upWorld);

    const float angleBetweenVehicleAndWorldUpInRad = acos(QVector3D::dotProduct(upWorld, upVehicle));
    const quint8 addedThrust = MotionCommand::thrustHover * (float)sin(angleBetweenVehicleAndWorldUpInRad);
    qDebug() << "MotionCommand::adjustThrustToPitchAndRoll(): vehicle points" << RAD2DEG(angleBetweenVehicleAndWorldUpInRad) << "deg away from world up, amplifying thrust from" << thrust << "to" << thrust + addedThrust;
    thrust += addedThrust;
}
