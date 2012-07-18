#include "motioncommand.h"

MotionCommand::MotionCommand(const quint8 thrust, const qint8 yaw, const qint8 pitch, const qint8 roll)
{
    this->thrust = thrust;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
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
    dbg.nospace() << "thrust:" << mc.thrust << ", yaw:" << mc.yaw << ", pitch:" << mc.pitch << ", roll:" << mc.roll;
    return dbg.space();
}

QString MotionCommand::toString() const
{
    return QString ("thrust %1, yaw %2, pitch %3, roll %4").arg(thrust).arg(yaw).arg(pitch).arg(roll);
}

MotionCommand MotionCommand::clampedToSafeLimits()
{
    MotionCommand clamped;

    // As documented in FlightController, about 127 means hovering.
    clamped.thrust = (quint8)qBound(100.0f, thrust, 140.0f);

    // A yaw of 15 rotates by about 15 degrees per second, which is slooow
    clamped.yaw = (qint8)qBound(-50.0, yaw > 0.0f ? ceil(yaw) : floor(yaw), 50.0);

    // Lets limit to 10 for testing, this seems plenty
    clamped.pitch = (qint8)qBound(-10.0f, pitch, 10.0f);
    clamped.roll = (qint8)qBound(-10.0f, roll, 10.0f);

    // For safety, we don't need it right now.
    clamped.roll = 0;

    return clamped;
}
