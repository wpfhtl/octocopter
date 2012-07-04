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
