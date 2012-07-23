#include "motioncommand.h"

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

MotionCommand::MotionCommand(const QString& string)
{
    QStringList list = string.split(" ", QString::KeepEmptyParts);
    Q_ASSERT(list.size() == 8);

    bool success = false;

    thrust = list.at(1).toInt(&success);
    Q_ASSERT(success);
    yaw = list.at(3).toInt(&success);
    Q_ASSERT(success);
    pitch = list.at(5).toInt(&success);
    Q_ASSERT(success);
    roll = list.at(7).toInt(&success);
    Q_ASSERT(success);
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
    dbg.nospace() << "thrust:" << mc.thrust << ", yaw:" << mc.yaw << ", pitch:" << mc.pitch << ", roll:" << mc.roll;
    return dbg.space();
}

MotionCommand MotionCommand::clampedToSafeLimits() const
{
    MotionCommand clamped;

    // As documented in the header, about 127 means hovering.
    clamped.thrust = (quint8)qBound(100.0f, (float)thrust, 140.0f);

    // A yaw of 15 rotates by about 15 degrees per second, which is slooow
    clamped.yaw = (qint8)qBound(-50.0, yaw > 0.0f ? ceil(yaw) : floor(yaw), 50.0);

    // Lets limit to 10 for testing, this seems plenty
    clamped.pitch = (qint8)qBound(-10.0f, (float)pitch, 10.0f);
    clamped.roll = (qint8)qBound(-10.0f, (float)roll, 10.0f);

    // For safety, we don't need it right now.
    clamped.roll = 0;

    return clamped;
}
