#ifndef MOTIONCOMMAND_H
#define MOTIONCOMMAND_H

#include <QDebug>
#include <QDataStream>

struct MotionCommand
{
public:
    MotionCommand(const quint8 thrust = 0, const qint8 yaw = 0, const qint8 pitch = 0, const qint8 roll = 0);

    MotionCommand clampedToSafeLimits();

    QString toString() const;

    quint8 thrust;
    qint8 yaw;
    qint8 pitch;
    qint8 roll;
};

// for using qDebug();
QDebug operator<<(QDebug dbg, const MotionCommand &mc);

// for streaming
QDataStream& operator<<(QDataStream &out, const MotionCommand &mc);
QDataStream& operator>>(QDataStream &in, MotionCommand &mc);

#endif // MOTIONCOMMAND_H
