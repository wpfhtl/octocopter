#ifndef MOTIONCOMMAND_H
#define MOTIONCOMMAND_H

#include <QDebug>
#include <QDataStream>
#include <QStringList>

#include <math.h> // ceil and floor

//TODO: I think FlightControllerValues is the only direct user of this. Merge them?


class MotionCommand
{
public:
    MotionCommand(const quint8 thrust = 0, const qint8 yaw = 0, const qint8 pitch = 0, const qint8 roll = 0);

    // Must be able to deserialize toString()
    MotionCommand(const QString& string);

    MotionCommand clampedToSafeLimits() const;

    QString toString() const;

    quint8 thrust;
    qint8 yaw;
    qint8 pitch;
    qint8 roll;

    // Tests have shown:
    // - with metal hood (2425 gram) it hovers at 127/128.
    // Guesses:
    // - without metal hood (95g) lets try 126
    // - without metal hood (95g) and with external wlan (75),  we stay at 127
    static const quint8 thrustHover = 127.0;
};

// for using qDebug();
QDebug operator<<(QDebug dbg, const MotionCommand &mc);

// for streaming
QDataStream& operator<<(QDataStream &out, const MotionCommand &mc);
QDataStream& operator>>(QDataStream &in, MotionCommand &mc);

#endif // MOTIONCOMMAND_H
