#ifndef MOTIONCOMMAND_H
#define MOTIONCOMMAND_H

#include <QDebug>
#include <QDataStream>

class MotionCommand
{
public:
    // Sets thrust to hover, others to zero.
    MotionCommand();

    MotionCommand(const quint8 thrust, const qint8 yaw, const qint8 pitch, const qint8 roll);

    // If we pass a float of 2362.0f to quint8 thrust, it will be converted to 2362%256=58,
    // which is NOT what we want. Thus, we create a float-constructor that does the right thing.
    MotionCommand(const float thrust, const float yaw, const float pitch, const float roll);

    MotionCommand(const quint8 thrust, const float yaw, const float pitch, const float roll);

    MotionCommand clampedToSafeLimits() const;

    void adjustThrustToPitchAndRoll();

    QString toString() const;

    // These two are versions of qBound() that don't take references.
    // This way, we can pass static const arguments below.
    static quint8 clamp(const quint8 min, const quint8 val, const quint8 max)
    {
        if(val < min) return min;
        if(val > max) return max;
        return val;
    }

    static qint8 clamp(const qint8 min, const qint8 val, const qint8 max)
    {
        if(val < min) return min;
        if(val > max) return max;
        return val;
    }

    quint8 thrust;
    qint8 yaw;
    qint8 pitch;
    qint8 roll;

    // Tests have shown:
    // - with metal hood (2425 gram) it hovers at 127/128.
    // Guesses:
    // - without metal hood (95g) lets try 126
    // - without metal hood (95g) and with external wlan (75),  we stay at 127
    static const quint8 thrustMin = 50;
    static const quint8 thrustHover = 137;
    static const quint8 thrustMax = 200;

    // A yaw of 15 rotates by about 15 degrees per second, which is slooow
    static const qint8 yawMax = 60;

    // Lets limit to 25 for testing, this seems plenty
    static const qint8 rollMax = 25;
    static const qint8 pitchMax = 25;
};

// for using qDebug();
QDebug operator<<(QDebug dbg, const MotionCommand &mc);

// for streaming
QDataStream& operator<<(QDataStream &out, const MotionCommand &mc);
QDataStream& operator>>(QDataStream &in, MotionCommand &mc);

#endif // MOTIONCOMMAND_H
