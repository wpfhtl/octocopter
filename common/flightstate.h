#ifndef FLIGHTSTATE_H
#define FLIGHTSTATE_H

#include <QString>
#include <QDebug>
#include <QDataStream>

class FlightState
{
public:

    enum class Value
    {
        UserControl,
        Hover,
        ApproachWayPoint,
        Idle,
        Undefined
    };

    FlightState() : state(Value::Undefined) {}
    FlightState(const Value s) : state(s) {}
//    FlightState(const QString s) {state = FlightState::fromString(s);}

    Value state;

    QString toString() const {return FlightState::toString(state);}

    bool operator==(FlightState const& other) const {return other.state == state;}
    bool operator!=(FlightState const& other) const {return other.state != state;}

    bool operator==(FlightState::Value const& other) const {return other == state;}
    bool operator!=(FlightState::Value const& other) const {return other != state;}


    FlightState &operator=(const FlightState& other);


    static QString toString(const FlightState& fs);
    static FlightState fromString(const QString& flightStateString);
};

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightState &fs);
QDataStream& operator>>(QDataStream &in, FlightState &fs);

#endif // FLIGHTSTATE_H
