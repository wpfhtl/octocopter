#ifndef FLIGHTSTATE_H
#define FLIGHTSTATE_H

#include <QString>
#include <QDebug>
#include <QDataStream>

class FlightState
{
public:

    enum class State
    {
        UserControl,
        Idle,
        Hover,
        ApproachWayPoint,
        Undefined
    };

    FlightState() : state(State::Undefined) {}
    FlightState(const State s) : state(s) {}
//    FlightState(const QString s) {state = FlightState::fromString(s);}

    State state;

    QString toString() const {return FlightState::toString(state);}

    bool operator==(FlightState const& other) const {return other.state == state;}
    bool operator!=(FlightState const& other) const {return other.state != state;}

    bool operator==(FlightState::State const& other) const {return other == state;}
    bool operator!=(FlightState::State const& other) const {return other != state;}


    FlightState &operator=(const FlightState& other);


    static QString toString(const FlightState& fs);
    static FlightState fromString(const QString& flightStateString);
};

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightState &fs);
QDataStream& operator>>(QDataStream &in, FlightState &fs);

#endif // FLIGHTSTATE_H
