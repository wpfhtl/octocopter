#include "flightstate.h"

QString FlightState::toString(const FlightState& fs)
{
    switch(fs.state)
    {
    case FlightState::State::UserControl: return "UserControl"; break;
    case FlightState::State::ApproachWayPoint: return "ApproachWayPoint"; break;
    case FlightState::State::Hover: return "Hover"; break;
    case FlightState::State::Idle: return "Idle"; break;
    default:
        qDebug() << "FlightState::toString(): WARNING, state" << (quint8)fs.state << "is undefined!";
        return "Undefined";
        break;
    }

}

FlightState FlightState::fromString(const QString& flightStateString)
{
    if(flightStateString == "UserControl")
        return FlightState(FlightState::State::UserControl);
    else if(flightStateString == "ApproachWayPoint")
        return FlightState(FlightState::State::ApproachWayPoint);
    if(flightStateString == "Hover")
        return FlightState(FlightState::State::Hover);
    if(flightStateString == "Idle")
        return FlightState(FlightState::State::Idle);
    if(flightStateString == "Undefined")
        return FlightState(FlightState::State::Undefined);

    qDebug() << "FlightState::fromString(): undefined flightstate string:" << flightStateString;
}

FlightState& FlightState::operator=(const FlightState& other)
{
    state = other.state;
    return *this;
}

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightState &fs)
{
    out << static_cast<quint8>(fs.state);
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightState& fs)
{
    quint8 state;
    in >> state;
    fs.state = static_cast<FlightState::State>(state);
    return in;
}
