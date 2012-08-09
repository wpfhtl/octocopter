#include "flightstate.h"

QString FlightState::toString(const FlightState& fs)
{
    switch(fs.state)
    {
    case FlightState::Value::UserControl: return "UserControl"; break;
    case FlightState::Value::ApproachWayPoint: return "ApproachWayPoint"; break;
    case FlightState::Value::Hover: return "Hover"; break;
    case FlightState::Value::Idle: return "Idle"; break;
    default:
        qDebug() << "FlightState::toString(): WARNING, state" << (quint8)fs.state << "is undefined!";
        return "Undefined";
        break;
    }

}

FlightState FlightState::fromString(const QString& flightStateString)
{
    if(flightStateString == "UserControl")
        return FlightState(FlightState::Value::UserControl);
    else if(flightStateString == "ApproachWayPoint")
        return FlightState(FlightState::Value::ApproachWayPoint);
    if(flightStateString == "Hover")
        return FlightState(FlightState::Value::Hover);
    if(flightStateString == "Idle")
        return FlightState(FlightState::Value::Idle);
    if(flightStateString == "Undefined")
        return FlightState(FlightState::Value::Undefined);

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
    out << (quint8)fs.state;
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightState& fs)
{
    in >> (quint8&)fs.state;
    return in;
}
