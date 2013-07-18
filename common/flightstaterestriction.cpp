#include "flightstaterestriction.h"





// for streaming
QDataStream& operator<<(QDataStream &out, const FlightStateRestriction &fsr)
{
    out << static_cast<quint8>(fsr.restriction);
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightStateRestriction& fsr)
{
    quint8 restriction;
    in >> restriction;
    fsr.restriction = static_cast<FlightStateRestriction::Restriction>(restriction);
    return in;
}
