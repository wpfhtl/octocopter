#include "flightstaterestriction.h"





// for streaming
QDataStream& operator<<(QDataStream &out, const FlightStateRestriction &fsr)
{
    quint8 restriction = static_cast<quint8>(fsr.restriction);
    out << restriction;
    //qDebug() << "sending fsr" << fsr.toString() << "as quint8" << restriction;
    return out;
}

QDataStream& operator>>(QDataStream &in, FlightStateRestriction& fsr)
{
    quint8 restriction;
    in >> restriction;
    fsr.restriction = static_cast<FlightStateRestriction::Restriction>(restriction);
//    qDebug() << "reconstructed fsr" << fsr.toString() << "from quint8" << restriction;
    return in;
}
