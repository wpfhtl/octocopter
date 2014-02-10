#include "common.h"

bool testBit(quint16 number, quint8 bit)
{
    return ((number & ipow(2, bit)) != 0);
}

bool testBitEqual(quint16 number1, quint16 number2, quint8 bit)
{
    quint16 mask = ipow(2, bit);

    return (number1 & mask) == (number2 & mask);
}

QDebug operator<<(QDebug dbg, const GnssConstellation &c)
{
    if(c == GnssConstellation::ConstellationGps) dbg << "GPS";
    else if(c == GnssConstellation::ConstellationGlonass) dbg << "Glonass";
    else if(c == GnssConstellation::ConstellationCompass) dbg << "Compass";
    else if(c == GnssConstellation::ConstellationGalileo) dbg << "Galileo";
    else if(c == GnssConstellation::ConstellationSbas) dbg << "Sbas";
    else if(c == GnssConstellation::ConstellationQzss) dbg << "Qzss";
    else dbg << "Unknown!";

    return dbg;
}

QString t() { return QTime::currentTime().toString("HH:mm:ss:zzz"); }
