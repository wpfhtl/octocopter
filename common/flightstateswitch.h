#ifndef FLIGHTSTATESWITCH_H
#define FLIGHTSTATESWITCH_H

#include <QtCore>

struct FlightStateRestriction
{
    enum class Restriction
    {
        RestrictionUserControl, // ~-129
        RestrictionHover,       // ~22
        RestrictionNone         // ~+184
    };

    // The actual value of the switch
    Restriction restriction;

    FlightStateRestriction()
    {
        restriction = Restriction::RestrictionUserControl;
    }

    FlightStateRestriction(const qint16 ppmValue)
    {
        if(ppmValue <= -80)
            restriction = Restriction::RestrictionUserControl;
        else if(ppmValue > -80 && ppmValue < 80)
            restriction = Restriction::RestrictionHover;
        else
            restriction = Restriction::RestrictionNone;
    }
/*
    FlightStateRestriction(const FlightStateRestriction::Restriction restriction)
    {
        this->restriction = restriction;
    }

*/
    bool operator!=(const FlightStateRestriction& b) const
    {
        return b.restriction != restriction;
    }
    /*
    FlightStateRestriction& operator=(const FlightStateRestriction& other)
    {
        restriction = other.restriction;
        return *this;
    }*/

    QString toString() const
    {
        switch(restriction)
        {
        case Restriction::RestrictionUserControl: return "RestrictionUserControl"; break;
        case Restriction::RestrictionHover: return "RestrictionHover"; break;
        case Restriction::RestrictionNone: return "RestrictionNone"; break;
        }

        return QString("undefined Value");
    }
};

#endif // FLIGHTSTATESWITCH_H
