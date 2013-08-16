#ifndef FLIGHTSTATESWITCH_H
#define FLIGHTSTATESWITCH_H

#include <QtCore>
#include "flightstate.h"

struct FlightStateRestriction
{
    enum class Restriction
    {
        // The values don't have a real meaning, just set to something special for debugging values.
        RestrictionUserControl = 131, // ~-129
        RestrictionHover       = 132, // ~22
        RestrictionNone        = 133  // ~+184
    };

    // The actual value of the switch
    Restriction restriction;

    FlightStateRestriction()
    {
        // By default, be strict!
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

    bool allowsFlightState(const FlightState::State& fs)
    {
        if(restriction == Restriction::RestrictionNone)
        {
            if(fs == FlightState::State::Hover || fs == FlightState::State::Idle || fs == FlightState::State::ApproachWayPoint)
                return true;
        }
        else if(restriction == Restriction::RestrictionHover)
        {
            if(fs == FlightState::State::Hover)
                return true;
        }
        else if(restriction == Restriction::RestrictionUserControl)
        {
            if(fs == FlightState::State::UserControl)
                return true;
        }

        qDebug() << __PRETTY_FUNCTION__ << "flightstate" << FlightState(fs).toString() << "is not allowed due to restriction" << toString();
        return false;
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

// for streaming
QDataStream& operator<<(QDataStream &out, const FlightStateRestriction &fs);
QDataStream& operator>>(QDataStream &in, FlightStateRestriction &fs);

#endif // FLIGHTSTATESWITCH_H
