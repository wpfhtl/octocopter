#ifndef FLIGHTSTATESWITCH_H
#define FLIGHTSTATESWITCH_H

#include <QtCore>

class FlightStateSwitch
{
public:

    enum FlightStateSwitchValue
    {
        UserControl,       // ~-129
        Hover,             // ~22
        ApproachWayPoint   // ~+184
    };

    FlightStateSwitch()
    {
        value = UserControl;
    }

    FlightStateSwitch(const qint16 ppmValue)
    {
        if(ppmValue <= -80)
            value = UserControl;
        else if(ppmValue > -80 && ppmValue < 80)
            value = Hover;
        else
            value = ApproachWayPoint;
    }

    FlightStateSwitch(const FlightStateSwitch::FlightStateSwitchValue fssw)
    {
        value = fssw;
    }

    // The actual value of the switch
    FlightStateSwitchValue value;

    bool operator!=(const FlightStateSwitch& b) const
    {
        return b.value != value;
    }

    FlightStateSwitch& operator=(const FlightStateSwitch& other)
    {
        value = other.value;
        return *this;
    }

    QString toString() const
    {
        switch(value)
        {
        case UserControl: return "UserControl"; break;
        case Hover: return "Hover"; break;
        case ApproachWayPoint: return "ApproachWayPoint"; break;
        }

        return QString("undefined FlightStateSwitchValue");
    }
};

#endif // FLIGHTSTATESWITCH_H
