#ifndef FLIGHTSTATESWITCH_H
#define FLIGHTSTATESWITCH_H

#include <QtCore>

class FlightStateSwitch
{
public:

    enum class Value
    {
        UserControl,       // ~-129
        Hover,             // ~22
        ApproachWayPoint   // ~+184
    };

    FlightStateSwitch()
    {
        value = Value::UserControl;
    }

    FlightStateSwitch(const qint16 ppmValue)
    {
        if(ppmValue <= -80)
            value = Value::UserControl;
        else if(ppmValue > -80 && ppmValue < 80)
            value = Value::Hover;
        else
            value = Value::ApproachWayPoint;
    }

    FlightStateSwitch(const FlightStateSwitch::Value fssw)
    {
        value = fssw;
    }

    // The actual value of the switch
    Value value;

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
        case Value::UserControl: return "UserControl"; break;
        case Value::Hover: return "Hover"; break;
        case Value::ApproachWayPoint: return "ApproachWayPoint"; break;
        }

        return QString("undefined Value");
    }
};

#endif // FLIGHTSTATESWITCH_H
