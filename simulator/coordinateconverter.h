#ifndef COORDINATECONVERTER_H
#define COORDINATECONVERTER_H

#include <QObject>
#include <QSettings>
#include <Ogre.h>
#include <math.h>

#include "coordinategps.h"
//#include "coordinateogre.h"

class CoordinateConverter : public QObject
{

    Q_OBJECT
private:
    CoordinateGps mOrigin; // The Ogre-Coordinate-System unfolds from this GPS-Coordinate. Thus mPositionOgre is 0/0/0 at mOrigin

public:
    CoordinateConverter(void);
    CoordinateConverter(const CoordinateGps &origin);

    void setOrigin(const CoordinateGps &origin);

    CoordinateGps convert(const Ogre::Vector3 &coordinate) const;
    Ogre::Vector3 convert(const CoordinateGps &coordinate) const;
};

#endif
