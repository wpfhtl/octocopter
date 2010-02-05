#ifndef COORDINATECONVERTER_H
#define COORDINATECONVERTER_H

#include <QObject>
#include <math.h>

#include "coordinategps.h"
#include "coordinateogre.h"

class CoordinateConverter : public QObject
{
//    enum PositionSource {NONE, GPS, OGRE};

    Q_OBJECT
private:
    CoordinateGps mOrigin; // The Ogre-Coordinate-System unfolds from this GPS-Coordinate. Thus mPositionOgre is 0/0/0 at mOrigin

    // We use floating points, so to keep conversions rare, each set()ter
    // writes to its own datatype. Then we convert only if necessary.
//    PositionSource mPositionSource;
//    CoordinateGps mPositionGps;
//    CoordinateOgre mPositionOgre;


public:
    CoordinateConverter(const CoordinateGps &origin = CoordinateGps());
    void setOrigin(const CoordinateGps &origin);
//    void setPosition(const CoordinateGps &coordinate);
//    void setPosition(const CoordinateOgre &coordinate);

    CoordinateGps convert(const CoordinateOgre &coordinate) const;
    CoordinateOgre convert(const CoordinateGps &coordinate) const;
};

#endif
