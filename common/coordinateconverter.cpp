#include "coordinateconverter.h"

CoordinateConverter::CoordinateConverter(void)
{
    QSettings settings("BenAdler", "simulator");
    mOrigin = CoordinateGps(
                        settings.value("gps/origin/longitude").toReal(),
                        settings.value("gps/origin/latitude").toReal(),
                        settings.value("gps/origin/elevation").toReal()
                        );

    mOrigin = CoordinateGps(53.5592, 9.83, 0);
}

CoordinateConverter::CoordinateConverter(const CoordinateGps &origin) : QObject()
{
    mOrigin = origin;
}

void CoordinateConverter::setOrigin(const CoordinateGps &origin)
{
    mOrigin = origin;
}

// In the following two methods, we ignore the geoid: http://www.kompf.de/gps/distcalc.html
// Which ogre-Coordinate does this GPS-Coordinate have? That
// is, how far in x/y/z-meters is it away from mOrigin?
//CoordinateOgre CoordinateConverter::convert(const CoordinateGps &coordinate) const
//{
//    CoordinateOgre co;
//    co.y = (coordinate.elevation() - mOrigin.elevation());
//    co.z = (-(coordinate.latitude() - mOrigin.latitude()) * 11300.0);
//    co.x = ((coordinate.longitude() - mOrigin.longitude()) * 111300.0 * cos(3.14159265 / 180.0 * mOrigin.latitude()));
//    return co;
//}

Ogre::Vector3 CoordinateConverter::convert(const CoordinateGps &coordinate) const
{
    Ogre::Vector3 co;
    co.y = (coordinate.elevation() - mOrigin.elevation());
    co.z = (-(coordinate.latitude() - mOrigin.latitude()) * 111300.0);

    co.x = ((coordinate.longitude() - mOrigin.longitude()) * 111300.0 * cos(M_PI / 180.0 * mOrigin.latitude()));

//    qDebug() << coordinate << "=>" << co.x << co.y << co.z;
    return co;
}

CoordinateGps CoordinateConverter::convert(const Ogre::Vector3 &coordinate) const
{
    CoordinateGps cg;
    cg.setElevation(mOrigin.elevation() + coordinate.y); // height is Y, easy
    cg.setLatitude(mOrigin.latitude() - (coordinate.z / 111300.0)); // north/south is Z

    // Don't divide coordinate.x / 111300.0 and then continue, as that will yield a very small number. Rather, precompute the divisor, then divide.
    // I don't really understand this, is Ogre::Vector3 using simple floats?
    cg.setLongitude(mOrigin.longitude() + (coordinate.x / (111300.0 * cos(M_PI / 180.0 * cg.latitude())))); // east/west is X

//    qDebug() << coordinate.x << coordinate.y << coordinate.z << "=>" << cg;
    return cg;
}

//QString CoordinateConverter::formatGpsDegree(const float value) const
//{
//    QString result;
//    const int deg = value;
//    const float min = (value-deg) * 60;
//    const float sec = (min - ((int)min)) * 60;
//    return result.sprintf("%d%c %d' %.2f\"", deg, 176, (int)min, sec);
//}
