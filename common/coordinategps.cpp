#include "coordinategps.h"

CoordinateGps::CoordinateGps() : QObject()
{
    this->mLongitude = 0.0;
    this->mLatitude = 0.0;
    this->mElevation = 0.0;
}

CoordinateGps::CoordinateGps(const CoordinateGps &coordinate)
{
    this->mLongitude = coordinate.mLongitude;
    this->mLatitude = coordinate.mLatitude;
    this->mElevation = coordinate.mElevation;
}

CoordinateGps::CoordinateGps(const double &longitude, const double &latitude, const double &elevation)
{
    this->mLongitude = longitude;
    this->mLatitude = latitude;
    this->mElevation = elevation;
}

CoordinateGps& CoordinateGps::operator=(const CoordinateGps &other)
{
        mLongitude = other.mLongitude;
        mLatitude = other.mLatitude;
        mElevation = other.mElevation;

        return *this;
}

double CoordinateGps::longitude(void) const
{
    return mLongitude;
}
double CoordinateGps::latitude(void) const
{
    return mLatitude;
}
double CoordinateGps::elevation(void) const
{
    return mElevation;
}

void CoordinateGps::setLongitude(const double &longitude)
{
    this->mLongitude = longitude;
}

void CoordinateGps::setLatitude(const double &latitude)
{
    this->mLatitude = latitude;
}

void CoordinateGps::setElevation(const double &elevation)
{
    this->mElevation = elevation;
}

QString CoordinateGps::toString(void) const
{
    return QString(
            "Lon " + QString::number(mLongitude, 'f', 0) + "° " +
            QString::number(60.0 * fmod(mLongitude, 1.0), 'f', 0) + "' " +
            QString::number(3600.0 * fmod(mLongitude, 1.0/60), 'f', 2) + "\" " +

            "Lat " + QString::number(mLatitude, 'f', 0) + "° " +
            QString::number(60.0 * fmod(mLatitude, 1.0), 'f', 0) + "' " +
            QString::number(3600.0 * fmod(mLatitude, 1.0/60), 'f', 2) + "\" " +

            "Elv " + QString::number(mElevation, 'f', 2) + "m"
            );
}

QDebug operator<<(QDebug dbg, const CoordinateGps &c)
 {
     dbg.nospace() << "(" << c.longitude() << ", " << c.latitude() << ", " << c.elevation() << ")";

     return dbg.space();
 }

QDataStream& operator>>(QDataStream & stream, CoordinateGps &cg)
{
    double lon, lat, ele;
    stream >> lon;
    stream >> lat;
    stream >> ele;
    cg.setLongitude(lon);
    cg.setLatitude(lat);
    cg.setElevation(ele);
}

/*
QDataStream& operator<<(QDataStream& stream, const CoordinateGps &object)
{
    stream << object.longitude();
    stream << object.latitude();
    stream << object.elevation();
}

QDataStream& operator<<(QDataStream& stream, const QList<CoordinateGps> &list)
{
    foreach (const CoordinateGps& c, list)
    {
        stream << c;
    }
//    stream << object.longitude();
//    stream << object.latitude();
//    stream << object.elevation();
}
*/

//<peppe> kernelpanic: like, add these two operators as non-members of your class: QDataStream & operator>> (QDataStream & stream, YourClass & object) , and QDataStream & operator<<(QDataStream& stream, const YourClass & object)
//<peppe> kernelpanic: then just create a QDataStream operating on a QByteArray and use that to serialize/deserialize your list
//<peppe> kernelpanic: you can use all the QDataStream methods you want. if you have to send something like an integer, then 3 doubles, then a string, etc. you can right use QDataStream existing operators
