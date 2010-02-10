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

CoordinateGps::CoordinateGps(const qreal &longitude, const qreal &latitude, const qreal &elevation)
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

qreal CoordinateGps::longitude(void) const
{
    return mLongitude;
}
qreal CoordinateGps::latitude(void) const
{
    return mLatitude;
}
qreal CoordinateGps::elevation(void) const
{
    return mElevation;
}

void CoordinateGps::setLongitude(const qreal &longitude)
{
    this->mLongitude = longitude;
}

void CoordinateGps::setLatitude(const qreal &latitude)
{
    this->mLatitude = latitude;
}

void CoordinateGps::setElevation(const qreal &elevation)
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
