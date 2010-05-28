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
            "Lon " + QString::number(mLongitude, 'f', 16) + ", Lat " + QString::number(mLatitude, 'f', 16) + "\n" +
            "Lon " + formatGpsDegree(mLongitude, 8) + " " +
            "Lat " + formatGpsDegree(mLatitude, 8) + " " +
            "Elv " + QString::number(mElevation, 'f', 5) + "m"
            );
}

QString CoordinateGps::formatGpsDegree(const double value, const int secondPrecision) const
{
    QString result;
//    const int deg = value;
    const double min = (value-((int)value)) * 60;
    const double sec = (min - ((int)min)) * 60;
    return result.sprintf("%d%c %d' %.*f\"", (int)value, 176, (int)min, secondPrecision, sec);
}

QDebug operator<<(QDebug dbg, const CoordinateGps &c)
 {
//     dbg.nospace() << "CoordinateGps(" << QString::number(c.longitude(), 'f', 8) << ", " << QString::number(c.latitude(), 'f', 8) << ", " << c.elevation() << ")";
     dbg.nospace() << c.toString();

     return dbg.space();
 }


// for reading from a stream into the cg
QDataStream& operator>>(QDataStream &in, CoordinateGps &cg)
{
//    qDebug() << "QDataStream& operator>>(QDataStream &in, CoordinateGps &cg):" << cg;
    double lon, lat, ele;
    in >> lon;
    in >> lat;
    in >> ele;
//    cg.setLongitude(lon);
//    cg.setLatitude(lat);
//    cg.setElevation(ele);

    cg = CoordinateGps(lon, lat, ele);

    return in;
}

// for writing a cg into a stream
QDataStream& operator<<(QDataStream& out, const CoordinateGps &cg)
{
//    qDebug() << "QDataStream& operator<<(QDataStream &out, const CoordinateGps &cg):" << cg;

    out << cg.longitude();
    out << cg.latitude();
    out << cg.elevation();

    return out;
}



//<peppe> kernelpanic: like, add these two operators as non-members of your class: QDataStream & operator>> (QDataStream & stream, YourClass & object) , and QDataStream & operator<<(QDataStream& stream, const YourClass & object)
//<peppe> kernelpanic: then just create a QDataStream operating on a QByteArray and use that to serialize/deserialize your list
//<peppe> kernelpanic: you can use all the QDataStream methods you want. if you have to send something like an integer, then 3 doubles, then a string, etc. you can right use QDataStream existing operators
