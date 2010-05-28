#ifndef COORDINATEGPS_H
#define COORDINATEGPS_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <cmath>

class CoordinateGps : public QObject
{
    Q_OBJECT

private:
    double mLongitude, mLatitude, mElevation;

public:
    CoordinateGps();
    CoordinateGps(const CoordinateGps &coordinate);
    CoordinateGps(const double &longitude, const double &latitude, const double &elevation);
    double longitude(void) const;
    double latitude(void) const;
    double elevation(void) const;

    void setLongitude(const double &longitude);
    void setLatitude(const double &latitude);
    void setElevation(const double &elevation);

    QString toString(void) const;
    QString formatGpsDegree(const double value, const int secondPrecision = 2) const;

    CoordinateGps& operator=(const CoordinateGps &other);
};

QDebug operator<<(QDebug dbg, const CoordinateGps &c);

QDataStream & operator<<(QDataStream &out, const CoordinateGps &cg); // for writing a cg into a stream
QDataStream & operator>>(QDataStream &in, CoordinateGps &cg); // for reading from a stream into the cg

#endif
