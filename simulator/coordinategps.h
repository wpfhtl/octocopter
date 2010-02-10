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
    qreal mLongitude, mLatitude, mElevation;

public:
    CoordinateGps();
    CoordinateGps(const CoordinateGps &coordinate);
    CoordinateGps(const qreal &longitude, const qreal &latitude, const qreal &elevation);
    qreal longitude(void) const;
    qreal latitude(void) const;
    qreal elevation(void) const;

    void setLongitude(const qreal &longitude);
    void setLatitude(const qreal &latitude);
    void setElevation(const qreal &elevation);

    QString toString(void) const;

    CoordinateGps& operator=(const CoordinateGps &other);
};

QDebug operator<<(QDebug dbg, const CoordinateGps &c);

#endif
