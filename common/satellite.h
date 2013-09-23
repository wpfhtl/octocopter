#ifndef SATELLITE_H
#define SATELLITE_H

#include <QString>
#include <QColor>
#include "common.h"
#include "sgp4sdp4/sgp4sdp4.h"

class Almanac;

class Satellite
{
private:
    // A pointer back to the almanac that manages this satellite
    Almanac* mAlmanac;
    GnssConstellation mConstellation;
    QString mName;
    QString mLine1, mLine2;
    double mVelocity, mAzimuth, mElevation, mRange, mRangeRate, mLatitude, mLongitude;
    double mHeight, mMeanAnomaly, mFootprint, mAlon, mAlat, mNextAosLos;
    int mCatalogNumber;
    long int mOrbitNum;
    QString mNextAosLosString, mCalculatedDate;


public:
    Satellite(Almanac *almanac, const QString& name, const QString& line1, const QString& line2);

    GnssConstellation getConstellation() const {return mConstellation;}
    void setConstellation(GnssConstellation c) {mConstellation = c;}

    quint32 getCatalogNumber();
    void setCatalogNumber(const quint32 catalogNumber);

    QString getName() const {return mName;}
    QString getShortName() const;

    QColor getTextColor() const;

    float longitude();
    float latitude();
    float getAzimuth() const;
    float getElevation() const;
    QString nextAosLos();
    QString getCalculatedDate();
    void setCalculatedDate(const QString& date) {mCalculatedDate = date;}
    float footprint();
    float range();
    float altitude();
    float velocity();
    float squint();
    long orbitnum();
    float ma();  // AMSAT mean anomalie
    double doppler(double f);
    double ALON();
    double ALAT();

    void computeOrbit();
    void computeOrbit(double daynum, bool doAosLos);
    bool aosHappens(tle_t* tle, geodetic_t* obs);
    double findAOS(double daynum, bool hasAos);
    double findLOS(double daynum, bool hasLos);
    double findLOS2(double daynum, bool haslos);
    double nextAOS(double daynum, bool aoslos);
    double nextAosLos(double daynum, bool aoslos);
    QString daynum2String(double daynum);

    bool operator==(const Satellite& b) const
    {
        return mName == b.mName;
    }
};

uint qHash(const Satellite &key, uint seed);

#endif // SATELLITE_H
