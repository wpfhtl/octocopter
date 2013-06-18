#ifndef SATELLITE_H
#define SATELLITE_H

#include <QString>
#include <QColor>
#include "common.h"
#include "sgp4sdp4/sgp4sdp4.h"

class Almanac;

class Satellite
{
public:
    enum class Constellation
    {
        ConstellationGps,
        ConstellationGlonass,
        ConstellationBeidou,
        ConstellationGalileo,
        ConstellationUnknown
    };
    /*
    struct TwoLineElement
    {
        QString name;
        qint32 number;
        float inclination;
        float rightAscension;
        float eccentricity;
        float argumentOfPerigee;
        float meanAnomaly;
        float meanMotion;

        TwoLineElement(const QString tleString)
        {
            const QString line1 = tleString.split("\n").at(1).simplified();
            const QString line2 = tleString.split("\n").at(2).simplified();

            name = tleString.split("\n").first().simplified();

            bool success;

            number = line1.mid(2, 5).toInt(&success);
            Q_ASSERT(success);

            inclination = line2.mid(9, 7).toFloat(&success);
            Q_ASSERT(success);

            rightAscension = line2.mid(18, 7).toFloat(&success);
            Q_ASSERT(success);

            eccentricity = QString("0.").append(line2.mid(27, 7)).toFloat(&success);
            Q_ASSERT(success);

            argumentOfPerigee = line2.mid(35, 8).toFloat(&success);
            Q_ASSERT(success);

            meanAnomaly = line2.mid(44, 8).toFloat(&success);
            Q_ASSERT(success);

            meanMotion = line2.mid(53, 11).toFloat(&success);
            Q_ASSERT(success);
        }

        QString toString() const
        {
            return QString("%1 #%2, inc %3, RA %4, ECC %5, ArgOfP %6 meanAnom %7, meanMotion %8")
                    .arg(name)
                    .arg(number)
                    .arg(inclination)
                    .arg(rightAscension)
                    .arg(eccentricity)
                    .arg(argumentOfPerigee)
                    .arg(meanAnomaly)
                    .arg(meanMotion);
        }
    };*/

private:
    //    TwoLineElement mTle;

    // A pointer back to the almanac that manages this satellite
    Almanac* mAlmanac;
    Constellation mConstellation;
    QString mName;
    QString mLine1, mLine2;
    double mVelocity, mAzimuth, mElevation, mRange, mRangeRate, mLatitude, mLongitude;
    double mHeight, mMeanAnomaly, mFootprint, mAlon, mAlat, mNextAosLos;
    int mCatalogNumber;
    long int mOrbitNum;
    QString mNextAosLosString, mCalculatedDate;


public:
    Satellite(Almanac *almanac, const QString& name, const QString& line1, const QString& line2);

    Constellation getConstellation() const {return mConstellation;}
    void setConstellation(Constellation c) {mConstellation = c;}

//    const QString& getLine1() const {return mLine1;}
//    const QString& getLine2() const {return mLine2;}

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
