#include "satellite.h"
#include "almanac.h"
#include <sys/time.h>

Satellite::Satellite(Almanac *almanac, const QString &name, const QString &line1, const QString &line2)
{
    mAlmanac = almanac;
    mName = name.simplified();
    mLine1 = line1;
    mLine2 = line2;
    mNextAosLos = 0.0;
}

QString Satellite::getShortName() const
{
    if(mConstellation == Constellation::ConstellationGps)
    {
        quint32 indexOfPRN = mName.indexOf("PRN");
        return QString("G%1").arg(mName.mid(indexOfPRN + 4, 2));
    }
    else if(mConstellation == Constellation::ConstellationGlonass)
    {
        return QString("R%1").arg(mName.mid(9, 2));
    }
    else if(mConstellation == Constellation::ConstellationGalileo)
    {
        if(mName.contains("GSAT"))
            return QString("E%1").arg(mName.right(3).left(2));
        else
            return QString("E%1").arg(mName.right(1));
    }
    else if(mConstellation == Constellation::ConstellationBeidou)
    {
        return QString("C%1").arg(mName);
    }

    return QString("??");
}

float Satellite::longitude()
{
    return mLongitude;
}

float Satellite::latitude()
{
    return mLatitude;
}

float Satellite::getAzimuth() const
{
    return mAzimuth;
}

float Satellite::getElevation() const
{
    return mElevation;
}

QString Satellite::nextAosLos()
{
    return mNextAosLosString;
}

QString Satellite::getCalculatedDate()
{
    return mCalculatedDate;
}

float Satellite::footprint()
{
    return mFootprint;
}

float Satellite::range()
{
    return mRange;
}

float Satellite::altitude()
{
    return mHeight;
}

float Satellite::velocity()
{
    return mVelocity;
}

long Satellite::orbitnum()
{
    return mOrbitNum;
}

float Satellite::ma()
{
    return mMeanAnomaly;
}

double Satellite::doppler(double f)
{
    double result;
    result = -f*((mRangeRate*1000.0)/299792458.0);
    return result;
}

double Satellite::ALON()
{
    return mAlon;
}

double Satellite::ALAT()
{
    return mAlat;
}

quint32 Satellite::getCatalogNumber()
{
    return mCatalogNumber;
}

void Satellite::setCatalogNumber(const quint32 catalogNumber)
{
    mCatalogNumber = catalogNumber;
}


void Satellite::computeOrbit()
{
    struct tm utc;
    struct timeval tmval;
    double daynum;
    UTC_Calendar_Now(&utc);
    gettimeofday(&tmval, NULL);
    daynum = Julian_Date(&utc);
    daynum = daynum+(double)tmval.tv_usec/8.64e+10;
    computeOrbit(daynum, true);
}

void Satellite::computeOrbit(double daynum, bool doAosLos)
{
    bool hasAosLos;
    char tle_lines[3][80];
    geodetic_t observerGeodetic;
    tle_t tle;

    mCalculatedDate = daynum2String(daynum);

    const PositionGeodetic receiverPos = mAlmanac->getReceiverPosition();
    observerGeodetic.lon = DEG2RAD(receiverPos.longitude);
    observerGeodetic.lat = DEG2RAD(receiverPos.latitude);
    observerGeodetic.alt= receiverPos.elevation/1000.0;
    observerGeodetic.theta=0.0;

    strcpy(tle_lines[0], "DUMMY");
    strcpy(tle_lines[1], mLine1.toLatin1());
    strcpy(tle_lines[2], mLine2.toLatin1());

    ClearFlag(ALL_FLAGS);
    Get_Next_Tle_Set(tle_lines, &tle);

    hasAosLos = aosHappens(&tle, &observerGeodetic);

    mCatalogNumber = tle.catnr;

    select_ephemeris(&tle);
    double jul_epoch, jul_utc, tsince, phase, age;
    vector_t vel = {0,0,0,0};
    vector_t pos = {0,0,0,0};
    vector_t obs_set;
    geodetic_t satelliteGeodetic;

    jul_utc = daynum;
    jul_epoch = Julian_Date_of_Epoch(tle.epoch);
    tsince = (jul_utc - jul_epoch) * xmnpda;

    // call the norad routines according to the deep-space flag
    if (isFlagSet(DEEP_SPACE_EPHEM_FLAG))
        SDP4(tsince, &tle, &pos, &vel, &phase);
    else
        SGP4(tsince, &tle, &pos, &vel, &phase);
    // scale position and velocity to km and km/sec
    Convert_Sat_State(&pos, &vel);

    // get the velocity of the Satellite
    Magnitude(&vel);
    mVelocity = vel.w;
    Calculate_Obs(jul_utc, &pos, &vel, &observerGeodetic, &obs_set);
    Calculate_LatLonAlt(jul_utc, &pos, &satelliteGeodetic);

    mAzimuth = Degrees(obs_set.x);
    mElevation = Degrees(obs_set.y);
    mRange = obs_set.z;
    mRangeRate = obs_set.w;
    mLatitude = Degrees(satelliteGeodetic.lat);
    mLongitude = Degrees(satelliteGeodetic.lon);
    mHeight = satelliteGeodetic.alt;
    mMeanAnomaly = Degrees(phase);
    mMeanAnomaly = (256.0/360.0) * mMeanAnomaly;
    mFootprint = 2.0 * xkmper * acos(xkmper / pos.w);
    age=jul_utc - jul_epoch;
    mOrbitNum = (long)floor((tle.xno*xmnpda/twopi+age*tle.bstar*ae)*age+tle.xmo/twopi)+tle.revnum-1;

    // squint computation removed.

    if(jul_utc > mNextAosLos && doAosLos && hasAosLos && false)
    {
        mNextAosLos = nextAosLos(jul_utc, hasAosLos);
        mNextAosLosString = daynum2String(mNextAosLos);
    }
}

bool Satellite::aosHappens(tle_t* tle, geodetic_t* obs)
{
    bool geo, canreach;
    double lin, sma, apogee;
    // first test if the Satellite is geostationary
    if (fabs(tle->xno-1.0027)<0.0002)
        geo=1; else geo=0;

    // does the sat appear on our qth ?

    if(tle->xno==0.0)
    {
        canreach=0;
    }
    else
    {
        lin=tle->xincl;

        if (lin>=90.0)
            lin=180.0-lin;

        sma=331.25*exp(log(1440.0/tle->xno)*(2.0/3.0));
        apogee=sma*(1.0+tle->eo)-xkmper;

        if((acos(xkmper/(apogee+xkmper))+(lin*de2ra)) > fabs(obs->lat))
        {
            canreach = 1;
        }
        else
        {
            canreach = 0;
        }
    }
    if(!geo && canreach)
        return true;
    else
        return false;
}

double Satellite::findAOS(double daynum, bool hasAos)
{
    /* This function finds and returns the time of AOS (aostime). */
    double aostime=0.0;

    if(hasAos)
    {
        computeOrbit(daynum, false);

        /* Get the Satellite in range */

        while (mElevation < -1.0)
        {
            daynum -= 0.00035 * (mElevation * ((mHeight / 8400.0) + 0.46) - 2.0);
            computeOrbit(daynum, false);
        }

        /* Find AOS */
        while(aostime == 0.0)
        {
            if(fabs(mElevation) < 0.03)
            {
                aostime = daynum;
            }
            else
            {
                daynum -= mElevation*sqrt(mHeight)/530000.0;
                computeOrbit(daynum, false);
            }
        }
    }
    //  calc(_daynum, sat, false);
    return aostime;
}

double Satellite::findLOS(double daynum, bool hasLos)
{
    double lostime=0.0;
    if(hasLos)
    {
        computeOrbit(daynum, false);
        do
        {
            daynum += mElevation * sqrt(mHeight) / 502500.0;
            computeOrbit(daynum, false);

            if (fabs(mElevation) < 0.03)
                lostime=daynum;

        } while (lostime==0.0);
    }
    //  calc(_daynum, sat, false);
    return lostime;
}

double Satellite::findLOS2(double daynum, bool haslos)
{
    /* This function steps through the pass to find LOS.
     FindLOS() is called to "fine tune" and return the result. */
    do
    {
        daynum+=cos((mElevation-1.0)*de2ra)*sqrt(mHeight)/25000.0;
        computeOrbit(daynum, haslos);

    } while (mElevation>=0.0);

    return(findLOS(daynum, haslos));
}

double Satellite::nextAOS(double daynum, bool aoslos)
{
    // This function finds and returns the time of the next AOS for a Satellite that is currently in range.
    if(aoslos)
        daynum = findLOS2(daynum, aoslos) + 0.014;  /* Move to LOS + 20 minutes */

    return findAOS(daynum, aoslos);
}

double Satellite::nextAosLos(double daynum, bool aoslos)
{
    double result=0.0;
    if (mElevation < 0.03)
        result=findAOS(daynum, aoslos);
    else
        result=findLOS2(daynum, aoslos);
    return result;
}

QString Satellite::daynum2String(double daynum)
{
    if (daynum==0.0) return "";
    struct tm TM;
    bzero(&TM, sizeof(tm));
    Calendar_Date(daynum, &TM);
    Time_of_Day(daynum, &TM);
    TM.tm_year-=1900;
    TM.tm_mon-=1;
    time_t t = mktime(&TM);
    char* r = ctime(&t);
    r[strlen(r)-1]=0;
    return QString(r);
}

uint qHash(const Satellite &key, uint seed)
{
    return qHash(key.getName(), seed);
}

QColor Satellite::getTextColor() const
{
    if(mConstellation == Constellation::ConstellationGps)
    {
        return QColor(0,0,255);
    }
    else if(mConstellation == Constellation::ConstellationGlonass)
    {
        return QColor(255,0,0);
    }
    else if(mConstellation == Constellation::ConstellationGalileo)
    {
        return QColor(0,255,0);
    }
    else if(mConstellation == Constellation::ConstellationBeidou)
    {
        return QColor(255,255,0);
    }
}
