#ifndef GNSSSTATUS_H
#define GNSSSTATUS_H

#include <QDebug>
#include <QString>
#include <math.h>

#include <common.h>

class GnssStatus
{
public:
    enum struct PvtMode
    {
        NoPVT = 0,
        StandAlone,
        Differential,
        FixedLocation,
        RtkFixed,
        RtkFloat,
        SbasAided,
        RtkMovingBaseFixed,
        RtkMovingBaseFloat,
        PppFixed,
        PppFloat
    };

    enum struct IntegrationMode
    {
        NoSolution = 0,
        Loosely_INS = 1,
        Loosely_INS_and_GNSS = 2,
        GNSS_only = 4,
    };

    enum struct Error
    {
        NoError = 0,
            NotEnoughMeasurements = 1,
            NotEnoughEphemeridesAvailable = 2,
            DopTooLarge = 3,
        SumOfSquaredResidualsTooLarge = 4,
        NoConvergence = 5,
        NotEnoughMeasurementsAfterOutlierRejection = 6,
        PositionOutputProhibitedDueToExportLaws = 7,
            NotEnoughDifferentialCorrectionsAvailable = 8,
            BasestationCoordinatesNotAvailable = 9,
            AmbiquitiesNotFixedButOnlyRtkFixedAllowed = 10,
        IntegratedPvNotRequestedByUser = 20,
        NotEnoughValidExtSensorValues = 21,
        CalibrationNotReady = 22,
        StaticAlignmentOngoing = 23,
        WaitingForGnssPvt = 24,
        WaitingForFineTime = 27,
        InMotionAlignmentOngoing = 28
    };

    PvtMode pvtMode;
    IntegrationMode integrationMode;
    Error error;

    quint16 info;
    quint8 numSatellitesUsed;
    quint8 gnssAge; // septentrio is still unsure whether its milliseconds or seconds
    quint8 meanCorrAge; // tenths of a second, giving 0.0 - 25.5 seconds
    quint8 cpuLoad; // in percent
    float covariances;

    GnssStatus()
    {
        pvtMode = PvtMode::NoPVT;
        integrationMode = IntegrationMode::NoSolution;
        error = Error::WaitingForGnssPvt;
        cpuLoad = 0;
        info = 0;
        numSatellitesUsed = 0;
        gnssAge = 255;
        covariances = 10.0f;
    }

    bool operator!=(const GnssStatus& b)
    {
        return !(*(this) == b);
    }

    bool operator==(const GnssStatus& b)
    {
        return pvtMode == b.pvtMode
                && integrationMode == b.integrationMode
                && info == b.info
                && error == b.error
                && numSatellitesUsed == b.numSatellitesUsed
                && gnssAge == b.gnssAge
                && meanCorrAge == b.meanCorrAge
                && cpuLoad == b.cpuLoad
                && covariances == b.covariances;
    }

    bool interestingOrDifferentComparedTo(const GnssStatus& b)
    {
        return pvtMode != b.pvtMode
                || integrationMode != b.integrationMode
                //|| info != b.info // useless high-traffic babble
                || error != b.error
                || abs(numSatellitesUsed - b.numSatellitesUsed) > 1
                || abs(gnssAge - b.gnssAge) > 6
                || meanCorrAge > 30
                || cpuLoad > 80
                || abs(cpuLoad - b.cpuLoad) > 2
                || fabs(covariances - b.covariances) > 0.1;
    }

    void setPvtMode(const quint8 pvtModeCode);
    bool hasPvtMode(const quint8 pvtModeCode) {return static_cast<quint8>(pvtMode) == (pvtModeCode & 15); } // also contains bitfield which we ignore, see pg. 51 of SBF reference guide

    void setIntegrationMode(const quint8 integrationModeCode);
    bool hasIntegrationMode(const quint8 integrationModeCode) {return static_cast<quint8>(integrationMode) == integrationModeCode; }

    void setError(const quint8 errorCode);
    bool hasError(const quint8 errorCode) {return static_cast<quint8>(error) == errorCode; }

    QString toString() const;
    QString getPvtMode() const {return getPvtMode(pvtMode);}
    QString getIntegrationMode() const {return getIntegrationMode(integrationMode);}
    QString getError() const {return getError(error);}
    QString getInfo() const {return getInfo(info);}
    QString getInfoRichText() const {return getInfoRichText(info);}

    static QString getPvtMode(const PvtMode& pvtMode);
    static QString getPvtMode(const quint8& pvtMode) {return getPvtMode(static_cast<GnssStatus::PvtMode>(pvtMode));}

    static QString getIntegrationMode(const IntegrationMode& integrationMode);
    static QString getIntegrationMode(const quint8& integrationMode) {return getIntegrationMode(static_cast<GnssStatus::IntegrationMode>(integrationMode));}

    static QString getError(const Error& error);
    static QString getError(const quint8& error) {return getError(static_cast<GnssStatus::Error>(error));}

    static QString getInfo(const quint16& info);
    static QString getInfoRichText(const quint16& info);

};

// for streaming
QDataStream& operator<<(QDataStream &out, const GnssStatus &pose);
QDataStream& operator>>(QDataStream &in, GnssStatus &pose);

#endif // GNSSSTATUS_H
