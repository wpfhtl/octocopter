#ifndef GPSSTATUSINFORMATION_H
#define GPSSTATUSINFORMATION_H

#include <QDebug>
#include <QString>
#include <math.h>

#include <common.h>

class GpsStatusInformation
{
//private:
//    GpsStatusInformation();

public:
    struct GpsStatus
    {
        quint8 gnssMode;
        quint8 integrationMode;
        quint16 info;
        quint8 error;
        quint8 numSatellitesUsed;
        quint8 gnssAge; // septentrio is still unsure whether its milliseconds or seconds
        quint8 meanCorrAge; // tenths of a second, giving 0.0 - 25.5 seconds
        quint8 cpuLoad; // in percent
        float covariances;

        GpsStatus()
        {
            error = 255;
            integrationMode = 255;
            cpuLoad = 0;
            info = 0;
            gnssMode = 0;
            numSatellitesUsed = 0;
            gnssAge = 255;
            covariances = 10.0f;
        }

        bool operator!=(const GpsStatus& b)
        {
            return !(*(this) == b);
        }

        bool operator==(const GpsStatus& b)
        {
            return gnssMode == b.gnssMode
                    && integrationMode == b.integrationMode
                    && info == b.info
                    && error == b.error
                    && numSatellitesUsed == b.numSatellitesUsed
                    && gnssAge == b.gnssAge
                    && meanCorrAge == b.meanCorrAge
                    && cpuLoad == b.cpuLoad
                    && covariances == b.covariances;
        }

        bool interestingOrDifferentComparedTo(const GpsStatus& b)
        {
            return gnssMode != b.gnssMode
                    || integrationMode != b.integrationMode
                    || info != b.info
                    || error != b.error
                    || abs(numSatellitesUsed - b.numSatellitesUsed) > 1
                    || abs(gnssAge - b.gnssAge) > 6
                    || meanCorrAge > 30
                    || cpuLoad > 80
                    || abs(cpuLoad - b.cpuLoad) > 1
                    || fabs(covariances - b.covariances) > 0.1;
        }

    };

    static QString getStatusText(const GpsStatusInformation::GpsStatus& status);
    static QString getGnssMode(const quint8& gnssMode);
    static QString getIntegrationMode(const quint8& integrationMode);
    static QString getError(const quint8& error);
    static QString getInfo(const quint16& info);
    static QString getInfoRichText(const quint16& info);
};

// for streaming
QDataStream& operator<<(QDataStream &out, const GpsStatusInformation::GpsStatus &pose);
QDataStream& operator>>(QDataStream &in, GpsStatusInformation::GpsStatus &pose);

#endif // GPSSTATUSINFORMATION_H
