#ifndef GPSSTATUSINFORMATION_H
#define GPSSTATUSINFORMATION_H

#include <QDebug>
#include <QString>

#include <common.h>

class GpsStatusInformation
{
//private:
//    GpsStatusInformation();

public:
    static QString getGnssMode(const quint8& gnssMode);
    static QString getIntegrationMode(const quint8& integrationMode);
    static QString getError(const quint8& error);
    static QString getInfo(const quint16& info);
    static QString getInfoRichText(const quint16& info);
};

#endif // GPSSTATUSINFORMATION_H
