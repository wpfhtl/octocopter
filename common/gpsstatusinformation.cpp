#include "gpsstatusinformation.h"

GpsStatusInformation::GpsStatusInformation()
{
}

QString GpsStatusInformation::getIntegrationMode(const quint8& integrationMode)
{
    switch(integrationMode)
    {
    case 0: return "Unavailable"; break;
    case 1: return "IMU only"; break;
    case 2: return "GPS + IMU"; break;
    default: return QString("Unknown IntMode %1").arg(integrationMode); break;
    }
};

QString GpsStatusInformation::getGnssMode(const quint8& gnssMode)
{
    QString gnssModeString;

    // the mode is in bits 0-3
    const quint8 gnssPvtMode = gnssMode & 15;

    switch(gnssPvtMode)
    {
    case 0: gnssModeString = "Error"; break;

    case 1: gnssModeString = "StandAlone"; break;

    case 2: gnssModeString = "Differential"; break;

    case 3: gnssModeString = "FixedLocation"; break;

    case 4: gnssModeString = "RTK FixedAmbiguities"; break;

    case 5: gnssModeString = "RTK FloatAmbiguities"; break;

    case 6: gnssModeString = "SBAS aided"; break;

    case 7: gnssModeString = "RTK MovingBase FixedAmbiguities"; break;

    case 8: gnssModeString = "RTK MovingBase FloatAmbiguities"; break;

    case 9: gnssModeString = "PPP FixedAmbiguities"; break;

    case 10: gnssModeString = "PPP FloatAmbiguities"; break;

    default:
        qWarning() << "GpsStatusInformation::getGnssMode(): WARNING: unknown GNSSPVTMode code" << gnssPvtMode;
        gnssModeString = QString("Unknown GNSSPVTMode %1").arg(gnssPvtMode);
        break;
    }

    if(gnssMode & 64 != 0) gnssModeString.prepend("Base, acquiring position, ");
    if(gnssMode & 128 != 0) gnssModeString.append(", 2D mode");

    return gnssModeString;
};

QString GpsStatusInformation::getError(const quint8& error)
{
    switch(error)
    {
    case 0: return "No Error"; break;
    case 1: return "Not enough measurements"; break;
    case 2: return "Not enough ephemerides available"; break;
    case 3: return "DOP too large (>15)"; break;
    case 4: return "Sum of squared residuals too large"; break;
    case 5: return "No convergence"; break;
    case 6: return "Not enough measurements after outlier rejection"; break;
    case 7: return "Position output prohibited due to export laws"; break;
    case 8: return "Not enough differential corrections available"; break;
    case 9: return "Basestation coordinates not available"; break;
    case 20: return "Integrated PV not requested by user"; break;
    case 21: return "Not enough valid external sensor measurements"; break;
    case 22: return "Calibration not ready"; break;
    case 23: return "Alignment not ready"; break;
    case 24: return "Waiting for GNSS PVT"; break;
    default: return QString("Unknown Error %1").arg(error); break;
    }
};

QString GpsStatusInformation::getInfo(const quint16& info)
{
    QString infoString = QString("ACC%1 GYR%2 AMB%3 ZER%4 GPSP%5 GPSV%6 GPSA%7")
            .arg(info & 1 != 0 ? 1 : 0)
            .arg(info & 2 != 0 ? 1 : 0)
            .arg(info & 2048 != 0 ? 1 : 0)
            .arg(info & 4096 != 0 ? 1 : 0)
            .arg(info & 8192 != 0 ? 1 : 0)
            .arg(info & 16384 != 0 ? 1 : 0)
            .arg(info & 32768 != 0 ? 1 : 0);

    return infoString;
};
