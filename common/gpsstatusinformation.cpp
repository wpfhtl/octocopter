#include "gpsstatusinformation.h"

//GpsStatusInformation::GpsStatusInformation()
//{
//}

QString GpsStatusInformation::getIntegrationMode(const quint8& integrationMode)
{
    switch(integrationMode)
    {
    case 0: return "Integrated PV Unavailable"; break;
    case 1: return "IMU only"; break;
    case 2: return "GPS + IMU"; break;
    default: return QString("Unknown IntMode %1").arg(integrationMode); break;
    }
}

QString GpsStatusInformation::getGnssMode(const quint8& gnssMode)
{
    QString gnssModeString;

    // the mode is in bits 0-3
    const quint8 gnssPvtMode = (gnssMode & 15);

    switch(gnssPvtMode)
    {
    case 0: gnssModeString = "Error"; break;

    case 1: gnssModeString = "StandAlone"; break;

    case 2: gnssModeString = "Differential"; break;

    case 3: gnssModeString = "FixedLocation"; break;

    case 4: gnssModeString = "RTK Fixed"; break;

    case 5: gnssModeString = "RTK Float"; break;

    case 6: gnssModeString = "SBAS aided"; break;

    case 7: gnssModeString = "RTK MovingBase Fixed"; break;

    case 8: gnssModeString = "RTK MovingBase Float"; break;

    case 9: gnssModeString = "PPP Fixed"; break;

    case 10: gnssModeString = "PPP Float"; break;

    default:
        qWarning() << "GpsStatusInformation::getGnssMode(): WARNING: unknown GNSSPVTMode code" << gnssPvtMode << "gnssmode" << gnssMode;
        gnssModeString = QString("Unknown GNSSPVTMode %1").arg(gnssPvtMode);
        break;
    }

    if(testBit(gnssMode, 6)) gnssModeString.prepend("Base, acquiring position, ");
    if(testBit(gnssMode, 7)) gnssModeString.append(" (2D)");

    return gnssModeString;
}

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
    case 21: return "Not enough valid ext sensor values"; break;
    case 22: return "Calibration not ready"; break;
    case 23: return "Alignment not ready"; break;
    case 24: return "Waiting for GNSS PVT"; break;
    default: return QString("Unknown Error %1").arg(error); break;
    }
}

QString GpsStatusInformation::getInfo(const quint16& info)
{
    QString infoString = QString("ACC%1 GYR%2 AMB%3 ZER%4 GPSP%5 GPSV%6 GPSA%7")
            .arg(testBit(info, 0) ? 1 : 0)
            .arg(testBit(info, 1) ? 1 : 0)
            .arg(testBit(info, 11) ? 1 : 0)
            .arg(testBit(info, 12) ? 1 : 0)
            .arg(testBit(info, 13) ? 1 : 0)
            .arg(testBit(info, 14) ? 1 : 0)
            .arg(testBit(info, 15) ? 1 : 0);

    return infoString;
}

QString GpsStatusInformation::getInfoRichText(const quint16& info)
{
    QString infoString = QString("ACC%1 GYR%2 AMB%3 ZER%4 GPSP%5 GPSV%6 GPSA%7")
            .arg(testBit(info, 0) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 1) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 11) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 12) ? "1" : "0")
            .arg(testBit(info, 13) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 14) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 15) ? "1" : "<font color='red'>0</font>");

    return infoString;
}

QString GpsStatusInformation::getStatusText(const GpsStatusInformation::GpsStatus& status)
{
    return QString("GnssMode %1, IntMode %2, Info %3, Error %4, NumSats %5, GnssAge %6, MeanCorrAge %7")
    .arg(status.gnssMode)
    .arg(status.integrationMode)
    .arg(status.info)
    .arg(status.error)
    .arg(status.numSatellitesUsed)
    .arg(status.lastPvtAge)
    .arg(status.meanCorrAge);
}

// for streaming
QDataStream& operator<<(QDataStream &out, const GpsStatusInformation::GpsStatus &status)
{
    out << status.gnssMode << status.integrationMode << status.info << status.error << status.numSatellitesUsed << status.lastPvtAge << status.meanCorrAge << status.cpuLoad << status.covariances;
    return out;
}

QDataStream& operator>>(QDataStream &in, GpsStatusInformation::GpsStatus& status)
{
    in >> status.gnssMode;
    in >> status.integrationMode;
    in >> status.info;
    in >> status.error;
    in >> status.numSatellitesUsed;
    in >> status.lastPvtAge;
    in >> status.meanCorrAge;
    in >> status.cpuLoad;
    in >> status.covariances;

    return in;
}
