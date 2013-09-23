#include "gnssstatus.h"

QString GnssStatus::getIntegrationMode(const GnssStatus::IntegrationMode& integrationMode)
{
    // In my mind, INS+GNSS doesn't make sense (should be IMU+GNSS), but the SBF reference Guide 1.14.0 says so on pg. 73.
    switch(integrationMode)
    {
    case IntegrationMode::NoSolution: return "NoSolution"; break;
    case IntegrationMode::Loosely_INS: return "INS Loosely"; break;
    case IntegrationMode::Loosely_INS_and_GNSS: return "GNSS + INS Loosely"; break;
    case IntegrationMode::GNSS_only: return "GNSS Only"; break;
    default: return QString("Unknown IntMode %1").arg(static_cast<quint8>(integrationMode)); break;
    }
}

QString GnssStatus::getPvtMode(const GnssStatus::PvtMode& pvtMode)
{
    QString pvtModeString;

    switch(pvtMode)
    {
    case PvtMode::NoPVT: pvtModeString = "NoPVT"; break;

    case PvtMode::StandAlone: pvtModeString = "StandAlone"; break;

    case PvtMode::Differential: pvtModeString = "Differential"; break;

    case PvtMode::FixedLocation: pvtModeString = "FixedLocation"; break;

    case PvtMode::RtkFixed: pvtModeString = "RTK Fixed"; break;

    case PvtMode::RtkFloat: pvtModeString = "RTK Float"; break;

    case PvtMode::SbasAided: pvtModeString = "SBAS aided"; break;

    case PvtMode::RtkMovingBaseFixed: pvtModeString = "RTK MovingBase Fixed"; break;

    case PvtMode::RtkMovingBaseFloat: pvtModeString = "RTK MovingBase Float"; break;

    case PvtMode::PppFixed: pvtModeString = "PPP Fixed"; break;

    case PvtMode::PppFloat: pvtModeString = "PPP Float"; break;

    default:
        qWarning() << "GnssStatus::getPvtMode(): WARNING: unknown pvtmode" << static_cast<quint8>(pvtMode);
        pvtModeString = QString("Unknown GNSSPVTMode %1").arg(static_cast<quint8>(pvtMode));
        break;
    }

    return pvtModeString;
}


QString GnssStatus::getError(const GnssStatus::Error& error)
{
    switch(error)
    {
    case Error::NoError: return "No Error"; break;
    case Error::NotEnoughMeasurements: return "Not enough measurements"; break;
    case Error::NotEnoughEphemeridesAvailable: return "Not enough ephemerides available"; break;
    case Error::DopTooLarge: return "DOP too large"; break;
    case Error::SumOfSquaredResidualsTooLarge: return "Sum of squared residuals too large"; break;
    case Error::NoConvergence: return "No convergence"; break;
    case Error::NotEnoughMeasurementsAfterOutlierRejection: return "Not enough measurements after outlier rejection"; break;
    case Error::PositionOutputProhibitedDueToExportLaws: return "Position output prohibited due to export laws"; break;
    case Error::NotEnoughDifferentialCorrectionsAvailable: return "Not enough differential corrections available"; break;
    case Error::BasestationCoordinatesNotAvailable: return "Basestation coordinates not available"; break;
    case Error::AmbiquitiesNotFixedButOnlyRtkFixedAllowed: return "Floating ambiguities, but only RtkFixed allowed"; break;
    case Error::IntegratedPvNotRequestedByUser: return "Integrated PV not requested by user"; break;
    case Error::NotEnoughValidExtSensorValues: return "Not enough valid ext sensor values"; break;
    case Error::CalibrationNotReady: return "Calibration not ready"; break;
    case Error::StaticAlignmentOngoing: return "Static alignment ongoing"; break;
    case Error::WaitingForGnssPvt: return "Waiting for GNSS PVT"; break;
    case Error::WaitingForFineTime: return "Waiting for fine time"; break;
    case Error::InMotionAlignmentOngoing: return "InMotion alignment ongoing"; break;

    default: return QString("Unknown Error %1").arg(static_cast<quint8>(error)); break;
    }
}

QString GnssStatus::getInfo(const quint16& info)
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

QString GnssStatus::getInfoRichText(const quint16& info)
{
    QString infoString = QString("ACC%1 GYR%2 AMB%3 ZER%4 GPSP%5 GPSV%6 GPSA%7")
            .arg(testBit(info, 0) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 1) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 11) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 12) ? "1" : "0")
            .arg(testBit(info, 13) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 14) ? "1" : "<font color='red'>0</font>")
            .arg(testBit(info, 15) ? "<font color='red'>1</font>" : "0");

    return infoString;
}

QString GnssStatus::toString() const
{
    return QString("GnssMode %1, IntMode %2, Error %3, Info %4, NumSats %5, GnssAge %6, MeanCorrAge %7")
    .arg(static_cast<quint8>(pvtMode))
    .arg(static_cast<quint8>(integrationMode))
    .arg(static_cast<quint8>(error))
    .arg(info)
    .arg(numSatellitesUsed)
    .arg(gnssAge)
    .arg(meanCorrAge);
}

void GnssStatus::setPvtMode(const quint8 pvtModeCode)
{
    // the &15 bitmask ignores the bitfield inicating 2d and basestation operation modes
    pvtMode = static_cast<PvtMode>(pvtModeCode & 15);
}

void GnssStatus::setIntegrationMode(const quint8 integrationModeCode)
{
    integrationMode = static_cast<IntegrationMode>(integrationModeCode);
}

void GnssStatus::setError(const quint8 errorCode)
{
    error = static_cast<Error>(errorCode);
}

// for streaming
QDataStream& operator<<(QDataStream &out, const GnssStatus &status)
{
    out << static_cast<quint8>(status.pvtMode) << static_cast<quint8>(status.integrationMode) << static_cast<quint8>(status.error) << status.info << status.numSatellitesUsed << status.gnssAge << status.meanCorrAge << status.cpuLoad << status.covariances << status.longitude << status.latitude << status.height;
    return out;
}

QDataStream& operator>>(QDataStream &in, GnssStatus& status)
{
    quint8 intPvtMode;
    in >> intPvtMode;
    status.pvtMode = static_cast<GnssStatus::PvtMode>(intPvtMode);

    quint8 intIntegrationMode;
    in >> intIntegrationMode;
    status.integrationMode = static_cast<GnssStatus::IntegrationMode>(intIntegrationMode);

    quint8 intError;
    in >> intError;
    status.error = static_cast<GnssStatus::Error>(intError);

    in >> status.info;
    in >> status.numSatellitesUsed;
    in >> status.gnssAge;
    in >> status.meanCorrAge;
    in >> status.cpuLoad;
    in >> status.covariances;
    in >> status.longitude;
    in >> status.latitude;
    in >> status.height;

    return in;
}

QDebug operator<<(QDebug dbg, const GnssStatus::GnssSignalType &g)
{
    switch(g)
    {
    case GnssStatus::GpsL1Ca: dbg << "GpsL1Ca"; break;
    case GnssStatus::GpsL1Py: dbg << "GpsL1Py"; break;
    case GnssStatus::GpsL2Py: dbg << "GpsL2Py"; break;
    case GnssStatus::GpsL2C: dbg << "GpsL2C"; break;
    case GnssStatus::GpsL5: dbg << "GpsL5"; break;
    case GnssStatus::QzssL1Ca: dbg << "QzssL1Ca"; break;
    case GnssStatus::QzssL2C: dbg << "QzssL2C"; break;
    case GnssStatus::GloL1Ca: dbg << "GloL1Ca"; break;
    case GnssStatus::GloL2P: dbg << "GloL2P"; break;
    case GnssStatus::GloL2Ca: dbg << "GloL2Ca"; break;
    case GnssStatus::GloL3: dbg << "GloL3"; break;
    case GnssStatus::GalL1Bc: dbg << "GalL1Bc"; break;
    case GnssStatus::GalE5a: dbg << "GalE5a"; break;
    case GnssStatus::GalE5b: dbg << "GalE5b"; break;
    case GnssStatus::GalE5: dbg << "GalE5"; break;
    case GnssStatus::GeoL1Ca: dbg << "GeoL1Ca"; break;
    case GnssStatus::QzssL5: dbg << "QzssL5"; break;
    case GnssStatus::CompassL1: dbg << "CompassL1"; break;
    case GnssStatus::CompassE5b: dbg << "CompassE5b"; break;
    }

    return dbg;
}
