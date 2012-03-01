#include "sbfparser.h"

SbfParser::SbfParser(QObject *parent) : QObject(parent)
{
    // Will be set on first PVT reception and is used to let the rover start at cartesian 0/0/0
    mOriginLongitude = 10e20;
    mOriginLatitude = 10e20;
    mOriginElevation = 10e20;

    mTimeStampStartup = QDateTime::currentDateTime();

    mPoseClockDivisor = 0;

    mMaxCovariances = 1.0f;
}

SbfParser::~SbfParser()
{
}

bool SbfParser::getNextValidPacketInfo(const QByteArray& sbfData, quint32* offset, qint32* tow)
{
    // We try to stay as close as possible to Septentrio's SBF reference guide pg. 13/14.
    Sbf_Header *sbfHeader;
    qint32 offsetToValidPacket = -2; // Set to -2, as we start searching from (offsetToValidPacket + sizeof(header.sync)), yielding a first try from 0.
    quint16 calculatedCrc;

    forever
    {
        // Look for a Sync-field ("$@") in the data, but start one byte after where we found a field the last time
        offsetToValidPacket = sbfData.indexOf("$@", (offsetToValidPacket + sizeof(sbfHeader->Sync)));

        // If the sync field "$@" was not found at all, no valid packet can be present. Quit.
        if(offsetToValidPacket < 0) return false;

        // Make sure that we have at least 8 bytes (the header size) to read (including the sync field)
        if(sbfData.size() < offsetToValidPacket + sizeof(Sbf_Header)) return false;

        sbfHeader = (Sbf_Header*)(sbfData.data() + offsetToValidPacket);

        // If sbfData doesn't hold enough bytes for an SBF block with the specified Length, it can have two reasons:
        //  a) the packet isn't received completely yet
        //  b) the packet's header->Length is corrupt
        // If it was a), we could return false, but we cannot be sure its not b), as we haven't checksummed yet. Thus,
        // instead of returning false, we need to look for further packets to guarantee working even in condition b)
        if(sbfHeader->Length > sbfData.size() - offsetToValidPacket) continue;

        // If the length is not a multiple of 4, its not a valid packet. Continue searching for another packet
        if(sbfHeader->Length % 4 != 0) continue;

        // Calculate the packet's checksum. For corrupt packets, the Length field might be random, so we bound
        // the bytes-to-be-checksummed to be between 0 and the buffer's remaining bytes after Sync and Crc fields.
        calculatedCrc = computeChecksum(
                    (void*)(sbfData.data() + offsetToValidPacket + sizeof(sbfHeader->Sync) + sizeof(sbfHeader->CRC)),
                    qBound(
                        (qint32)0,
                        (qint32)(sbfHeader->Length - sizeof(sbfHeader->Sync) - sizeof(sbfHeader->CRC)),
                        (qint32)(sbfData.size() - offsetToValidPacket - sizeof(sbfHeader->Sync) - sizeof(sbfHeader->CRC))
                        )
                    );

        // Quit searching if we found a valid packet.
        if(sbfHeader->CRC == calculatedCrc) break;
    }

    // If we're here, we have a valid SBF packet starting at offsetToValidPacket
    if(offset) *offset = offsetToValidPacket;

    const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)(sbfData.data()+offsetToValidPacket);
    if(tow) *tow = block->TOW;

    return true;
}

void SbfParser::slotEmitCurrentGpsStatus()
{
    emit status(mGpsStatus);
}

quint16 SbfParser::computeChecksum(const void *buf, unsigned int length) const
{
  quint32  i;
  quint16  crc = 0;
  const quint8  *buf8 = (quint8*)buf; /* Convert the type to access by byte. */

  /* see for example the BINEX web site */
  for (i=0; i < length; i++)
  {
    crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buf8[i] ];
  }

  return crc;
}

void SbfParser::setPose(
    const qint32& lon,
    const qint32& lat,
    const qint32& alt,
    const quint16& heading,
    const qint16& pitch,
    const qint16& roll,
    const quint32& tow,
    const quint8& precision)
{
    const double floatLon = ((double)lon) / 10000000.0l;
    const double floatLat = ((double)lat) / 10000000.0l;
    const double floatAlt = ((double)alt) / 1000.0l;

    // Please look at the septentrio "Firmware user manual", page 47ff for conversion rules.
    mLastPose = Pose(
                convertGeodeticToCartesian(floatLon, floatLat, floatAlt, precision),
                -((double)heading) * 0.01l, // Z axis points down in VehicleReferenceFrame, values from 0 to 360, both point north
                ((double)pitch) * 0.01l,
                -((double)roll) * 0.01l, // their roll axis points forward from the vehicle. We have it OpenGL-style with Z pointing backwards
                (qint32)tow // Receiver time in milliseconds. WARNING: be afraid of WNc rollovers at runtime!
                );
}

QVector3D SbfParser::convertGeodeticToCartesian(const double &lon, const double &lat, const double &elevation, const quint8& precision)
{
    // Set longitude, latitude and elevation of first GNSS fix to let the rover start at cartesian 0/0/0
    // This may ONLY happen based on really good positional fixes (=RTK Fixed): if we initialize these
    // origins using e.g. Differential and then switch to RTKFixed, the kopter might remain underground
    // due to a -2m Y-offset between Differential and RtkFixed.
    if(mOriginLongitude > 10e19 && mOriginLatitude > 10e19 && mOriginElevation > 10e19 && precision & Pose::RtkFixed)
    {
        mOriginLongitude = lon;
        mOriginLatitude = lat;
        mOriginElevation = elevation;
    }

    // Doesn't matter, as offset isn't hardcoded anymore this is just for historical reference :)
    // FBI in Hamburg is 53.600515,09.931478 with elevation of about 70m
    // PPM in Penzberg is 47.757201,11.377133 with elevation of about 656m

    QVector3D co;

    co.setY(elevation - mOriginElevation);
    co.setZ(-(lat - mOriginLatitude) * 111300.0l);
    co.setX((lon -  mOriginLongitude) * 111300.0l * cos(DEG2RAD(mOriginLatitude)));

    return co;
}

void SbfParser::processNextValidPacket(QByteArray& sbfData)
{
    //qDebug() << "SbfParser::processNextValidPacket():" << sbfData.size() << "bytes present.";

    quint32 offsetToValidPacket;

    if(getNextValidPacketInfo(sbfData, &offsetToValidPacket) == false)
    {
        qDebug() << "SbfParser::processNextValidPacket(): no more valid packets in sbf data of size" << sbfData.size();
        return;
    }

    // If there's garbage before the next valid packet, save it into sbf log and log a warning
    if(offsetToValidPacket != 0)
    {
        qWarning() << "SbfParser::processNextValidPacket(): WARNING: SBF Sync Marker $@ was not at byte 0, but at" << offsetToValidPacket;
        // Log this data for later error analysis
        emit processedPacket(sbfData.left(offsetToValidPacket));
        sbfData.remove(0, offsetToValidPacket);
    }

    const quint16 msgCrc = *(quint16*)(sbfData.data() + 2);
    const quint16 msgId = *(quint16*)(sbfData.data() + 4);
    const quint16 msgIdBlock = msgId & 0x1fff;
    const quint16 msgIdRev = msgId >> 13;
    const quint16 msgLength = *(quint16*)(sbfData.data() + 6);

    // Save our current gpsStatus in a const place, so we can check whether it changed after processing the whole packet
    const GpsStatusInformation::GpsStatus previousGpsStatus = mGpsStatus;

//    qDebug() << "SbfParser::processNextValidPacket(): processing" << sbfData.size() << "bytes SBF data with ID" << msgId << "from TOW" << ((Sbf_PVTCartesian*)sbfData.data())->TOW;

    // Process the message if we're interested.
    //qDebug() << "received sbf block" << msgIdBlock;
    switch(msgIdBlock)
    {

    case 4006:
    {
        // PVTCartesian
        const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)sbfData.data();
        // block->MeanCorrAge is quint16 in hundreds of a second
//        qDebug() << "SBF: PVTCartesian: MeanCorrAge in seconds:" << ((float)block->MeanCorrAge)/100.0;
        mGpsStatus.meanCorrAge = std::min(block->MeanCorrAge / 10, 255);
    }
    break;

    case 4072:
    {
        // IntAttCovEuler
        const Sbf_IntAttCovEuler *block = (Sbf_IntAttCovEuler*)sbfData.data();
//        qDebug() << "SBF: IntAttCovEuler: covariances for heading, pitch, roll:" << block->Cov_HeadHead << block->Cov_PitchPitch << block->Cov_RollRoll;
        float newCovarianceValue = std::max(std::max(block->Cov_HeadHead, block->Cov_PitchPitch), block->Cov_RollRoll);
        if(fabs(mGpsStatus.covariances - newCovarianceValue) > 0.02)
        {
            mGpsStatus.covariances = newCovarianceValue;
//            emit status(mGpsStatus);
        }
    }
    break;

    case 4014:
    {
        // ReceiverStatus
        const Sbf_ReceiverStatus *block = (Sbf_ReceiverStatus*)sbfData.data();

//        qDebug() << "SBF: ReceiverStatus: CPU Load:" << block->CPULoad;

        mGpsStatus.cpuLoad = block->CPULoad;

        if(block->CPULoad > 80)
        {
            qWarning() << "SbfParser::processNextValidPacket(): WARNING, receiver CPU load is" << block->CPULoad;
            emit message(Warning,
                         QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                         QString("Warning, CPU load is too high (%1 percent)").arg(block->CPULoad));
        }

        if(block->ExtError != 0)
        {
            qWarning() << "SbfParser::processNextValidPacket(): ExtError is not 0 but" << block->ExtError;
            emit message(
                        Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Warning, ExtError is not zero (%1)").arg(block->ExtError));

            // According to SBF guide pg. 101, this means diff corr data error. Lets see what it is.
            if(block->ExtError == 2) emit receiverCommand("lif,DiffCorrError");
        }

        if(block->RxError != 0)
        {
            qWarning() << "SbfParser::processNextValidPacket(): RxError is not 0 but" << block->RxError;
            emit message(
                        Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Warning, RxError is not zero (%1)").arg(block->RxError));

            // According to SBF guide pg. 102, this means software warning or error. Lets see what it is.
            if(block->RxError == 8) receiverCommand("lif,error");
        }
    }
    break;

    case 4045:
    {
        // IntPVAAGeod
        const Sbf_PVAAGeod *block = (Sbf_PVAAGeod*)sbfData.data();

//        qDebug() << "SBF: IntPVAAGeod";

        // Check the Info-field and emit states if it changes
        if(mGpsStatus.info != block->Info)
        {
            //qDebug() << t() << "SbfParser::processNextValidPacket(): info changed from" << mGpsStatus.info << "to" << block->Info;

            if(!testBitEqual(mGpsStatus.info, block->Info, 0))
                emit message(
                            testBit(block->Info, 0) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("ACLR measurements used: %1").arg(testBit(block->Info, 0) ? "true" : "false"));

            if(!testBitEqual(mGpsStatus.info, block->Info, 1))
                emit message(
                            testBit(block->Info, 1) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GYRO measurements used: %1").arg(testBit(block->Info, 1) ? "true" : "false"));

            if(!testBitEqual(mGpsStatus.info, block->Info, 11))
                emit message(
                            testBit(block->Info, 11) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Heading ambiguity fixed: %1").arg(testBit(block->Info, 11) ? "true" : "false"));

            if(false && !testBitEqual(mGpsStatus.info, block->Info, 12)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 12) ? Error : Information, // We don't use this, should be 0
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Zero constraint used: %1").arg(testBit(block->Info, 12) ? "true" : "false"));

            if(false && !testBitEqual(mGpsStatus.info, block->Info, 13)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 13) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS position used: %1").arg(testBit(block->Info, 13) ? "true" : "false"));

            if(false && !testBitEqual(mGpsStatus.info, block->Info, 14)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 14) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS velocity used: %1").arg(testBit(block->Info, 14) ? "true" : "false"));

            if(!testBitEqual(mGpsStatus.info, block->Info, 15))
                emit message(
                            testBit(block->Info, 15) ? Information : Error, // Not sure whether GNSS attitude means multi-antenna
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS attitude used: %1").arg(testBit(block->Info, 15) ? "true" : "false"));

            // If mode was 26627 or 30723, that would be 11X100000000011
            if(block->Info == 26627 || block->Info == 30723) Q_ASSERT("Whee, GPS INFO is the way it should be!");

            mGpsStatus.info = block->Info;
        }

        // Check the Mode-field and emit states if it changes
        if(mGpsStatus.integrationMode != block->Mode                     && block->Mode == 2) // FIXME: until firmware works?!
        {
            //qDebug() << t() << "SbfParser::processNextValidPacket(): mode changed from" << mGpsStatus.integrationMode << "to" << block->Mode;

            switch(block->Mode)
            {
            case 0:
                emit message(
                            Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Mode changed, no integrated solution available"));
                break;

            case 1:
                emit message(
                            Warning,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Mode changed, using only external sensor"));
                break;

            case 2:
                emit message(
                            Information,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            ("Mode changed, using integrated solution"));
                break;

            default:
                qWarning() << "SbfParser::processNextValidPacket(): WARNING: unknown mode code" << block->Mode << "at TOW" << block->TOW;
                emit message(
                            Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Unknown IntegrationMode %1 at TOW %2").arg(block->Mode).arg(block->TOW));
                break;
            }

            mGpsStatus.integrationMode = block->Mode;
        }

        // Check the Error-field and emit states if it changes
        if(mGpsStatus.error != block->Error)
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): error changed from" << mGpsStatus.error << "to" << block->Error << "at TOW" << block->TOW;

            emit message(
                        block->Error == 0 ? Information : Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Error changed from %1 (%2) to %3 (%4)")
                        .arg(mGpsStatus.error)
                        .arg(GpsStatusInformation::getError(mGpsStatus.error))
                        .arg(block->Error)
                        .arg(GpsStatusInformation::getError(block->Error)));

            mGpsStatus.error = block->Error;
        }

        // Check the GnssPvtMode-field and emit states if it changes AND if its not DO-NOT-USE
        if(mGpsStatus.gnssMode != block->GNSSPVTMode && block->GNSSPVTMode != 255)
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): GnssPvtMode changed from" << mGpsStatus.gnssMode << "to" << block->GNSSPVTMode << "at TOW" << block->TOW;

            emit message(
                        (block->GNSSPVTMode & 15) == 4 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GnssPvtMode changed: %1 => %2")
                        .arg(GpsStatusInformation::getGnssMode(mGpsStatus.gnssMode))
                        .arg(GpsStatusInformation::getGnssMode(block->GNSSPVTMode)));

            mGpsStatus.gnssMode = block->GNSSPVTMode;
        }

        if(mGpsStatus.gnssAge != block->GNSSage             && block->GNSSage == 0) // FIXME: until firmware works?!
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): GnssAge changed from" << mGpsStatus.gnssAge << "to" << block->GNSSage << "at TOW" << block->TOW;

            emit message(
                        block->GNSSage > 0 ? Information : Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("No GNSS PVT for %1 seconds").arg(block->GNSSage));

            mGpsStatus.gnssAge = block->GNSSage;
        }

        const quint8 numberOfSatellitesUsed = (block->NrSVAnt & 31);
        if(mGpsStatus.numSatellitesUsed != numberOfSatellitesUsed)
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): numSats changed from" << mGpsStatus.numSatellitesUsed << "to" << numberOfSatellitesUsed;
            /*emit message(
                        numberOfSatellitesUsed > 5 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Number of used satellites changed from %1 to %2").arg(mGpsStatus.numSatellitesUsed).arg(numberOfSatellitesUsed));*/

            mGpsStatus.numSatellitesUsed = numberOfSatellitesUsed;
        }

        // Only emit a pose if the values are not set to the do-not-use values.
        if(
                block->Error == 0
                && block->TOW != 4294967295L
                && block->Lat != -2147483648L
                && block->Lon != -2147483648L
                && block->Alt != -2147483648L)
        {

            quint8 precisionFlags = 0;

            if(block->Heading != 65535 && block->Pitch != -32768 && block->Roll != -32768)
                precisionFlags |= Pose::AttitudeAvailable;
//            else
//                qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, YPR do-not-use";

            if(testBit(block->Info, 11)) // Heading ambiguity is Fixed
                precisionFlags |= Pose::HeadingFixed;
//            else
//                qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): pose from PVAAGeod not valid, heading ambiguity is not fixed.";

            if(block->Mode == 2) // integrated solution, not sensor-only or GNSS-only
                precisionFlags |= Pose::ModeIntegrated;
//            else
//                qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, not integrated solution, but" << GpsStatusInformation::getIntegrationMode(block->Mode);

            if((block->GNSSPVTMode & 15) == 4) // Thats RTK Fixed, see GpsStatusInformation::getGnssMode().
                precisionFlags |= Pose::RtkFixed;
//            else
//                qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, GnssPvtMode is" << GpsStatusInformation::getGnssMode(block->GNSSPVTMode) << "corrAge:" << mGpsStatus.meanCorrAge << "sec";

            if(mGpsStatus.meanCorrAge < 40) // thats four seconds
                precisionFlags |= Pose::CorrectionAgeLow;

            /* Not an error! Wait until firmware is fixed?!
            if(block->GNSSage > 1)
            {
                qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, GNSSAge is" << block->GNSSage;
            }*/

            setPose(block->Lon, block->Lat, block->Alt, block->Heading, block->Pitch, block->Roll, block->TOW, precisionFlags);
            mLastPose.precision = precisionFlags;
            mLastPose.covariances = mGpsStatus.covariances;

            emit newVehiclePose(mLastPose);
            mPoseClockDivisor++;
        }
        else if(block->Error != 0)
        {
            qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, error:" << block->Error << "" << GpsStatusInformation::getError(block->Error) ;
        }
        else
        {
            qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, do-not-use values found in elementary fields.";
        }

        // If the last pose is valid (i.e. not default-constructed), emit it now.
        // On the first time, this will start the laserscanner.
        if(mPoseClockDivisor % 25 == 0 && mLastPose.timestamp != 0)
            emit newVehiclePoseLowFreq(mLastPose);
    }
    break;

    case 5914:
    {
        // ReceiverTime
        const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)sbfData.data();

        qDebug() << "SBF: ReceiverTime: TOW:" << block->TOW;

        //qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): received ReceiverTime block: msgid" << msgId << "msgIdBlock" << msgIdBlock << "msgLength" << msgLength << "revision" << msgIdRev;

        if(block->TOW == 4294967295L)
        {
            emit message(
                        Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GPS Receiver TOW is at its do-not-use-value, give it time to initialize."));

            qWarning() << "SbfParser::processNextValidPacket(): GPS Receiver TOW is at its do-not-use-value, give it time to initialize.";

            if(mTimeStampStartup.secsTo(QDateTime::currentDateTime()) < 15)
            {
                qWarning() << "SbfParser::processNextValidPacket(): GPS Receiver TOW is at its do-not-use-value during startup - quitting.";
                QCoreApplication::quit();
            }
        }
        else
        {
            // Set system time to gps time. Adding a roundtrip-timer is not a good idea, as the board waits until the
            // second leaps, meaning the time from request to output doesn't equal the time from output to reception.
            emit gpsTimeOfWeekEstablished((qint32)block->TOW);
        }
    }
    break;

    case 5924:
    {
        // ExtEvent
        qDebug() << "SBF: ExtEvent";
        const Sbf_ExtEvent *block = (Sbf_ExtEvent*)sbfData.data();

        // Laserscanner sync signal is soldered to both ports, but port 1 is broken. If it ever starts working again, I want to know.
        Q_ASSERT(block->Source == 2);

        if(block->TOW != 4294967295L)
        {
            // Emit the time of the scan. The Scanner sets the pulse at the END of a scan,
            // but our convention is to use times of a scans middle. Thus, decrement 12ms.
            //qDebug() << "SbfParser::processNextValidPacket(): emitting scanFinished with a scanTimeGps of" << block->TOW - 12;
            emit scanFinished(block->TOW - 12);
        }
        else
            qDebug() << "SbfParser::processNextValidPacket(): WARNING: scan finished, but TOW is set to do-not-use!";
    }
    break;
    case 4037:
    {
        // ExtEventPvtCartesian
        qDebug() << "SBF: ExtEventPvtCartesian";
    }
    break;
    case 4050:
    {
        // ExtSensorMeas
//        qDebug() << "SBF: ExtSensorMeas";
    }
    break;
    default:
    {
//        qDebug() << "SbfParser::processNextValidPacket(): ignoring block id" << msgIdBlock;
    }
    }

    // emit new status if it changed significantly.
    if(mGpsStatus.interestingOrDifferentComparedTo(previousGpsStatus)) emit status(mGpsStatus);

    // Announce what packet we just processed. Might be used for logging.
    emit processedPacket(sbfData.left(msgLength));

    // Remove the SBF block body from our incoming buffer, so it contains either nothing or the next SBF message
    const int indexOfNextPacket = sbfData.indexOf("$@", offsetToValidPacket+1);
    if(indexOfNextPacket == -1)
    {
        // There is no next packet, so remove the packet, possibly without the padding bytes at the end.
        sbfData.remove(0, msgLength); // this does not remove the padding bytes after the current packet
    }
    else
    {
        // A next packet was found, so remove everything up to its start (including padding bytes from our current packet)
        sbfData.remove(0, indexOfNextPacket);
    }

    //if(sbfData.size()) qDebug() << "SbfParser::processNextValidPacket(): done processing SBF data, bytes left in buffer:" << sbfData.size() << "bytes:" << sbfData;
}
