#include "sbfparser.h"

SbfParser::SbfParser(QObject *parent) : QObject(parent)
{
    // Will be set on first PVT reception and is used to let the rover start at cartesian 0/0/0
    mOriginLongitude = 10e20;
    mOriginLatitude = 10e20;
    mOriginElevation = 10e20;

    mPacketErrorCount = 0;

    mMaxCovariances = 1.0f;

    mGnssDeviceWorkingPrecisely = false;

    // Offset from ARP to vehicle center in meters, local vehicle coordinate system
    mTransformArpToVehicle.translate(0.09f, -0.53f, -0.04f);
}

SbfParser::~SbfParser()
{
}

bool SbfParser::getNextValidPacketInfo(const QByteArray& sbfData, const quint32& sbfDataStart, quint32* offsetToValidPacket, qint32* tow)
{
    // We try to stay as close as possible to Septentrio's SBF reference guide pg. 13/14.
//    Sbf_Header *sbfHeader;
    Sbf_PVTCartesian *block;
    qint32 offsetToValidPacketLocal = sbfDataStart - 2; // Set to -2, as we start searching from (offsetToValidPacket + sizeof(header.sync)), yielding a first try from 0.
    quint16 calculatedCrc;

    forever
    {
        // Look for a Sync-field ("$@") in the data, but start one byte after where we found a field the last time
        offsetToValidPacketLocal = sbfData.indexOf("$@", (offsetToValidPacketLocal + sizeof(block->Header.Sync)));

        // If the sync field "$@" was not found at all, no valid packet can be present. Quit.
        if(offsetToValidPacketLocal < 0)
        {
//            qDebug() << "SbfParser::getNextValidPacketInfo(): offsetToValidPacket" << offsetToValidPacket << "no $@ found, returning false";
            return false;
        }

        // Make sure that we have at least 8 bytes (the header size) to read (including the sync field)
        if(sbfData.size() < offsetToValidPacketLocal + sizeof(Sbf_Header))
        {
//            qDebug() << "SbfParser::getNextValidPacketInfo(): offsetToValidPacket" << offsetToValidPacket << ", $@ found, but not enough data (" << sbfData.size() << "bytes), returning false";
            return false;
        }

//        sbfHeader = (Sbf_Header*)(sbfData.constData() + offsetToValidPacket);
        block = (Sbf_PVTCartesian*)(sbfData.constData() + offsetToValidPacketLocal);

        // If sbfData doesn't hold enough bytes for an SBF block with the specified Length, it can have two reasons:
        //  a) the packet isn't received completely yet
        //  b) the packet's header->Length is corrupt
        // If it was a), we could return false, but we cannot be sure its not b), as we haven't checksummed yet. Thus,
        // instead of returning false, we need to look for further packets to guarantee working even in condition b)
        if(block->Header.Length > sbfData.size() - offsetToValidPacketLocal)
        {
//            qDebug() << "SbfParser::getNextValidPacketInfo(): offsetToValidPacket" << offsetToValidPacket << "$@ found, header present, packetLength" << block->Header.Length << "," << sbfData.size() << "bytes present, returning false";
            continue;
        }

        // If the length is not a multiple of 4, its not a valid packet. Continue searching for another packet
        if(block->Header.Length % 4 != 0)
        {
//            qDebug() << "SbfParser::getNextValidPacketInfo(): offsetToValidPacket" << offsetToValidPacket << "$@ found, header present, packetLength" << block->Header.Length << "is not a multiple of 4, returning false";
            continue;
        }

        // If the packet has a TOW DO-NOT-USE, skip it. TODO: Really?
//        if(block->TOW == 4294967295)
//        {
//            qDebug() << "SbfParser::getNextValidPacketInfo(): offsetToValidPacket" << offsetToValidPacket << "$@ found, header present, TOW is DONOTUSE.";
//            continue;
//        }

        // Calculate the packet's checksum. For corrupt packets, the Length field might be random, so we bound
        // the bytes-to-be-checksummed to be between 0 and the buffer's remaining bytes after Sync and Crc fields.
        calculatedCrc = computeChecksum(
                    (void*)(sbfData.constData() + offsetToValidPacketLocal + sizeof(block->Header.Sync) + sizeof(block->Header.CRC)),
                    qBound(
                        (qint32)0,
                        (qint32)(block->Header.Length - sizeof(block->Header.Sync) - sizeof(block->Header.CRC)),
                        (qint32)(sbfData.size() - offsetToValidPacketLocal - sizeof(block->Header.Sync) - sizeof(block->Header.CRC))
                        )
                    );

        // Quit searching if we found a valid packet.
        if(block->Header.CRC == calculatedCrc)
        {
            break;
        }
        else
        {
            mPacketErrorCount++;
//            qDebug() << "SbfParser::getNextValidPacketInfo(): packet at offset" << offsetToValidPacket << "has CRC error ("<< mPacketErrorCount <<"):" << block->Header.CRC << calculatedCrc << ", will start searching two bytes later";
        }
    }

    // If we're here, we have a valid SBF packet starting at offsetToValidPacket
    if(offsetToValidPacket) *offsetToValidPacket = offsetToValidPacketLocal;

    if(tow) *tow = block->TOW;

    return true;
}

void SbfParser::slotEmitCurrentGnssStatus()
{
    emit status(&mGnssStatus);
}

quint16 SbfParser::computeChecksum(const void *buf, const quint32 length) const
{
  quint32  i;
  quint16  crc = 0;
  const quint8 *buf8 = (quint8*)buf; /* Convert the type to access by byte. */

  /* see for example the BINEX web site */
  for(i=0; i < length; i++)
  {
    crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buf8[i] ];
  }

  return crc;
}

void SbfParser::setPose(const Sbf_IntPVAAGeod* block, const GnssStatus& gnssStatus)
{
    const quint8 latFine = block->PosFine & 15;  // the lower 4 bits
    const quint8 lonFine = block->PosFine & 240; // the upper 4 bits

    const double floatLat = ((double)block->Lat) / 10000000.0l + latFine*6.25e-9f;
    const double floatLon = ((double)block->Lon) / 10000000.0l + lonFine*6.25e-9f;
    const double floatAlt = ((double)block->Alt) / 1000.0l;

    // The rotational velocity is contained in IntAttEuler, which we don't have. So calculate
    // it using the previous pose before mLastPose is overwritten.
    const float timeSinceLastPose = (block->TOW - mLastPose.timestamp) / 1000.0f;
    const QVector3D previousPosePosition = mLastPose.getPosition();
    const QQuaternion previousPoseOrientation = mLastPose.getOrientation();


    // Please look at the septentrio "Firmware user manual", page 47ff for conversion rules.
    mLastPose = Pose(
                convertGeodeticToCartesian(floatLon, floatLat, floatAlt),
                -((double)block->Heading) * 0.01l, // Z axis points down in VehicleReferenceFrame, values from 0 to 360, both point north
                ((double)block->Pitch) * 0.01l,
                -((double)block->Roll) * 0.01l, // their roll axis points forward from the vehicle. We have it OpenGL-style with Z pointing backwards
                (qint32)block->TOW // Receiver time in milliseconds. WARNING: be afraid of WNc rollovers at runtime!
                );

    if(block->Heading != 65535 && block->Pitch != -32768 && block->Roll != -32768)
        mLastPose.precision |= Pose::AttitudeAvailable;

    if(testBit(block->Info, 11)) // Heading ambiguity is Fixed
        mLastPose.precision |= Pose::HeadingFixed;

    // This flag is useless for precision, as non-integrated (the 4 steps between two integrated poses) are good enough for sensor fusion, too.
    if(block->Mode == 2) // integrated solution, not sensor-only or GNSS-only
        mLastPose.precision |= Pose::ModeIntegrated;

    if((block->GNSSPVTMode & 15) == 4) // Thats RTK Fixed, see GpsStatusInformation::getGnssMode().
        mLastPose.precision |= Pose::RtkFixed;

    if(gnssStatus.meanCorrAge < 100) // Thats ten seconds
        mLastPose.precision |= Pose::CorrectionAgeLow;

    // Move the pose from the ARP/Marker that IntPVAAGeod outputs to the vehicle's center
    mLastPose.getMatrixRef() *= mTransformArpToVehicle;

    mLastPose.covariances = gnssStatus.covariances;

    // Determine rotational speed in degrees per second
    if(timeSinceLastPose < 0.15f)
    {
        mLastPose.rotation = Pose::getAngleBetweenDegrees(previousPoseOrientation, mLastPose.getOrientation());
        mLastPose.rotation /= timeSinceLastPose;
    }

    // Determine the velocity. As can be seen in speedbased4 logfiles, the receiver's velocity values ALL
    // drop to zero sometimes for no reason. When that happens, compute velocities based on the last pose.
    if(block->Vnorth != I32_DONOTUSE && block->Veast != I32_DONOTUSE && block->Vup != I32_DONOTUSE && (block->Vnorth != 0 || block->Veast != 0 || block->Vup != 0))
    {
        const QVector3D velocity = QVector3D(block->Veast, block->Vup, -block->Vnorth);
        mLastPose.setVelocity(velocity / 1000.0f);
        //qDebug() << "SbfParser::setPose(): ENU velocities" << block->Veast << block->Vnorth << block->Vup << "are valid, velocity:" << mLastPose.getVelocity();
    }
    else
    {
        const QVector3D velocity = (mLastPose.getPosition() - previousPosePosition) / timeSinceLastPose;
        mLastPose.setVelocity(velocity);
        //qDebug() << "SbfParser::setPose(): ENU velocities" << block->Veast << block->Vnorth << block->Vup << "are either DO-NOT-USE or all zero (=buggy), computing velocities using previous pose:" << mLastPose.getVelocity();
    }

    // Determine acceleration magnitude
    if(block->Ax != I16_DONOTUSE && block->Ay != I16_DONOTUSE && block->Az != I16_DONOTUSE)
        mLastPose.acceleration = QVector3D(block->Ax, block->Ay, block->Az+981).length() / 100.0f;
    else
        mLastPose.acceleration = 0.0f;
}

QVector3D SbfParser::convertGeodeticToCartesian(const double &lon, const double &lat, const double &elevation)
{
    // Set longitude, latitude and elevation of first GNSS fix to let the rover start at cartesian 0/0/0
    // This may ONLY happen based on really good positional fixes (=RTK Fixed): if we initialize these
    // origins using e.g. Differential and then switch to RTKFixed, the kopter might remain underground
    // due to a -2m Y-offset between Differential and RtkFixed.
    if(mOriginLongitude > 10e19 && mOriginLatitude > 10e19 && mOriginElevation > 10e19)
    {
        // Our world coordinate system is not set yet. If the GNSS measurement is precise enough, define it now. If
        // its NOT precise, return a null QVector3D.
        if(mGnssStatus.pvtMode == GnssStatus::PvtMode::RtkFixed)
        {
            mOriginLongitude = lon;
            mOriginLatitude = lat;

            // Whats the transform for?
            mOriginElevation = elevation + mTransformArpToVehicle(1, 3);
        }
        else
        {
            return QVector3D();
        }
    }

    // Doesn't matter, as offset isn't hardcoded anymore this is just for historical reference :)
    // FBI in Hamburg is 53.600515,09.931478 with elevation of about 70m
    // PPM in Penzberg is 47.757201,11.377133 with elevation of about 656m

    // X is Longitude   (+ East/ - West)
    // Y is Elevation / Height
    // Z is Latitude    (+ North / - South)
    QVector3D co;

    co.setY(elevation - mOriginElevation);
    co.setZ(-(lat - mOriginLatitude) * 111300.0l);
    co.setX((lon -  mOriginLongitude) * 111300.0l * cos(DEG2RAD(mOriginLatitude)));

    return co;
}

quint32 SbfParser::processNextValidPacket(const QByteArray &sbfData, const quint32 offsetToValidPacket)
{
    //qDebug() << "SbfParser::processNextValidPacket():" << sbfData.size() << "bytes present.";
/*
    quint32 offsetToValidPacket;
    qint32 tow;

    if(getNextValidPacketInfo(sbfData, &offsetToValidPacket, &tow) == false)
    {
        qDebug() << "SbfParser::processNextValidPacket(): no more valid packets in sbf data of size" << sbfData.size();
        return;
    }

    // If there's garbage before the next valid packet, save it into sbf log and log a warning
    if(offsetToValidPacket != 0)
    {
//        qDebug() << "SbfParser::processNextValidPacket(): WARNING: offset to valid packet was" << offsetToValidPacket << "instead of 0, content:" << readable(sbfData.left(offsetToValidPacket));
        // Log this data for later error analysis
//        emit processedPacket(sbfData.left(offsetToValidPacket), -1);
        emit processedPacket(-1, sbfData.constData(), offsetToValidPacket);
        sbfData.remove(0, offsetToValidPacket);
    }*/

    if(sbfData.at(offsetToValidPacket + 0) != '$' || sbfData.at(offsetToValidPacket + 1) != '@')
    {
        qDebug() << __PRETTY_FUNCTION__ << "sbfData at" << offsetToValidPacket << "doesn't start with $@, but with" << readable(sbfData.mid(offsetToValidPacket, 20));
        Q_ASSERT(false);
    }

    const quint16 msgCrc = *(quint16*)(sbfData.constData() + offsetToValidPacket + 2);
    const quint16 msgId = *(quint16*)(sbfData.constData() + offsetToValidPacket + 4);
    const quint16 msgIdBlock = msgId & 0x1fff;
    const quint16 msgIdRev = msgId >> 13;
    const quint16 msgLength = *(quint16*)(sbfData.constData() + offsetToValidPacket + 6);

    // Save our current gpsStatus in a const place, so we can check whether it changed after processing the whole packet
    const GnssStatus previousGpsStatus = mGnssStatus;

//    qDebug() << "SbfParser::processNextValidPacket(): processing" << sbfData.size() << "bytes SBF data with ID" << msgId << "from TOW" << ((Sbf_PVTCartesian*)sbfData.constData())->TOW;

    // Process the message if we're interested.
    //qDebug() << "received sbf block" << msgIdBlock;
    switch(msgIdBlock)
    {

    case 4006:
    {
        // PVTCartesian
        const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)(sbfData.constData() + offsetToValidPacket);
        // block->MeanCorrAge is quint16 in hundreds of a second
//        qDebug() << "SBF: PVTCartesian: MeanCorrAge in seconds:" << ((float)block->MeanCorrAge)/100.0;
        mGnssStatus.meanCorrAge = std::min(block->MeanCorrAge / 10, 255);
    }
    break;

    case 4072:
    {
        // IntAttCovEuler
        const Sbf_IntAttCovEuler *block = (Sbf_IntAttCovEuler*)(sbfData.constData() + offsetToValidPacket);
        qDebug() << "SBF: IntAttCovEuler: covariances for heading, pitch, roll:" << block->Cov_HeadHead << block->Cov_PitchPitch << block->Cov_RollRoll;
        float newCovarianceValue = std::max(std::max(block->Cov_HeadHead, block->Cov_PitchPitch), block->Cov_RollRoll);
        if(fabs(mGnssStatus.covariances - newCovarianceValue) > 0.02)
        {
            mGnssStatus.covariances = newCovarianceValue;
//            emit status(&mGnssStatus);
        }
    }
    break;

    case 4014:
    {
        // ReceiverStatus
        const Sbf_ReceiverStatus *block = (Sbf_ReceiverStatus*)(sbfData.constData() + offsetToValidPacket);

//        qDebug() << "SBF: ReceiverStatus: CPU Load:" << block->CPULoad;

        mGnssStatus.cpuLoad = block->CPULoad;

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
        const Sbf_IntPVAAGeod *block = (Sbf_IntPVAAGeod*)(sbfData.constData() + offsetToValidPacket);

        qDebug() << "SBF: IntPVAAGeod" << block->TOW << block->GNSSPVTMode << block->Alt << block->Heading << block->Pitch << block->Roll;

        // Check the Info-field and emit states if it changes
        if(mGnssStatus.info != block->Info)
        {
            //qDebug() << t() << "SbfParser::processNextValidPacket(): info changed from" << mGnssStatus.info << "to" << block->Info;


            if(!testBitEqual(mGnssStatus.info, block->Info, 11))
                emit message(
                            testBit(block->Info, 11) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Heading ambiguity fixed: %1").arg(testBit(block->Info, 11) ? "true" : "false"));

            /* These are disabled: GNSS pos and vel are only used in every 5th pose, so these things cause
               high traffic for nothing. Zero constraint is used automatically when GNSS doesn't move.

            if(!testBitEqual(mGnssStatus.info, block->Info, 0))
                emit message(
                            testBit(block->Info, 0) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("ACLR measurements used: %1").arg(testBit(block->Info, 0) ? "true" : "false"));

            if(!testBitEqual(mGnssStatus.info, block->Info, 1))
                emit message(
                            testBit(block->Info, 1) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GYRO measurements used: %1").arg(testBit(block->Info, 1) ? "true" : "false"));

            if(!testBitEqual(mGnssStatus.info, block->Info, 12)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 12) ? Error : Information, // We don't use this, should be 0
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Zero constraint used: %1").arg(testBit(block->Info, 12) ? "true" : "false"));

            if(!testBitEqual(mGnssStatus.info, block->Info, 13)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 13) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS position used: %1").arg(testBit(block->Info, 13) ? "true" : "false"));

            if(!testBitEqual(mGnssStatus.info, block->Info, 14)) // FIXME: until firmware works
                emit message(
                            testBit(block->Info, 14) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS velocity used: %1").arg(testBit(block->Info, 14) ? "true" : "false"));

            if(!testBitEqual(mGnssStatus.info, block->Info, 15))
                emit message(
                            testBit(block->Info, 15) ? Information : Error, // GNSS attitude means multi-antenna, so we don't use it.
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS attitude used: %1").arg(testBit(block->Info, 15) ? "true" : "false"));
            */

            mGnssStatus.info = block->Info;
        }

        /* IntegrationMode changes between integrated and not integrated with 10Hz, we don't honestly care to describe this in some log.
        // Check the Mode-field and emit states if it changes
        if(mGnssStatus.integrationMode != block->Mode)
        {
            //qDebug() << t() << "SbfParser::processNextValidPacket(): mode changed from" << mGnssStatus.integrationMode << "to" << block->Mode;

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
        }*/
        mGnssStatus.setIntegrationMode(block->Mode);

        // Check the Error-field and emit states if it changes
        if(! mGnssStatus.hasError(block->Error))
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): error changed from" << mGnssStatus.error << "to" << block->Error << "at TOW" << block->TOW;

            emit message(
                        block->Error == 0 ? Information : Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Error changed from (%1) to (%2)")
                        .arg(mGnssStatus.getError())
                        .arg(GnssStatus::getError(block->Error)));

            mGnssStatus.setError(block->Error);
        }

        // Check the GnssPvtMode-field and emit states if it changes AND if its not DO-NOT-USE
        if(! mGnssStatus.hasPvtMode(block->GNSSPVTMode) && block->GNSSPVTMode != 255)
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): GnssPvtMode changed from" << mGnssStatus.gnssMode << "to" << block->GNSSPVTMode << "at TOW" << block->TOW;

            emit message(
                        (block->GNSSPVTMode & 15) == 4 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GnssPvtMode changed: %1 => %2")
                        .arg(mGnssStatus.getPvtMode())
                        .arg(GnssStatus::getPvtMode(block->GNSSPVTMode)));

            mGnssStatus.setPvtMode(block->GNSSPVTMode);
        }

        // It is perfectly normal for the GNSSAge to be 0,2,4,6 or 8 milliseconds.
        if(mGnssStatus.gnssAge != block->GNSSage && block->GNSSage > 10)
        {
            //qDebug() << t() << "SbfParser::processNextValidPacket(): GnssAge changed from" << mGnssStatus.gnssAge << "to" << block->GNSSage << "at TOW" << block->TOW;
            emit message(
                        block->GNSSage > 0 ? Information : Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("No GNSS PVT for %1 milliseconds").arg(block->GNSSage));
        }
        mGnssStatus.gnssAge = block->GNSSage;

        const quint8 numberOfSatellitesUsed = (block->NrSVAnt & 31);
        if(mGnssStatus.numSatellitesUsed != numberOfSatellitesUsed)
        {
//            qDebug() << t() << "SbfParser::processNextValidPacket(): numSats changed from" << mGnssStatus.numSatellitesUsed << "to" << numberOfSatellitesUsed;
            /*emit message(
                        numberOfSatellitesUsed > 5 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Number of used satellites changed from %1 to %2").arg(mGnssStatus.numSatellitesUsed).arg(numberOfSatellitesUsed));*/

            mGnssStatus.numSatellitesUsed = numberOfSatellitesUsed;
        }

        setPose(block, mGnssStatus);

        // Here we emit the gathered pose. Depending on the pose's time and its quality, we emit it for different consumers

        // Emitted at full rate, any precision
        emit newVehiclePose(&mLastPose);

        // Emitted slowly (1Hz), any precision
        // PTU-tracking might need faster rate (10Hz)
        if(mLastPose.timestamp % 1000 == 0)
            emit newVehiclePoseStatus(&mLastPose);

        // Only emit a precise pose if the values are not set to the do-not-use values.
        if(
                block->Error == 0
                && block->TOW != U32_DONOTUSE
                && block->Lat != I32_DONOTUSE
                && block->Lon != I32_DONOTUSE
                && block->Alt != I32_DONOTUSE)
        {
            emit newVehiclePoseSensorFuser(&mLastPose);

            // Tell others whether we're working well.
            if(!mGnssDeviceWorkingPrecisely && mLastPose.precision & Pose::AttitudeAvailable && mLastPose.precision & Pose::HeadingFixed && mLastPose.precision & Pose::RtkFixed && mLastPose.precision & Pose::CorrectionAgeLow)
            {
                mGnssDeviceWorkingPrecisely = true;
                qDebug() << block->TOW << "SbfParser::processNextValidPacket(): precise pose, setting mGnssDeviceWorkingPrecisely to true, emitting.";
                emit gnssDeviceWorkingPrecisely(mGnssDeviceWorkingPrecisely);
            }
            else if(mGnssDeviceWorkingPrecisely
                    &&
                    !
                    (mLastPose.precision & Pose::AttitudeAvailable && mLastPose.precision & Pose::HeadingFixed && mLastPose.precision & Pose::RtkFixed && mLastPose.precision & Pose::CorrectionAgeLow)
                    )
            {
                mGnssDeviceWorkingPrecisely = false;
                qDebug() << block->TOW << "SbfParser::processNextValidPacket(): unprecise pose (flags" << mLastPose.precision << "), setting mGnssDeviceWorkingPrecisely to false, emitting.";
                emit gnssDeviceWorkingPrecisely(mGnssDeviceWorkingPrecisely);
            }
        }
        else if(block->Error != 0)
        {
//            qDebug() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, error:" << block->Error << "" << GnssStatus::getError(block->Error) ;
            if(mGnssDeviceWorkingPrecisely)
            {
                mGnssDeviceWorkingPrecisely = false;
                qDebug() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, setting mGnssDeviceWorkingPrecisely to false, emitting.";
                emit gnssDeviceWorkingPrecisely(mGnssDeviceWorkingPrecisely);
            }
        }
        else
        {
            qDebug() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, do-not-use values found in elementary fields.";
            if(mGnssDeviceWorkingPrecisely)
            {
                mGnssDeviceWorkingPrecisely = false;
                qDebug() << block->TOW << "SbfParser::processNextValidPacket(): invalid pose, setting mGnssDeviceWorkingPrecisely to false, emitting.";
                emit gnssDeviceWorkingPrecisely(mGnssDeviceWorkingPrecisely);
            }
        }
    }
    break;

    case 5914:
    {
        // ReceiverTime
        const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)(sbfData.constData() + offsetToValidPacket);

        qDebug() << "SBF: ReceiverTime: TOW:" << block->TOW;

        //qDebug() << t() << block->TOW << "SbfParser::processNextValidPacket(): received ReceiverTime block: msgid" << msgId << "msgIdBlock" << msgIdBlock << "msgLength" << msgLength << "revision" << msgIdRev;

        if(block->TOW == U32_DONOTUSE)
        {
            emit message(
                        Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GPS Receiver TOW is at its do-not-use-value, give it time to initialize."));

            emit insError("SbfParser::processNextValidPacket(): GPS Receiver TOW is at its do-not-use-value, give it time to initialize.");
        }
        else
        {
            // Set system time to gps time. Adding a roundtrip-timer is not a good idea, as the board waits until the
            // second leaps, meaning the time from request to output doesn't equal the time from output to reception.
            qDebug() << __PRETTY_FUNCTION__ << "emitting time for syncing:" << (qint32)block->TOW;
            emit gnssTimeOfWeekEstablished((qint32)block->TOW);
        }
    }
    break;

    case 5924:
    {
        // ExtEvent
        const Sbf_ExtEvent* const block = (Sbf_ExtEvent* const)(sbfData.constData() + offsetToValidPacket);
        qDebug() << "SBF: ExtEvent at" << block->TOW;

        // Laserscanner sync signal is soldered to both ports, but port 1 is broken. If it ever starts working again, I want to know.
        Q_ASSERT(block->Source == 2);

        if(block->TOW != U32_DONOTUSE)
        {
            // Emit the time of the scan. The Scanner sets the pulse at the END of a scan,
            // but our convention is to use times of a scans middle. Thus, decrement 12ms.
            //qDebug() << "SbfParser::processNextValidPacket(): emitting scanFinished with a scanTimeGnss of" << block->TOW - 12;
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

//    qDebug() << "SbfParser::processNextValidPacket(): processed packet id:" << msgIdBlock;

    // emit new status if it changed significantly.
    if(mGnssStatus.interestingOrDifferentComparedTo(previousGpsStatus))
        emit status(&mGnssStatus);

    return msgLength;

    /*
     Remove the processed SBF block from our incoming buffer, so that it contains either nothing, the next (possibly
     half-complete) SBF message or a (possibly half-complete) command-reply. SBF blocks often end with padding bytes
     which are NOT included in the msgLength counter (contrary to SBF guide, pg. 11).

     So, after processing a packet, we cut off AT LEAST msgLength bytes and then look for the next $@ or $R,
     further removing the padding bytes before that next message starts:

     $@<header/><body>...$@...</body>...padding...$R;listCurrentConfig\n\n...........\nUSB1>$@<header/><body/>...padding...$@

                         ^^- *data* showing up as SYNC - thats rare, but it happens
                                                  ^--------- a reply to a command ---------^
     |<---------- msgLength ------->|
     |<------------- to be removed ------------->|

     We cannot search for the next $R using indexOf() because oftentimes, this won't exist in @sbfData, which is many
     megabytes in size. Searching through this hundred times per second is slooooow. So, we just search for a $ using
     the fast indexOf(), and when found look further for a @,R:,R; or R?. This indicates either the next SBF packet,
     the next command-reply (or garbage, e.g. in padding bytes). Still, this makes sure we won't just delete command-
     replies.
    */

    // Search for the next info (command-reply or SBF)
    bool nextInfoFound = false;
    qint16 positionOfNextInfo = msgLength;
//    qDebug() << "SbfParser::processNextValidPacket(): looking for next info, previous msgLength was" << msgLength;
    while(positionOfNextInfo >= 0 && !nextInfoFound)
    {
//        qDebug() << "SbfParser::processNextValidPacket(): looking for a $ starting at" << positionOfNextInfo;
        positionOfNextInfo = sbfData.indexOf('$', positionOfNextInfo);
//        qDebug() << "SbfParser::processNextValidPacket(): $ found at" << positionOfNextInfo  << "of sbfData size:" << sbfData.size();
        if(positionOfNextInfo >= msgLength)
        {
            // construct a string starting at the found position, but make sure not to overrun the buffer-end
            const QString sync = QString::fromLatin1(sbfData.constData() + positionOfNextInfo, std::min(3, sbfData.size() - positionOfNextInfo));
            if(sync.left(2) == "$@" || sync == "$R:" || sync == "$R?" || sync == "$R;")
            {
//                qDebug() << "SbfParser::processNextValidPacket(): nextInfo found at" << positionOfNextInfo << "- breaking.";
                nextInfoFound = true;
                break;
            }
            else
            {
//                qDebug() << "SbfParser::processNextValidPacket(): unfortunately, sync did not match, was:" << readable(sync.toAscii());
//                qDebug() << "SbfParser::processNextValidPacket(): sbfData:" << readable(sbfData);
            }
        }

        if(positionOfNextInfo > 0 && !nextInfoFound)
        {
            // If a $ was found (but no $@, $R*), continue searching AFTER the previous occurence
            positionOfNextInfo++;
//            qDebug() << "SbfParser::processNextValidPacket(): next info not found, will continue to look at:" << positionOfNextInfo;
        }
        else
        {
            // If no $ was found, do not increment positionOfNextInfo
            // from -1 to 0, otherwise the if above would loop again.
        }
    }

    /*
     If positionOfNextInfo is -1 here, it means that there was no $@ or $R in all of sbfData after the processed packet.
     If the LAST BYTE of sbfData is a "$", then don't delete it, it probably is the start of another packet still coming
     in via serial port. On next iteration, we'll probably find the missing and neighboring @ or R character.
    */
    if(positionOfNextInfo < 0)
    {
        // If we have more data after the processed message, but no SYNC was found...
        if((sbfData.size() - msgLength) != 0)
        {
//            qDebug() << "SbfParser::processNextValidPacket(): no $@ or $R* found after processed packet (of"<< msgLength << "bytes), deleting buffer containing" << sbfData.size() - msgLength << "trailing bytes.";
//            qDebug() << "SbfParser::processNextValidPacket(): to be deleted after the processed packet:" << readable(sbfData.right(sbfData.size() - msgLength));
        }

        if(sbfData.right(1) == "$")
        {
//            qDebug() << "SbfParser::processNextValidPacket(): last char was $, so the @ is probably coming in next. Not deleting the $.";
//            qDebug() << "SbfParser::processNextValidPacket(): sbfData:" << readable(sbfData);
            positionOfNextInfo = sbfData.size()-1;
        }
        else
        {
            positionOfNextInfo = sbfData.size();
        }
    }

//    const quint16 bytesToRemove = std::max(msgLength, (quint16)positionOfNextInfo);

    // Announce what packet we just processed. Might be used for logging.
    // ExtEvent is generic enough, the TOW is always at the same location
    const Sbf_ExtEvent * const block = (Sbf_ExtEvent*)sbfData.constData();
    emit processedPacket((qint32)block->TOW);

    //sbfData.remove(0, bytesToRemove);
}

// replace every non-printable char with a special char, for making sbf-data readable
QString SbfParser::readable(const QByteArray& bytes)
{
    QString out;

    for(int i=0;i< bytes.size();i++)
    {
        const char *c = (bytes.constData()+i);
        if(*c > 126 || *c < 32)
            out.append('X');
        else
            out.append(*c);
    }

    return out;
}
