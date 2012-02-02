#include "sbfparser.h"

SbfParser::SbfParser(QObject *parent) : QObject(parent)
{
    // Will be set on first PVT reception and is used to let the rover start at cartesian 0/0/0
    mOriginLongitude = 10e20;
    mOriginLatitude = 10e20;
    mOriginElevation = 10e20;

    mTimeStampStartup = QDateTime::currentDateTime();

    mPoseClockDivisor = 0;

    mFirmwareBug_20120111_RtkWasEnabledAfterAttitudeDeterminationSucceeded = false;

//    mStatusTimer = new QTimer(this);
//    mStatusTimer->setInterval(1000);
//    connect(mStatusTimer, SIGNAL(timeout()), SLOT(slotEmitCurrentGpsStatus()));
//    mStatusTimer->start(); // emit status signal periodically.
}

SbfParser::~SbfParser()
{
    // no need to update status of a disabled device
//    mStatusTimer->stop();
//    mStatusTimer->deleteLater();
}

qint32 SbfParser::peekNextTow(const QByteArray& sbfData)
{
    if(sbfData.length() >= 12 && sbfData.left(2) == "$@")
    {
//        tow = (quint32)(*(sbfData.constData()+8));
        const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)sbfData.data();
        return block->TOW;
    }

    return 0;
}

/*void SbfParser::slotEmitCurrentGpsStatus(const QString& text)
{
    emit gpsStatus(mLastGpsStatus, text);

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                );
}*/

quint16 SbfParser::getCrc(const void *buf, unsigned int length)
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

void SbfParser::setPose(const qint32& lon, const qint32& lat, const qint32& alt, const quint16& heading, const qint16& pitch, const qint16& roll, const quint32& tow)
{
    const double floatLon = ((double)lon) / 10000000.0l;
    const double floatLat = ((double)lat) / 10000000.0l;
    const double floatAlt = ((double)alt) / 1000.0l;

    mLastPose = Pose(
                convertGeodeticToCartesian(floatLon, floatLat, floatAlt),
                ((double)heading) * 0.01l,
                ((double)pitch) * 0.01l,
                ((double)roll) * 0.01l,
                (qint32)tow // Receiver time in milliseconds. WARNING: be afraid of WNc rollovers at runtime!
                );
}

QVector3D SbfParser::convertGeodeticToCartesian(const double &lon, const double &lat, const double &elevation)
{
    // Set longitude, latitude and elevation of first GNSS fix to let the rover start at cartesian 0/0/0
    if(mOriginLongitude > 10e19 && mOriginLatitude > 10e19 && mOriginElevation > 10e19)
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

bool SbfParser::processSbfData(QByteArray& sbfData)
{
    //qDebug() << "SbfParser::processSbfData():" << sbfData.size() << "bytes present.";

    if(sbfData.size() <= 8)
        return false;

    //qDebug() << "SbfParser::processSbfData(): more than 8 data bytes present, processing.";
    const int indexOfSyncMarker = sbfData.indexOf("$@");

    if(indexOfSyncMarker == -1)
    {
        // The sync marker wasn't found! This means the buffer contains unusable data,
        // because we cannot use any data not starting with a sync-marker. So the data must
        // be non-SBF and should be consumed by someone else.
        return false;
    }
    else if(indexOfSyncMarker != 0)
    {
        qWarning() << "SbfParser::processSbfData(): WARNING: SBF Sync Marker $@ was not at byte 0, but at" << indexOfSyncMarker;
        // Log this data for later error analysis
        emit processedPacket(sbfData.left(indexOfSyncMarker));
        sbfData.remove(0, indexOfSyncMarker);
        // Tell our caller that he can call us again to try to parse everything after the junk we just removed.
        return true;
    }

    const quint16 msgCrc = *(quint16*)(sbfData.data() + 2);
    const quint16 msgId = *(quint16*)(sbfData.data() + 4);
    const quint16 msgIdBlock = msgId & 0x1fff;
    const quint16 msgIdRev = msgId >> 13;
    const quint16 msgLength = *(quint16*)(sbfData.data() + 6);

    if(sbfData.size() < msgLength)
    {
//        qDebug() << t() << "SbfParser::processSbfData(): message incomplete, we only have" << sbfData.size() << "of" << msgLength << "bytes. Processing postponed..";
        return false;
    }

    // We limit the length to the buffer's size, because for broken packets, msgLength might be a random value
    if(getCrc(sbfData.data()+4, std::min(sbfData.size()-4, msgLength-4)) != msgCrc)
    {
        qWarning() << "SbfParser::processSbfData(): WARNING: CRC in msg" << msgCrc << "computed" << getCrc(sbfData.data()+4, msgLength-4) << "msgIdBlock" << msgIdBlock;
        // Remove the SBF block body from the buffer, so it contains either nothing or the next SBF message
        // Since the CRC is wrong, msgLength might also be off. Thus we delete just two bytes at the beginning, causing
        // a warning about spurious data in the next processing iteration, but thats still more safe.
        sbfData.remove(0, 2);
        return true;
    }

    // Save our current gpsStatus in a const place, so we can check whether it changed after processing the whole packet
    const GpsStatusInformation::GpsStatus previousGpsStatus = mGpsStatus;

//    qDebug() << "SbfParser::processSbfData(): processing" << sbfData.size() << "bytes SBF data with ID" << msgId << "from TOW" << ((Sbf_PVTCartesian*)sbfData.data())->TOW;

    // Process the message if we're interested.
    //qDebug() << "received sbf block" << msgIdBlock;
    switch(msgIdBlock)
    {

    case 4006:
    {
        // PVTCartesian
        const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)sbfData.data();
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
        if(abs(mGpsStatus.covariances - newCovarianceValue) > 0.02)
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

        // Only emit changes when CPU load changes a lot.
        if(abs(mGpsStatus.cpuLoad - block->CPULoad) > 0)
        {
            mGpsStatus.cpuLoad = block->CPULoad;
//            emit status(mGpsStatus);
        }

        if(block->CPULoad > 80)
        {
            qWarning() << "SbfParser::processSbfData(): WARNING, receiver CPU load is" << block->CPULoad;
            emit message(Warning,
                         QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                         QString("Warning, CPU load is too high (%1 percent)").arg(block->CPULoad));
        }

        if(block->ExtError != 0)
        {
            qWarning() << "SbfParser::processSbfData(): ExtError is not 0 but" << block->ExtError;
            emit message(
                        Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Warning, ExtError is not zero (%1)").arg(block->ExtError));

            // According to SBF guide pg. 101, this means diff corr data error. Lets see what it is.
            if(block->ExtError == 2) emit receiverCommand("lif,DiffCorrError");
        }

        if(block->RxError != 0)
        {
            qWarning() << "SbfParser::processSbfData(): RxError is not 0 but" << block->RxError;
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
            //qDebug() << t() << "SbfParser::processSbfData(): info changed from" << mGpsStatus.info << "to" << block->Info;

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

            if(!testBitEqual(mGpsStatus.info, block->Info, 12))
                emit message(
                            testBit(block->Info, 12) ? Error : Information, // We don't use this, should be 0
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("Zero constraint used: %1").arg(testBit(block->Info, 12) ? "true" : "false"));

            if(!testBitEqual(mGpsStatus.info, block->Info, 13))
                emit message(
                            testBit(block->Info, 13) ? Information : Error,
                            QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                            QString("GNSS position used: %1").arg(testBit(block->Info, 13) ? "true" : "false"));

            if(!testBitEqual(mGpsStatus.info, block->Info, 14))
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
        if(mGpsStatus.integrationMode != block->Mode)
        {
            //qDebug() << t() << "SbfParser::processSbfData(): mode changed from" << mGpsStatus.integrationMode << "to" << block->Mode;

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
                qWarning() << "SbfParser::processSbfData(): WARNING: unknown mode code" << block->Mode << "at TOW" << block->TOW;
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
//            qDebug() << t() << "SbfParser::processSbfData(): error changed from" << mGpsStatus.error << "to" << block->Error << "at TOW" << block->TOW;

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
//            qDebug() << t() << "SbfParser::processSbfData(): GnssPvtMode changed from" << mGpsStatus.gnssMode << "to" << block->GNSSPVTMode << "at TOW" << block->TOW;

            emit message(
                        block->GNSSPVTMode & 15 == 4 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GnssPvtMode changed from %1 to %2")
                        .arg(GpsStatusInformation::getGnssMode(mGpsStatus.gnssMode))
                        .arg(GpsStatusInformation::getGnssMode(block->GNSSPVTMode)));

            mGpsStatus.gnssMode = block->GNSSPVTMode;
        }

        if(mGpsStatus.gnssAge != block->GNSSage)
        {
//            qDebug() << t() << "SbfParser::processSbfData(): GnssAge changed from" << mGpsStatus.gnssAge << "to" << block->GNSSage << "at TOW" << block->TOW;

            emit message(
                        block->GNSSage > 0 ? Information : Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("No GNSS PVT for %1 seconds").arg(block->GNSSage));

            mGpsStatus.gnssAge = block->GNSSage;
        }

        const quint8 numberOfSatellitesUsed = (block->NrSVAnt & 31);
        if(mGpsStatus.numSatellitesUsed != numberOfSatellitesUsed)
        {
//            qDebug() << t() << "SbfParser::processSbfData(): numSats changed from" << mGpsStatus.numSatellitesUsed << "to" << numberOfSatellitesUsed;
            emit message(
                        numberOfSatellitesUsed > 5 ? Information : Warning,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("Number of used satellites changed from %1 to %2").arg(mGpsStatus.numSatellitesUsed).arg(numberOfSatellitesUsed));

            mGpsStatus.numSatellitesUsed = numberOfSatellitesUsed;
        }

        // Only emit a pose if the values are not set to the do-not-use values.
        if(
                block->Error == 0
                && block->Lat != -2147483648L
                && block->Lon != -2147483648L
                && block->Alt != -2147483648L
                && block->Heading != 65535 // This is not to the erroneous (off-by-one) spec (SBF Ref Guide, p. 80).
                && block->Pitch != -32768
                && block->Roll != -32768
                && block->TOW != 4294967295L
                && testBit(block->Info, 11) // Heading ambiguity is Fixed
                //&& block->Mode == 2 // integrated solution, not sensor-only or GNSS-only
                && (block->GNSSPVTMode & 15) == 4 // Thats RTK Fixed, see GpsStatusInformation::getGnssMode().
                //&& block->GNSSage < 1 // seconds interval of no GNSS PVT
                && mGpsStatus.covariances < 1.0 // make sure the filter is happy with itself
                )
        {
            setPose(block->Lon, block->Lat, block->Alt, block->Heading, block->Pitch, block->Roll, block->TOW);
            emit newVehiclePosePrecise(mLastPose);
            mPoseClockDivisor++;
        }
        else
        {
            /* hopefully not necessary anymore, firmware is fixed.
            // Special case for buggy firmware (no attitude in RTK-mode): if we're not in RTK-Mode, but the heading is correct
            if(!mFirmwareBug_20120111_RtkWasEnabledAfterAttitudeDeterminationSucceeded
                    && block->Heading != 65535
                    && block->Pitch != -32768
                    && block->Roll != -32768
                    && testBit(mGpsStatus.info, 11)
                    )
            {
                qDebug() << t() << "SbfParser::processSbfData(): enabling RTK after heading is determined and ambiguity is fixed.";
                // Enable RTK Mode for 2cm precision
                receiverCommand("setPVTMode,Rover,all,auto,Loosely");
                // Now we want to know the precise pose 25 times a second
                receiverCommand("setSBFOutput,Stream1,#USB#,IntPVAAGeod,msec40");
                mFirmwareBug_20120111_RtkWasEnabledAfterAttitudeDeterminationSucceeded = true;
            }*/

            // this whole section is logically weak. Get this sorted!
            if(!testBit(block->Info, 11))
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): pose from PVAAGeod not valid, heading ambiguity is not fixed.";
            }

            if(block->Error != 0)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, error:" << block->Error << "" << GpsStatusInformation::getError(block->Error) ;
            }

            if(block->Heading == 65535)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, heading do-not-use";
            }

            if(block->GNSSage > 1)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, GNSSAge is" << block->GNSSage;
            }

            if(
                    block->Lat == -2147483648L
                    || block->Lon == -2147483648L
                    || block->Alt == -2147483648L
                    || block->Pitch == -32768
                    || block->Roll == -32768
                    || block->TOW == 4294967295L)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, do-not-use values found.";
            }
            else if(block->Mode != 2)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, not integrated solution, but" << GpsStatusInformation::getIntegrationMode(block->Mode);
                setPose(block->Lon, block->Lat, block->Alt, block->Heading, block->Pitch, block->Roll, block->TOW);
                emit newVehiclePose(mLastPose);
                mPoseClockDivisor++;
            }
            else if((block->GNSSPVTMode & 15) != 4)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, GnssPvtMode is" << GpsStatusInformation::getGnssMode(block->GNSSPVTMode) << "corrAge:" << mGpsStatus.meanCorrAge << "sec";
                setPose(block->Lon, block->Lat, block->Alt, block->Heading, block->Pitch, block->Roll, block->TOW);
                emit newVehiclePose(mLastPose);
                mPoseClockDivisor++;
            }
            else if(mGpsStatus.covariances > 1.0f)
            {
                qDebug() << t() << block->TOW << "SbfParser::processSbfData(): invalid pose, covariances are" << mGpsStatus.covariances;
                setPose(block->Lon, block->Lat, block->Alt, block->Heading, block->Pitch, block->Roll, block->TOW);
                emit newVehiclePose(mLastPose);
                mPoseClockDivisor++;
            }
        }

        // If the last pose is valid (i.e. not default-constructed), emit it now.
        // On the first time, this will start the laserscanner.
        if(mPoseClockDivisor % 20 == 0 && mLastPose.timestamp != 0)
            emit newVehiclePoseLowFreq(mLastPose);

        //qDebug() << "SBF: IntAttEuler: Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "TOW" << block->TOW << "WNc" << block->WNc << "HPR:" << block->Heading << block->Pitch << block->Roll;;
        //qDebug() << "Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "HPR:" << block->Heading << block->Pitch << block->Roll;;
    }
    break;

    case 5914:
    {
        // ReceiverTime
        const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)sbfData.data();

        qDebug() << "SBF: ReceiverTime: TOW:" << block->TOW;

        //qDebug() << t() << block->TOW << "SbfParser::processSbfData(): received ReceiverTime block: msgid" << msgId << "msgIdBlock" << msgIdBlock << "msgLength" << msgLength << "revision" << msgIdRev;

        if(block->TOW == 4294967295L)
        {
            emit message(
                        Error,
                        QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                        QString("GPS Receiver TOW is at its do-not-use-value, give it time to initialize."));

            qWarning() << "SbfParser::processSbfData(): GPS Receiver TOW is at its do-not-use-value, give it time to initialize.";

            if(mTimeStampStartup.secsTo(QDateTime::currentDateTime()) < 15)
            {
                qWarning() << "SbfParser::processSbfData(): GPS Receiver TOW is at its do-not-use-value during startup - quitting.";
                QCoreApplication::quit();
            }
        }
        else
        {
            // Set system time to gps time. Adding a roundtrip-timer is not a good idea, as the board waits until the
            // second leaps, meaning the time from request to output doesn't equal the time from output to reception.
            emit gpsTimeOfWeekEstablished(block->TOW);
        }
    }
    break;

    case 5924:
    {
        // ExtEvent
//        qDebug() << "SBF: ExtEvent";
        const Sbf_ExtEvent *block = (Sbf_ExtEvent*)sbfData.data();

        // Laserscanner sync signal is soldered to both ports, but port 1 is broken. If it ever starts working again, I want to know.
        Q_ASSERT(block->Source == 2);

        if(block->TOW != 4294967295L)
        {
            // Emit the time of the scan. The Scanner sets the pulse at the END of a scan,
            // but our convention is to use times of a scans middle. Thus, decrement 12ms.
            //qDebug() << "SbfParser::processSbfData(): emitting scanFinished with a scanTimeGps of" << block->TOW - 12;
            emit scanFinished(block->TOW - 12);
        }
        else
            qDebug() << "SbfParser::processSbfData(): WARNING: scan finished, but TOW is set to do-not-use!";
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
        qDebug() << "SBF: ExtSensorMeas";
    }
    break;
    default:
    {
        qDebug() << "SbfParser::processSbfData(): ignoring block id" << msgIdBlock;
    }
    }

    // emit new status if it changed.
    if(mGpsStatus != previousGpsStatus) emit status(mGpsStatus);

    // Announce what packet we just processed. Might be used for logging.
    emit processedPacket(sbfData.left(msgLength));

    // Remove the SBF block body from our incoming USB buffer, so it contains either nothing or the next SBF message
    sbfData.remove(0, msgLength);

    //if(sbfData.size()) qDebug() << "SbfParser::processSbfData(): done processing SBF data, bytes left in buffer:" << sbfData.size() << "bytes:" << sbfData;

    // Tell the caller whether we'd be able to process another packet in this buffer.
    return sbfData.size() > 8;
}
