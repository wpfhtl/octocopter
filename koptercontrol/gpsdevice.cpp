#include "pose.h"
#include "qextserialport/src/qextserialport.h"
#include <math.h>

#include "gpsdevice.h"

/* 
 * This is the receiver's bootup-config concerning I/O:
 * > $R: gdio
 * >   DataInOut, DSK1, CMD, SBF+NMEA, (off)
 * >   DataInOut, COM1, CMD, none, (on)
 * >   DataInOut, COM2, MTI, none, (on)
 * >   DataInOut, COM3, RTCMv3, NMEA, (on)
 * >   DataInOut, COM4, CMD, none, (on)
 * >   DataInOut, USB1, CMD, SBF+NMEA, (on)
 * >   DataInOut, USB2, CMD, SBF+NMEA, (off)
 * 
 * We'd usually talk to the receiver on its COM-port to find that ports name (COM3 as above),
 * but as you can see, that port is not configured to accept CMDs, so it won't talk to us.
 * Unfortunately, it cannot be set to accept both CMD and RTCMv3. So the port is configured
 * to accept RTCMv3 and we pre-set mSerialPortOnDeviceCom to COM3 below.
 * 
 * Obviously, we now rely on the bootup-config being correct, but that should work.
 * 
 */

GpsDevice::GpsDevice(QString &serialDeviceFileUsb, QString &serialDeviceFileCom, QObject *parent) : QObject(parent)
{
    qDebug() << "GpsDevice::GpsDevice(): Using usb port" << serialDeviceFileUsb << "and com port" << serialDeviceFileCom;

    mDeviceIsInitialized = false;

    mPoseClockDivisor = 0;

    mNumberOfRemainingRepliesUsb = 0;
    mRtkDataCounter = 0;
    mSerialPortOnDeviceCom = "COM3";
    mSerialPortOnDeviceUsb = "";

    mLastErrorFromDevice = 24;
    mLastModeFromDevice = 0;
    mLastInfoFromDevice = 0;
    mLastGnssPvtModeFromDevice = 0;
    mLastNumberOfSatellitesUsed = 0;
    mLastGnssAgeFromDevice = 255;

    // We use the USB port to talk to the GPS receiver and receive poses
    mSerialPortUsb = new QextSerialPort(serialDeviceFileUsb, QextSerialPort::EventDriven);
    mSerialPortUsb->setBaudRate(BAUD115200);
    mSerialPortUsb->setFlowControl(FLOW_OFF);
    mSerialPortUsb->setParity(PAR_NONE);
    mSerialPortUsb->setDataBits(DATA_8);
    mSerialPortUsb->setStopBits(STOP_1);
    mSerialPortUsb->setTimeout(500);

    mSerialPortUsb->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPortUsb->isOpen())
    {
        qDebug() << "GpsDevice::GpsDevice(): Opening serial port" << serialDeviceFileUsb << "failed:" << mSerialPortUsb->errorString() << "Exiting.";
        exit(1);
    }

    // We use the COM port to feed the device RTK correction data
    mSerialPortCom = new QextSerialPort(serialDeviceFileCom, QextSerialPort::EventDriven);
    mSerialPortCom->setBaudRate(BAUD115200);
    mSerialPortCom->setFlowControl(FLOW_OFF);
    mSerialPortCom->setParity(PAR_NONE);
    mSerialPortCom->setDataBits(DATA_8);
    mSerialPortCom->setStopBits(STOP_1);
    mSerialPortCom->setTimeout(500);

    mSerialPortCom->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if(!mSerialPortCom->isOpen())
    {
        qDebug() << "GpsDevice::GpsDevice(): Opening serial port" << serialDeviceFileCom << "failed:" << mSerialPortCom->errorString() << "Exiting.";
        exit(1);
    }

    mStatusTimer = new QTimer(this);
    mStatusTimer->setInterval(1000);
    connect(mStatusTimer, SIGNAL(timeout()), SLOT(slotEmitCurrentGpsStatus()));

    // initialize the device whenever we get time to do this. By doing it asynchronously, we can give our creator time to connect our signals and fetch them.
    QTimer::singleShot(0, this, SLOT(slotDetermineSerialPortsOnDevice()));
}

GpsDevice::~GpsDevice()
{
    qDebug() << "GpsDevice::~GpsDevice(): stopping output, closing serial ports";
    slotCommunicationStop();
    mSerialPortUsb->close();
    mSerialPortCom->close();
}

void GpsDevice::sendAsciiCommand(QString command)
{
    mCommandQueueUsb.append(command.append("\r\n").toAscii());

    slotFlushCommandQueue();
}

quint8 GpsDevice::slotFlushCommandQueue()
{
    if(mNumberOfRemainingRepliesUsb == 0 && mCommandQueueUsb.size())
    {
        mLastCommandToDeviceUsb = mCommandQueueUsb.takeFirst();
        qDebug() << "GpsDevice::slotFlushCommandQueue(): currently not waiting for a reply, so sending next command:" << mLastCommandToDeviceUsb.trimmed();
        if(mReceiveBufferUsb.size() != 0) qDebug() << "GpsDevice::slotFlushCommandQueue(): WARNING! Receive Buffer still contains:" << mReceiveBufferUsb;

        mSerialPortUsb->write(mLastCommandToDeviceUsb);
        mNumberOfRemainingRepliesUsb++;
    }
    else if(mNumberOfRemainingRepliesUsb)
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): still waiting for" << mNumberOfRemainingRepliesUsb << "command-replies, not sending.";
    }
    else
    {
        qDebug() << "GpsDevice::slotFlushCommandQueue(): nothing to send.";
    }
    
    return mCommandQueueUsb.size();
}

void GpsDevice::slotDetermineSerialPortsOnDevice()
{
    // This method finds the name of the used communication-port on the GPS-device.
    // For example, we might connect using /dev/ttyUSB1, which is a usb2serial adapter
    // and connects to the devices COM2. This method will chat with the device, analyze
    // the prompt in its replies and set mSerialPortOnDevice to COM2. This can be used
    // lateron to tell the device to output useful info on COM2.
    slotEmitCurrentGpsStatus("Setting up communication");

    // We use two connections to the board, the other one is just for feeding the RTK
    // data that we received from rtkfetcher. But we also need to know that ports name,
    // as we need to tell the receiver to accept RTK data on that port.
    if(!mSerialPortUsb->isOpen() || !mSerialPortCom->isOpen())
    {
        slotEmitCurrentGpsStatus("Cannot open GPS serial port(s)");
    }
    
    Q_ASSERT(mSerialPortUsb->isOpen());
    Q_ASSERT(mSerialPortCom->isOpen());

    // We just send any string causing a reply to the device, so we can see on what port it is talking to us.
    if(mSerialPortOnDeviceUsb.isEmpty())
    {
        mSerialPortUsb->write("getReceiverCapabilities\n");
        usleep(100000);
        QCoreApplication::processEvents();

        // After waiting for the reply, read and analyze.
        QByteArray dataUsb = mSerialPortUsb->readAll();
        QString portNameUsb = dataUsb.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataUsb.right(1) == ">" && (portNameUsb.left(3) == "COM" || portNameUsb.left(3) == "USB"))
        {
            mSerialPortOnDeviceUsb = portNameUsb;
            qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial usb port on device is now " << mSerialPortOnDeviceUsb;
        }
        else
        {
            qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialUsbPortOnDevice, data is:" << dataUsb;
            slotEmitCurrentGpsStatus("Couldn't get serialUsbPortOnDevice");
        }
    }
    
    if(mSerialPortOnDeviceCom.isEmpty())
    {
        mSerialPortCom->write("getReceiverCapabilities\n");
        usleep(100000);
        QCoreApplication::processEvents();

        // After waiting for the reply, read and analyze.
        QByteArray dataCom = mSerialPortCom->readAll();
        QString portNameCom = dataCom.right(5).left(4);

        // Use lines ending with e.g. COM2 or USB1 to determine the port on the serial device being used.
        if(dataCom.right(1) == ">" && (portNameCom.left(3) == "COM" || portNameCom.left(3) == "USB"))
        {
            mSerialPortOnDeviceCom = portNameCom;
            qDebug() << "GpsDevice::determineSerialPortOnDevice(): serial com port on device is now " << mSerialPortOnDeviceCom;
        }
        else
        {
            qWarning() << "GpsDevice::determineSerialPortOnDevice(): couldn't get serialComPortOnDevice, data is:" << dataCom;
            slotEmitCurrentGpsStatus("Couldn't get serialComPortOnDevice");
        }
    }

    // Do not start if receiver-time will rollover soon. Should only fail on saturdays, as it will roll over at the end of saturday.
    const quint32 secondsToRollOver = getTimeToTowRollOver();
    if(secondsToRollOver < 86400) qFatal("ReceiverTime will rollover soon (in %d seconds), quitting.", secondsToRollOver);

    // Now that we know what the ports are named, we can setup the board.
    // We connect this signal not in the c'tor but here, because we don't want the slot
    // to be called for answers to requests made in this method.
    connect(mSerialPortUsb, SIGNAL(readyRead()), SLOT(slotSerialPortDataReady()));
    slotCommunicationSetup();
}

quint32 GpsDevice::getTimeToTowRollOver()
{
    Q_ASSERT(mSerialPortOnDeviceUsb.size() == 4);

    qDebug() << "GpsDevice::getTimeToTowRollOver(): requesting receiver time using USB port:" << mSerialPortOnDeviceUsb;
    mSerialPortUsb->write(QString("setDataInOut,"+mSerialPortOnDeviceUsb+",,+SBF\n").toAscii());
    usleep(1000000);
    QCoreApplication::processEvents();
    mSerialPortUsb->readAll();

    mSerialPortUsb->write(QString("exeSBFOnce,"+mSerialPortOnDeviceUsb+",ReceiverTime\n").toAscii());
    usleep(1000000);
    QCoreApplication::processEvents();

    // After waiting for the reply, read and analyze.
    QByteArray dataUsb = mSerialPortUsb->readAll();

    dataUsb.remove(0, dataUsb.indexOf("$@"));

    const Sbf_ReceiverTime *block = (Sbf_ReceiverTime*)dataUsb.data();

    if(block->TOW == 4294967295)
    {
        emit message(Error, "GpsDevice::getTimeToTowRollOver()", "GPS Receiver TOW is at its do-not-use-value, give it time to initialize. Quitting.");
        qWarning() << "GpsDevice::getTimeToTowRollOver(): GPS Receiver TOW is at its do-not-use-value, give it time to initialize. Quitting.";
        QCoreApplication::quit();
    }

    // SecondsPerWeek - CurrentSecondInWeek is number of seconds till rollover
    const int secondsToRollOver = (7 * 86400) - (block->TOW / 1000);

    qDebug() << "GpsDevice::getTimeToTowRollOver(): TOW" << block->TOW << "will roll over in" << secondsToRollOver << "s =" << ((float)secondsToRollOver)/86400.0 << "d";

    return secondsToRollOver;
}

void GpsDevice::slotCommunicationSetup()
{
    Q_ASSERT(mSerialPortOnDeviceUsb.size() == 4);
    Q_ASSERT(mSerialPortOnDeviceCom.size() == 4);

    // Have a look at the Septentrio Firmware User Manual.pdf. These commands
    // are necessary to set the device into RTK-Rover-Mode

    qDebug() << "GpsDevice::setupCommunication(): setting up communication";

    /* use bootup config for now

    // reset communications
    sendAsciiCommand("setDataInOut,all,CMD,none");

    // make the receiver output SBF blocks on our USB connection
    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceUsb+",,+SBF");

    // make the receiver listen to RTK data on specified port
    sendAsciiCommand("setDataInOut,"+mSerialPortOnDeviceCom+",RTCMv3");

    // configure rover in standalone+rtk mode
    sendAsciiCommand("setPVTMode,Rover,StandAlone+RTK");

    // use a static (not moving) base station
    sendAsciiCommand("setDiffCorrUsage,,,,,off");

    // set up INS/GNSS integration
    sendAsciiCommand("setDataInOut,COM2,MTI");

    // when GPS fails, use IMU for how many seconds?
    sendAsciiCommand("setExtSensorUsage,COM2,,10");

    // specify in what orientation the IMU is attached relative to vehicle reference frame (x front, y right and Z down)
    sendAsciiCommand("setExtSensorCalibration,COM2,90,0,0"); // X pointing forward, Y pointing down and Z pointing left

    // specify vector from GPS antenna ARP to IMU in Vehicle reference frame
    // (vehicle reference frame has X forward, Y right and Z down)
    // IMU is 7cm in front, 6.7cm to the right and 33cm below ARP
    sendAsciiCommand("setExtSensorCalibration,COM2,manual,0,90,270,manual,0.07,0.067,0.33");

    // start the integration filter
    sendAsciiCommand("setPVTMode,,,,loosely");
    */

    // set up processing of the event-pulse from the lidar. Use falling edge, not rising.
    sendAsciiCommand("setEventParameters,EventA,High2Low");

    // output IntPVCart, IntAttEuler, and Event-position. ExtSensorMeas is direct IMU measurements
    // We want to know the pose 25 times a second
    sendAsciiCommand("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",IntPVAAGeod,msec400");

    // We want to know PVTCartesion (4006) for MeanCorrAge only, so stream it slowly
    sendAsciiCommand("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",PVTCartesian,msec500");

    // We want to know whenever a scan is finished.
    sendAsciiCommand("setSBFOutput,Stream3,"+mSerialPortOnDeviceUsb+",ExtEvent,OnChange");

    qDebug() << "GpsDevice::setupCommunication(): done setting up communication";

    // emit status signal periodically.
    mStatusTimer->start();
}

void GpsDevice::slotCommunicationStop()
{
    mDeviceIsInitialized = false;

    // For some reason, resetting this port with the SDIO command below doesn't work.
    // We need to get it to accept CMDs by sending 10 Ss to it.
//     mSerialPortCom->write("SSSSSSSSSS");
//     usleep(100000);
//     QCoreApplication::processEvents();

    qDebug() << "GpsDevice::communicationStop(): stopping SBF streams";

    mSerialPortUsb->write(QString("setSBFOutput,Stream1,"+mSerialPortOnDeviceUsb+",none\n").toAscii());
    usleep(100000);
    mSerialPortUsb->write(QString("setSBFOutput,Stream2,"+mSerialPortOnDeviceUsb+",none\n").toAscii());
    usleep(100000);
    mSerialPortUsb->write(QString("setSBFOutput,Stream3,"+mSerialPortOnDeviceUsb+",none\n").toAscii());
//     mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
//     usleep(100000);
//     mSerialPortUsb->write("setDataInOut,all,CMD,none\n");
//     usleep(100000);

//    emit stateChanged(GpsDevice::Stopped, "Orderly shutdown finished");
    slotEmitCurrentGpsStatus("Orderly shutdown finished");

    // no need to update status of a disabled device
    mStatusTimer->stop();

    QCoreApplication::processEvents();
}

void GpsDevice::slotSerialPortDataReady()
{
    usleep(100000); // wait for the later bytes of this message to come on in...
    mReceiveBufferUsb.append(mSerialPortUsb->readAll());

//    qDebug() << "GpsDevice::slotSerialPortDataReady():" << mReceiveBufferUsb;

    if(mNumberOfRemainingRepliesUsb != 0)
    {
        const int position = mReceiveBufferUsb.indexOf(mSerialPortOnDeviceUsb + QString(">"));
        if(position != -1)
        {
            qDebug() << "GpsDevice::slotSerialPortDataReady(): received reply to" << mLastCommandToDeviceUsb.trimmed() << ":" << mReceiveBufferUsb.left(position).trimmed();
//            qDebug() << "GpsDevice::slotSerialPortDataReady(): now sending next command, if any";

            if(mReceiveBufferUsb.left(position).contains("$R? ASCII commands between prompts were discarded!"))
                qDebug() << "GpsDevice::slotSerialPortDataReady(): it seems we were talking too fast?!";

            mReceiveBufferUsb.remove(0, position+5);
            mNumberOfRemainingRepliesUsb--;
            slotFlushCommandQueue();
        }
        else
        {
            Q_ASSERT(false && "Expected a GPS command reply from USB port, but didn't find a prompt at the end");
        }
    }
    else
    {
        // We're receiving from the device, and it is not a reply to some request we sent ourselves. Thus, the device is
        // talking to us on its own, which only happens after initializing it
        mDeviceIsInitialized = true;
        // We're not waiting for a reply to a command, this must be SBF data!
//        qDebug() << "GpsDevice::slotSerialPortDataReady(): received" << mReceiveBufferUsb.size() << "bytes of SBF data.";
        processSbfData();
    }

}

quint16 GpsDevice::getCrc(const void *buf, unsigned int length)
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

void GpsDevice::processSbfData()
{
    //qDebug() << "GpsDevice::processSbfData():" << mReceiveBufferUsb.size() << "bytes present.";

    while(mReceiveBufferUsb.size() > 8)
    {
//        qDebug() << "GpsDevice::processSbfData(): more than 8 data bytes present, processing.";
        const int indexOfSyncMarker = mReceiveBufferUsb.indexOf("$@");

        if(indexOfSyncMarker != 0)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: SBF Sync Marker $@ was not at byte 0, but at" << indexOfSyncMarker;
            qWarning() << "GpsDevice::processSbfData(): WARNING: removing section before $@:" << mReceiveBufferUsb.mid(0, indexOfSyncMarker);
            mReceiveBufferUsb.remove(0, indexOfSyncMarker);
        }

        const quint16 msgCrc = *(quint16*)(mReceiveBufferUsb.data() + 2);
        const quint16 msgId = *(quint16*)(mReceiveBufferUsb.data() + 4);
        const quint16 msgIdBlock = msgId & 0x1fff;
        const quint16 msgIdRev = msgId >> 13;
        const quint16 msgLength = *(quint16*)(mReceiveBufferUsb.data() + 6);

        if(mReceiveBufferUsb.size() < msgLength)
        {
            qDebug() << "GpsDevice::processSbfData(): message incomplete, we only have" << mReceiveBufferUsb.size() << "of" << msgLength << "bytes";
            return;
        }

        if(getCrc(mReceiveBufferUsb.data()+4, msgLength-4) != msgCrc)
        {
            qWarning() << "GpsDevice::processSbfData(): WARNING: CRC in msg" << msgCrc << "computed" << getCrc(mReceiveBufferUsb.data()+4, msgLength-4) << "msgIdBlock" << msgIdBlock;
            continue;
        }

//        const quint32 timeInWeek = *(sbfMessageBody.data());
//        const quint16 timeWeekNumber = *(sbfMessageBody.data()+4);
//        printf("time in week: %8.8f\n\nv", ((float)timeInWeek)/1000.0);
//        fflush(stdout);
//        qDebug() << "GpsDevice::processSbfData(): gps time is" << timeInWeek/1000.0 << "seconds into week" << timeWeekNumber;

        // Process the message if we're interested.
        //qDebug() << "received sbf block" << msgIdBlock;
        switch(msgIdBlock)
        {

        case 4006:
        {
            // PVTCartesian
            const Sbf_PVTCartesian *block = (Sbf_PVTCartesian*)mReceiveBufferUsb.data();
            mLastMeanCorrAge = std::min(block->MeanCorrAge / 10.0, 255.0);
            qDebug() << "SBF: PVTCartesian: MeanCorrAge in seconds:" << ((float)block->MeanCorrAge)/100.0;
        }
        break;

        case 4045:
        {
            // IntPVAAGeod
//            qDebug() << "SBF: PVAAGeod";
            const Sbf_PVAAGeod *block = (Sbf_PVAAGeod*)mReceiveBufferUsb.data();

            // Check the Info-field and emit states if it changes
            if(mLastInfoFromDevice != block->Info)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): info changed from" << mLastInfoFromDevice << "to" << block->Info;
                const quint16 previousInfoFromDevice = mLastInfoFromDevice;
                mLastInfoFromDevice = block->Info;

                if(!testBitEqual(previousInfoFromDevice, block->Info, 0))
                    slotEmitCurrentGpsStatus(QString("ACLR measurements used: %1").arg(testBit(block->Info, 0) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 1))
                    slotEmitCurrentGpsStatus(QString("GYRO measurements used: %1").arg(testBit(block->Info, 1) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 11))
                    slotEmitCurrentGpsStatus(QString("Heading ambiguity fixed: %1").arg(testBit(block->Info, 11) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 12))
                    slotEmitCurrentGpsStatus(QString("Zero constraint used: %1").arg(testBit(block->Info, 12) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 13))
                    slotEmitCurrentGpsStatus(QString("GNSS position used: %1").arg(testBit(block->Info, 13) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 14))
                    slotEmitCurrentGpsStatus(QString("GNSS velocity used: %1").arg(testBit(block->Info, 14) ? "true" : "false"));

                if(!testBitEqual(previousInfoFromDevice, block->Info, 15))
                    slotEmitCurrentGpsStatus(QString("GNSS attitude used: %1").arg(testBit(block->Info, 15) ? "true" : "false"));

                // If mode was 26627 or 30723, that would be 11X100000000011
                if(block->Info == 26627 || block->Info == 30723) Q_ASSERT("Whee, GPS INFO is the way it should be!");
            }

            // Check the Mode-field and emit states if it changes
            if(mLastModeFromDevice != block->Mode)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): mode changed from" << mLastModeFromDevice << "to" << block->Mode;
                mLastModeFromDevice = block->Mode;

                switch(block->Mode)
                {
                case 0:
//                    mStatus = Error;
//                    emit stateChanged(mStatus, "Mode changed, no integrated solution available");
                    slotEmitCurrentGpsStatus("Mode changed, no integrated solution available");
                    break;

                case 1:
//                    emit stateChanged(mStatus, "Mode changed, using only external sensor");
                    slotEmitCurrentGpsStatus("Mode changed, using only external sensor");
                    break;

                case 2:
//                    mStatus = Running;
//                    emit stateChanged(mStatus, "Mode changed, using integrated solution");
                    slotEmitCurrentGpsStatus("Mode changed, using integrated solution");
                    break;

                default:
                    qWarning() << "GpsDevice::processSbfData(): WARNING: unknown mode code" << block->Mode;
//                    mStatus = Error;
//                    emit stateChanged(mStatus, QString("Unknown Mode %1").arg(block->Mode));
                    slotEmitCurrentGpsStatus(QString("Unknown Mode %1").arg(block->Mode));
                    break;
                }
            }

            // Check the Error-field and emit states if it changes
            if(mLastErrorFromDevice != block->Error)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): error changed from" << mLastErrorFromDevice << "to" << block->Error;
                mLastErrorFromDevice = block->Error;
                slotEmitCurrentGpsStatus(GpsStatusInformation::getError(mLastErrorFromDevice));
            }

            // Check the GnssPvtMode-field and emit states if it changes
            if(mLastGnssPvtModeFromDevice != block->GNSSPVTMode)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): GnssPvtMode changed from" << mLastGnssPvtModeFromDevice << "to" << block->GNSSPVTMode;
                mLastGnssPvtModeFromDevice = block->GNSSPVTMode;
                //Q_ASSERT(mLastGnssPvtModeFromDevice != 4 && "Finally, we have fixed RTK!");
                slotEmitCurrentGpsStatus(GpsStatusInformation::getGnssMode(mLastGnssPvtModeFromDevice));
            }

            // TODO: this might change often in regular usage, really notify?
/*            if(mLastGnssAgeFromDevice != block->GNSSage)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): GnssAge changed from" << mLastGnssAgeFromDevice << "to" << block->GNSSage;
                mLastGnssAgeFromDevice = block->GNSSage;
                slotEmitCurrentGpsStatus(QString("No GNSS-PVT for %1 seconds").arg(block->GNSSage));
            }
*/
            const quint8 numberOfSatellitesUsed = (block->NrSVAnt & 31);
            if(numberOfSatellitesUsed != mLastNumberOfSatellitesUsed)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): numSats changed from" << mLastNumberOfSatellitesUsed << "to" << numberOfSatellitesUsed;
                mLastNumberOfSatellitesUsed = numberOfSatellitesUsed;
                slotEmitCurrentGpsStatus(QString("Number of used satellites changed to %1").arg(numberOfSatellitesUsed));
            }

            // Only emit a pose if the values are not set to the do-not-use values.
            if(
                    block->Error == 0
                    && block->Lat != -2147483648
                    && block->Lon != -2147483648
                    && block->Alt != -2147483648
                    && block->Heading != 65536
                    && block->Pitch != -32768
                    && block->Roll != -32768
                    && block->TOW != 4294967295
                    )
            {
                // TODO: we COULD read the sub-cm part, too...
                const float  lon = ((float)block->Lon) / 10000000.0;
                const float  lat = ((float)block->Lat) / 10000000.0;
                const float  alt = ((float)block->Alt) / 1000.0;
                Pose p(
                            // TODO: use PosFine, see SBF reference guide, page 80?
                            convertGeodeticToCartesian(lon, lat, alt),
                            QQuaternion::fromAxisAndAngle(0,1,0, ((float)block->Heading) * 0.001) *
                            QQuaternion::fromAxisAndAngle(1,0,0, ((float)block->Pitch) * 0.001) *
                            QQuaternion::fromAxisAndAngle(0,0,1, ((float)block->Roll) * 0.001),
                            block->TOW // Receiver time in milliseconds. WARNING: be afraid of WNc rollovers at runtime!
                            );

                qDebug() << "GpsDevice::processSbfData(): new pose:"
                         << "x" << p.position.x()
                         << "y" << p.position.y()
                         << "z" << p.position.z()
                         << "p" << p.getPitchDegrees()
                         << "r" << p.getRollDegrees()
                         << "y" << p.getYawDegrees();

                emit newVehiclePose(p);

                mPoseClockDivisor++;
                if(mPoseClockDivisor % 20 == 0) emit newVehiclePoseLowFreq(p);

            }
            else if(block->Error != 0)
            {
                qDebug() << t() << "GpsDevice::processSbfData(): pose from PVAAGeod not valid, error:" << block->Error << " " << GpsStatusInformation::getError(block->Error) ;
            }
            else
            {
                qDebug() << t() << "GpsDevice::processSbfData(): pose from PVAAGeod not valid, do-not-use values found.";
            }

//            qDebug() << "SBF: IntAttEuler: Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "TOW" << block->TOW << "WNc" << block->WNc << "HPR:" << block->Heading << block->Pitch << block->Roll;;
//            qDebug() << "Info" << block->Info << "Mode" << block->Mode << "Error" << block->Error << "HPR:" << block->Heading << block->Pitch << block->Roll;;
        }
            break;
        case 5924:
        {
            // ExtEvent
            qDebug() << "SBF: ExtEvent";
            const Sbf_ExtEvent *block = (Sbf_ExtEvent*)mReceiveBufferUsb.data();

            if(block->TOW != 4294967295)
                emit scanFinished(block->TOW);
            else
                qDebug() << "GpsDevice::processSbfData(): WARNING: scan finished, but TOW is set to do-not-use!";
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
            qDebug() << "GpsDevice::processSbfData(): ignoring block id" << msgIdBlock;
        }
        }

        // Remove the SBF block body from our incoming USB buffer, so it contains either nothing or the next SBF message
        mReceiveBufferUsb.remove(0, msgLength);
    }

    if(mReceiveBufferUsb.size())
        qDebug() << "GpsDevice::processSbfData(): done processing SBF data, bytes left in buffer:" << mReceiveBufferUsb.size();

}

void GpsDevice::slotSetRtkData(const QByteArray &data)
{
    if(mDeviceIsInitialized)
    {
        // simply write the RTK data into the com-port
        mRtkDataCounter += data.size();
        qDebug() << "GpsDevice::slotSetRtkData(): forwarding" << data.size() << "bytes of rtk-data to gps device, total is" << mRtkDataCounter;
        mSerialPortCom->write(data);
        emit message(
                Information,
                "GpsDevice::slotSetRtkData()",
                QString("Fed %1 bytes of RTK data into rover gps device.").arg(data.size())
                );
    }
    else
    {
        qDebug() << "GpsDevice::slotSetRtkData(): NOT forwarding" << data.size() << "bytes of rtk-data to gps device, its not initialized yet.";
    }
}

void GpsDevice::slotEmitCurrentGpsStatus(const QString& text)
{
    emit gpsStatus(mLastGnssPvtModeFromDevice, mLastModeFromDevice, mLastInfoFromDevice, mLastErrorFromDevice, mLastNumberOfSatellitesUsed, mLastGnssAgeFromDevice, mLastMeanCorrAge, text);
    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("GnssMode %1, IntMode %2, Info %3, Error %4, NumSats %5, GnssAge %6, MeanCorrAge %8").arg(mLastGnssPvtModeFromDevice).arg(mLastModeFromDevice).arg(mLastInfoFromDevice).arg(mLastErrorFromDevice).arg(mLastNumberOfSatellitesUsed).arg(mLastGnssAgeFromDevice).arg(mLastMeanCorrAge)
                );
}

QVector3D GpsDevice::convertGeodeticToCartesian(const double &lon, const double &lat, const float &elevation)
{
    QVector3D co;
    co.setY(elevation);
    co.setZ(-(lat - 53.600669) * 111300.0);
    co.setX((lon - 9.933817l) * 111300.0l * cos(M_PI / 180.0 * 53.600669l));

    return co;
}
