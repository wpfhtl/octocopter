#include "logplayer.h"
#include "ui_logplayer.h"

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);

    setMaximumSize(minimumSize());

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotPlay()));

    mSensorFuser = new SensorFuser(1);
    //mSensorFuser->setMaximumFusableRayLength(20.0f);
    mSbfParser = new SbfParser(this);

    mIndexLaser = -1;
    mIndexFlightController = -1;

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonPlay, SIGNAL(clicked()), SLOT(slotPlay()));
    connect(ui->mPushButtonStepForward, SIGNAL(clicked()), SLOT(slotStepForward()));

    connect(ui->mSpinBoxLaserScannerX, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerY, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerZ, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerYaw, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerPitch, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerRoll, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));

    // emit fused lidarpoints
    connect(mSensorFuser, SIGNAL(newScannedPoints(QVector<QVector3D>,QVector3D)), SIGNAL(scanData(QVector<QVector3D>,QVector3D)));

    connect(mSbfParser, SIGNAL(status(GnssStatusInformation::GnssStatus)), SIGNAL(gnssStatus(GnssStatusInformation::GnssStatus)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePoseLogPlayer(Pose)), SIGNAL(vehiclePose(Pose)));
    connect(mSbfParser, SIGNAL(newVehiclePose(Pose)), SIGNAL(vehiclePose(Pose)));

    connect(mSbfParser, SIGNAL(newVehiclePoseSensorFuser(Pose)), mSensorFuser, SLOT(slotNewVehiclePose(Pose)));
    connect(mSbfParser, SIGNAL(processedPacket(QByteArray,qint32)), SLOT(slotNewSbfTime(QByteArray,qint32)));
//    connect(mSbfParser, SIGNAL(scanFinished(quint32)), mSensorFuser, SLOT(slotScanFinished(quint32)));

    // Actually invoke slotLaserScannerRelativePoseChanged() AFTER our parent
    // has connected our signals, so the values are propagated to sensorfuser.
    QTimer::singleShot(1000, this, SLOT(slotLaserScannerRelativePoseChanged()));
}

LogPlayer::~LogPlayer()
{
    delete ui;
}

void LogPlayer::slotLaserScannerRelativePoseChanged()
{
    mSensorFuser->setLaserScannerRelativePose(
                Pose(
                    QVector3D(      // Offset from Antenna to Laser Source. In Vehicle Reference Frame: Like OpenGL, red arm forward pointing to screen
                                    ui->mSpinBoxLaserScannerX->value(),
                                    ui->mSpinBoxLaserScannerY->value(),
                                    ui->mSpinBoxLaserScannerZ->value()),
                    ui->mSpinBoxLaserScannerYaw->value(),
                    ui->mSpinBoxLaserScannerPitch->value(),
                    ui->mSpinBoxLaserScannerRoll->value(),
                    10)             // Use 10 msec TOW, so that the relative pose is always older than whatever new pose coming in. Don't use 0, as that would be set to current TOW, which might be newer due to clock offsets.
                );
}

bool LogPlayer::slotOpenLogFiles()
{
    QString logFileName;
    QString fileNameOfNextLogFile;

    logFileName = QFileDialog::getOpenFileName(this, "Select SBF log", QString(), "SBF Data (*.sbf)");
    if(logFileName.isEmpty()) return false;

    QFile logFileSbf(logFileName);
    if(!logFileSbf.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open SBF log file");
        QMessageBox::critical(this, "Error opening file", "Unable to open SBF log file");
        return false;
    }

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading SBF log file %1...").arg(logFileName));
    // Fill the sbf backup copy. The real data will be filled in slotRewind()
    mDataSbfCopy = logFileSbf.readAll();

    // We try to open the laser log. But even if this fails, do not abort, as SBF only is still something we can work with for playing back
    fileNameOfNextLogFile = logFileName.replace("gnssdata.sbf", "scannerdata.lsr");
    logFileName = QFileDialog::getOpenFileName(this, "Select laser log", fileNameOfNextLogFile, "Laser Data (*.lsr)");
    QFile logFileLaser(logFileName);
    if(!logFileLaser.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Unable to open Laser log file %1").arg(logFileName));
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading Laser log file %1...").arg(logFileName));
        mDataLaser = logFileLaser.readAll();
    }

    // We try to open the flightcontroller log. But even if this fails, do not abort, as SBF only is still something we can work with for playing back
    fileNameOfNextLogFile = logFileName.replace("scannerdata.lsr", "flightcontroller.flt");
    logFileName = QFileDialog::getOpenFileName(this, "Select flightcontroller log", fileNameOfNextLogFile, "Flightcontroller Log Data (*.flt)");
    QFile logFileFlightController(logFileName);
    if(!logFileFlightController.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Unable to open FlightController log file %1").arg(logFileName));
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading FlightController log file %1...").arg(logFileName));
        mDataFlightController = logFileFlightController.readAll();
    }

    slotRewind();

    return true;
}

void LogPlayer::slotRewind()
{
    // The file doesn't always start with valid sbf, so lets seek to the first packet
    mDataSbf = mDataSbfCopy;
    mIndexLaser = 0;
    mIndexFlightController = 0;
    mTimePlaybackStartReal = QTime(); // set invalid

    // Whats the minimum TOW in our data? Set progressbar accordingly
    qint32 towStart;
    getNextDataSource(&towStart);

    // Whats the maximum TOW in our data? Set progressbar accordingly
    const qint32 towStop = std::max(getLastTow(Source_Sbf), std::max(getLastTow(Source_Laser), getLastTow(Source_FlightController)));

    qDebug() << "LogPlayer::slotRewind(): this file contains data between" << towStart << "and" << towStop << "- length in seconds:" << (towStop - towStart)/1000;
    ui->mProgressBarTow->setRange(towStart, towStop);
    ui->mProgressBarTow->setValue(towStart);
}

qint32 LogPlayer::getLastTow(const DataSource& source)
{
    qint32 tow = -1;
    switch(source)
    {

    case Source_Sbf:
    {
        qint32 lastSearchIndex = -1;

        // Search backwards as long as we cannot find a valid SBF packe to extract TOW from
        while(tow < 0)
        {
            QByteArray lastPacketSbf = mDataSbf.right(mDataSbf.size() - mDataSbf.lastIndexOf("$@", lastSearchIndex));
            mSbfParser->getNextValidPacketInfo(lastPacketSbf, 0, &tow);

            // Where to search next if TOW couldn't be extracted
            lastSearchIndex = mDataSbf.size() - lastPacketSbf.size() - 2;
        }
    }
    break;

    case Source_Laser:
    {
        const QByteArray magic("LASER");
        const int lastPos = mDataLaser.lastIndexOf(magic);
        qDebug() << "LogPlayer::getLastTow(): last laser packet startpos is" << lastPos;

        if(lastPos == -1) return -1;

        tow = *((qint32*)(mDataLaser.data() + lastPos + 5 + sizeof(quint16)));

        qDebug() << "LogPlayer::getLastTow(): last laser tow is" << tow;
    }
    break;

    case Source_FlightController:
    {
        const QByteArray lastLine = mDataFlightController.right(mDataFlightController.size() - mDataFlightController.lastIndexOf("\n", mDataFlightController.lastIndexOf("\n") - 1) - 1);

        // If we can extract something, fine. If not, we might just not have any data at all. In that case we'll return -1
        if(lastLine.size())
            tow = lastLine.split(' ').at(0).toInt();

        if(tow == 0) tow = -1;
    }
    break;

    default:
        Q_ASSERT("unknown datasource!");
        break;


    }

    return tow;
}

qint32 LogPlayer::getNextTow(const DataSource& source)
{
    qint32 tow = -1;

    switch(source)
    {

    case Source_Sbf:
    {
        if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &tow))
            tow = -1;
    }
    break;

    case Source_Laser:
    {
        // 5 bytes is magic packet LASER, 2 is two bytes packetlength
        tow = *((qint32*)(mDataLaser.data() + mIndexLaser + 5 + sizeof(quint16)));
    }
    break;

    case Source_FlightController:
    {
        const QByteArray nextPacket = getNextPacket(source);

        // If we can extract something, fine. If not, we might just not have any laser data at all. In that case we'll return -1
        if(nextPacket.size())
            tow = nextPacket.split(' ').at(0).toInt();

        if(tow == 0) tow = -1;
    }
    break;

    default:
        Q_ASSERT("datasource not implemented");
        break;
    }

    return tow;
}

QByteArray LogPlayer::getNextPacket(const DataSource& source)
{
    QByteArray result;

    switch(source)
    {

    case Source_Laser:
    {
        // check uninitialized and out-of-bounds conditions
        if(mIndexLaser >= mDataLaser.size() || !mDataLaser.size())
            return result;

        const quint16 length = *((quint16*)(mDataLaser.constData() + mIndexLaser + 5));

        result = mDataLaser.mid(mIndexLaser, length);

        Q_ASSERT(result.size() == length);
    }
    break;

    case Source_FlightController:
    {
        // check uninitialized and out-of-bounds conditions
        if(mIndexFlightController >= mDataFlightController.size() || !mDataFlightController.size())
            return result;

        const int indexEndOfNextPacketLaser = mDataFlightController.indexOf("\n", mIndexFlightController);
        if(indexEndOfNextPacketLaser > 0)
            result = QByteArray(mDataFlightController.data() + mIndexFlightController, indexEndOfNextPacketLaser - mIndexFlightController);

        Q_ASSERT(mIndexFlightController == 0 || mDataFlightController.at(mIndexFlightController-1) == '\n');
        Q_ASSERT(!result.contains("\n") && "contains a newline!");
    }
    break;

    default:
        Q_ASSERT("datasource not implemented");
        break;
    }

    return result;
}

LogPlayer::DataSource LogPlayer::getNextDataSource(qint32* tow)
{
    // Retrieve TOWs from all devices to figure out who to process next
    qint32 towSbf, towLaser, towFlightController;
    const qint32 big = 2147483647; // thats a tow 24 days into the week, so we're safe

    towSbf = getNextTow(Source_Sbf);
    towSbf = towSbf < 0 ? towSbf = big : towSbf;

    towLaser = getNextTow(Source_Laser);
    towLaser = towLaser < 0 ? towLaser = big : towLaser;

    towFlightController = getNextTow(Source_FlightController);
    towFlightController = towFlightController < 0 ? towFlightController = big : towFlightController;

    qint32 towMin = std::min(std::min(towSbf, towLaser), towFlightController);

    if(tow) *tow = towMin == big ? -1 : towMin;

    if(towMin == towSbf)
    {
        return Source_Sbf;
    }
    else if(towMin == towLaser)
    {
        return Source_Laser;
    }
    else if(towMin == towFlightController)
    {
        return Source_FlightController;
    }
    else
    {
        return Source_Invalid;
    }
}

bool LogPlayer::slotStepForward()
{
    DataSource source = getNextDataSource();

    switch(source)
    {
    case Source_Sbf:
    {
        // process SBF
        mSbfParser->processNextValidPacket(mDataSbf);
    }
    break;

    case Source_Laser:
    {
        // process laser data
        QByteArray packet = getNextPacket(Source_Laser);
        mIndexLaser += packet.size();
        processPacket(source, packet);
    }
    break;

    case Source_FlightController:
    {
        // process FlightController data
        QByteArray packet = getNextPacket(Source_FlightController);
        mIndexFlightController += packet.size() + 1; // skip the newline (\n)
        processPacket(source, packet);
    }
    break;

    default:
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further Sbf, FlightControlle or Laser packets from logs";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached end of log data.");
        return false;
    }
    break;

    }

    return true;
}

void LogPlayer::processPacket(const LogPlayer::DataSource& source, const QByteArray& packet)
{
    switch(source)
    {
    case Source_Laser:
    {
        const quint16 length = *((quint16*)(packet.constData() + 5));

        const qint32 tow = *((qint32*)(packet.constData() + 5 + sizeof(length)));

        const quint16 indexStart = *((quint16*)(packet.constData() + 5 + sizeof(length) + sizeof(tow)));

        std::vector<long>* data = new std::vector<long>;

        for(int i=0;i<indexStart;i++)
            data->push_back(1);

        const quint16 rayBytes = length
                -5                // LASER
                - sizeof(quint16) // length at beginning
                - sizeof(qint32)  // tow
                - sizeof(quint16);// indexStart

        const quint16 numRaysSaved = rayBytes / sizeof(quint16);

        for(int i=0;i<numRaysSaved;i++)
        {
            const quint16 distance = *((quint16*)(packet.constData() + 5 + sizeof(length) + sizeof(tow) + sizeof(indexStart) + (i*sizeof(quint16))));
            data->push_back(distance);
        }

        mSensorFuser->slotNewScanData(tow, data);
    }
    break;

    case Source_FlightController:
    {
        FlightControllerValues fcv(packet);
        emit flightState(fcv.flightState);
        emit flightControllerValues(fcv);
    }
    break;
    }

}

void LogPlayer::slotPlay()
{
    if(ui->mPushButtonPlay->isChecked())
    {
        qint32 minTowBefore;
        getNextDataSource(&minTowBefore);

        if(!mTimePlaybackStartReal.isValid())
        {
            // The first time slotPlay() has been called after loading/rewinding logdata.
            // Take note of GNSS-TOW and real time, so we can synchronize them
            mTimePlaybackStartReal = QTime::currentTime();
            mTimePlaybackStartTow = minTowBefore;
        }

        if(!slotStepForward())
        {
            // Playback failed, we're probably at the end
            qDebug() << "LogPlayer::slotPlay(): slotStepForward() failed, we're probably at the end, stopping timer.";
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Stepping forward failed, stopping playback timer.");
            mTimerAnimation->stop();
            ui->mPushButtonPlay->setChecked(false);
            return;
        }

        qint32 minTowAfter;
        getNextDataSource(&minTowAfter);

        mTimerAnimation->stop();

        if(minTowAfter > 0)
        {
            // Packets in the SBF stream are not guaranteed to be in chronological order, especially
            // ExtEvent-packets don't let this assumption hold. For this reason, we might have to deal
            // with negative intervals, which we just set to 0 here.

            qint32 towElapsedAtNextPacket = minTowAfter - mTimePlaybackStartTow;
            QTime timeOfNextPacketReal = mTimePlaybackStartReal.addMSecs(towElapsedAtNextPacket);
            const qint32 timeToSleep = QTime::currentTime().msecsTo(timeOfNextPacketReal) * ui->mSpinBoxTimeFactor->value();

            //qDebug() << "LogPlayer::slotPlay(): slotStepForward() succeeded, sleeping for" << timeToSleep;

            // Wait between 0 and 1 secs, scaled by timefactor
            mTimerAnimation->start(qBound(0, timeToSleep, 5000));
        }
        else
        {
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Playback reached end of logfile, stopping.");
            qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next tow is -1";
            ui->mPushButtonPlay->setChecked(false);
        }
    }
    else
    {
        mTimePlaybackStartReal = QTime();
        mTimerAnimation->stop();
    }
}

void LogPlayer::slotNewSbfTime(QByteArray,qint32 tow)
{
    ui->mProgressBarTow->setValue(tow);
}
