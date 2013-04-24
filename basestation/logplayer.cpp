#include "logplayer.h"
#include "ui_logplayer.h"
#include <QMenu>

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);
    mProgressBarTow = new ProgressBar;
    ui->verticalLayout->insertWidget(2, mProgressBarTow);

    //setMaximumSize(minimumSize());

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotPlay()));

    mSensorFuser = new SensorFuser(1);
    //mSensorFuser->setMaximumFusableRayLength(20.0f);
    mSbfParser = new SbfParser(this);

    mIndexLaser = -1;
    mIndexFlightController = -1;

    // Allow user to choose to step until a specific datasource was processed
    mStepUntilDataSource = DataSource::Source_Invalid;
    mStepSignalMapper = new QSignalMapper(this);
    mStepMenu = new QMenu(this);

    mStepMenu->addAction("Any", mStepSignalMapper, SLOT(map()));
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)DataSource::Source_Invalid);

    mStepMenu->addAction("GNSS", mStepSignalMapper, SLOT(map()));
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)DataSource::Source_Sbf);

    mStepMenu->addAction("LIDAR");
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)DataSource::Source_Laser);

    mStepMenu->addAction("FlightController");
    connect(mStepMenu->actions().last(), SIGNAL(triggered()), mStepSignalMapper, SLOT(map()));
    mStepSignalMapper->setMapping(mStepMenu->actions().last(), (int)DataSource::Source_FlightController);

    ui->mPushButtonStepForward->setMenu(mStepMenu);
    connect(mStepSignalMapper, SIGNAL(mapped(int)), this, SLOT(slotStepDataSourceChanged(int)));

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonGoTo, SIGNAL(clicked()), SLOT(slotGoToTow()));
    connect(ui->mPushButtonPlay, SIGNAL(clicked()), SLOT(slotPlay()));
    connect(ui->mPushButtonStepForward, SIGNAL(clicked()), SLOT(slotStepUntilDataSourceProcessed()));

    connect(mProgressBarTow, SIGNAL(seekToTow(qint32)), SLOT(slotGoToTow(qint32)));

    connect(ui->mSpinBoxLaserScannerX, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerY, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerZ, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerYaw, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerPitch, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerRoll, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));

    // emit fused lidarpoints
    connect(mSensorFuser, SIGNAL(scanData(float*const,quint32,QVector3D*const)), SIGNAL(scanData(float*const,quint32,QVector3D*const)));

    // For visualizing interpolated poses - disabled by default
    connect(mSensorFuser, SIGNAL(vehiclePose(Pose*const)), SIGNAL(vehiclePose(Pose*const)));

    connect(mSbfParser, SIGNAL(status(const GnssStatus* const)), SIGNAL(gnssStatus(const GnssStatus* const)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePose(const Pose* const)), SIGNAL(vehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(newVehiclePoseSensorFuser(const Pose* const)), mSensorFuser, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(processedPacket(qint32,const char*,quint16)), SLOT(slotNewSbfTime(qint32,const char*,quint16)));
    connect(mSbfParser, SIGNAL(scanFinished(const quint32&)), mSensorFuser, SLOT(slotScanFinished(const quint32&)));

    // Actually invoke slotLaserScannerRelativePoseChanged() AFTER our parent
    // has connected our signals, so the values are propagated to sensorfuser.
    QTimer::singleShot(1000, this, SLOT(slotLaserScannerRelativePoseChanged()));
}

LogPlayer::~LogPlayer()
{
    delete mSensorFuser;
    delete ui;
    delete mStepMenu;
    delete mStepSignalMapper;
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
//    if(logFileName.isEmpty()) return false;

    QFile logFileSbf(logFileName);
    if(!logFileSbf.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open SBF log file");
//        QMessageBox::critical(this, "Error opening file", "Unable to open SBF log file");
//        return false;
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading SBF log file %1...").arg(logFileName));
        // Fill the sbf backup copy. The real data will be filled in slotRewind()
        mDataSbfCopy = logFileSbf.readAll();
    }

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
    fileNameOfNextLogFile = fileNameOfNextLogFile.replace("scannerdata.lsr", "flightcontroller.flt");
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
    mProgressBarTow->setRange(towStart, towStop);
    mProgressBarTow->setValue(towStart);
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
        while(lastSearchIndex > -2 && tow < 0)
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
//        qDebug() << "LogPlayer::getLastTow(): last laser packet startpos is" << lastPos;

        if(lastPos == -1) return -1;

        tow = *((qint32*)(mDataLaser.data() + lastPos + 5 + sizeof(quint16)));

//        qDebug() << "LogPlayer::getLastTow(): last laser tow is" << tow;
    }
    break;

    case Source_FlightController:
    {
        qint32 lastSearchIndex = -1;

        // Search backwards as long as we cannot find a valid SBF packe to extract TOW from
        while(lastSearchIndex > -2 && tow < 0)
        {
            QByteArray lastPacket = mDataFlightController.right(mDataFlightController.size() - mDataFlightController.lastIndexOf("FLTCLR", lastSearchIndex));

            if(lastPacket.isEmpty()) return -1;

            FlightControllerValues* fcv = (FlightControllerValues*)(lastPacket.data() + 6); // FLTCLR
            tow = fcv->timestamp;

            // Where to search next if values couldn't be extracted
            lastSearchIndex = mDataFlightController.size() - lastPacket.size() - 6;
        }
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
        const QByteArray nextPacket = getNextPacket(source);

        if(nextPacket.size())
        {
            tow = *((qint32*)(nextPacket.data() + 5 + sizeof(quint16)));
        }
    }
    break;

    case Source_FlightController:
    {
        const QByteArray nextPacket = getNextPacket(source);

        // If we can extract something, fine. If not, we might just not have any laser data at all. In that case we'll return -1
        if(nextPacket.size())
        {
            FlightControllerValues* fcv = (FlightControllerValues*)(nextPacket.data() + 6); // magic bytes FLTCLR
            tow = fcv->timestamp;
        }
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
        mIndexLaser = mDataLaser.indexOf("LASER", mIndexLaser);

        if(mIndexLaser < 0) return result;

        const quint16 packetSize = *((quint16*)(mDataLaser.constData() + mIndexLaser + 5));

        result = mDataLaser.mid(mIndexLaser, packetSize);

//        qDebug() << "laser packet size:" << result.size();

        Q_ASSERT(result.size() == packetSize);
    }
    break;

    case Source_FlightController:
    {
        // check uninitialized and out-of-bounds conditions
        if(mIndexFlightController + sizeof(FlightControllerValues) > mDataFlightController.size() || !mDataFlightController.size())
            return result;

        // We can be called with ESTIMATED mIndexFlightController-values. So, search for the next packet's beginning
        mIndexFlightController = mDataFlightController.indexOf("FLTCLR", mIndexFlightController);
        // search for packet-end right after its beginning, otherwise we find out own beginning.
        const qint32 posPacketEnd = mDataFlightController.indexOf("FLTCLR", mIndexFlightController + 1);

        result = mDataFlightController.mid(mIndexFlightController, posPacketEnd - mIndexFlightController);
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

    //qDebug() << "LogPlayer::getNextDataSource(): sbf" << towSbf << "lidar" << towLaser << "fc" << towFlightController;

    // If twoSbf and towFlightController are equal, process sbf first, because fc is based on sbf data.
    // That just makes more sense when stepping through the data.
    if(towMin == big)
    {
        return Source_Invalid;
    }
    else if(towMin == towSbf)
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

    Q_ASSERT(false && "LogPlayer::getNextDataSource(): illegal state!");
    return Source_Invalid;
}

bool LogPlayer::slotStepUntilDataSourceProcessed()
{
    if(mStepUntilDataSource == DataSource::Source_Invalid)
    {
        slotStepForward(DataSource::Source_Invalid);
        return true;
    }

    DataSource processedSource;
    do
    {
        processedSource = slotStepForward();
    }
    while(processedSource != mStepUntilDataSource && processedSource != DataSource::Source_Invalid);
    qDebug() << "LogPlayer::slotStepUntilDataSourceProcessed(): processed datasource" << processedSource << "- done!";
}

LogPlayer::DataSource LogPlayer::slotStepForward(DataSource source)
{
    if(source == Source_Invalid) source = getNextDataSource();

    switch(source)
    {
    case Source_Sbf:
    {
        // process SBF
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "sbf";
        mSbfParser->processNextValidPacket(mDataSbf);
    }
    break;

    case Source_Laser:
    {
        // process laser data
        QByteArray packet = getNextPacket(Source_Laser);
        mIndexLaser += packet.size();
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "lidar";
        processPacket(source, packet);
    }
    break;

    case Source_FlightController:
    {
        // process FlightController data
        QByteArray packet = getNextPacket(Source_FlightController);
        mIndexFlightController += packet.size();
        //qDebug() << "LogPlayer::slotStepForward(): processing" << tow << "fc";
        processPacket(source, packet);
    }
    break;

    default:
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further Sbf, FlightController or Laser packets from logs";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached end of log data.");
        return DataSource::Source_Invalid;
    }
    break;

    }

    return source;
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

        const quint16 rayBytes = length
                - 5               // LASER
                - sizeof(quint16) // length of whole packet
                - sizeof(qint32)  // tow
                - sizeof(quint16);// indexStart

        std::vector<quint16>* data = new std::vector<quint16>(indexStart + (rayBytes / sizeof(quint16)), 1);

        // Copy the distances from the packet to the vector
        memcpy(
                    &data->at(indexStart),
                    packet.constData() + 5 + sizeof(length) + sizeof(tow) + sizeof(indexStart),
                    rayBytes
                    );

        emit newScanData(tow, data);
        mSensorFuser->slotNewScanData(tow, data);

        mProgressBarTow->setFormat("TOW %v L");
        mProgressBarTow->setValue(tow);
    }
    break;

    case Source_FlightController:
    {
        FlightControllerValues* fcv = (FlightControllerValues*)(packet.data() + 6); // FLTCLR
        mProgressBarTow->setFormat("TOW %v F");
        mProgressBarTow->setValue(mFlightControllerValues.timestamp);

        if(!(
                    fcv->controllerThrust.hasSameWeights(&mFlightControllerValues.controllerThrust)
                    &&
                    fcv->controllerYaw.hasSameWeights(&mFlightControllerValues.controllerYaw)
                    &&
                    fcv->controllerPitch.hasSameWeights(&mFlightControllerValues.controllerPitch)
                    &&
                    fcv->controllerRoll.hasSameWeights(&mFlightControllerValues.controllerRoll)
             ))
        {
            // mFlightControllerValues needs to be set before emitting!
            mFlightControllerValues = *fcv;
            emit flightControllerWeightsChanged();
        }

        mFlightControllerValues = *fcv;

//        qDebug() << "FLTCLR:" << fcv->timestamp;

        emit flightControllerValues(&mFlightControllerValues);
    }
    break;
    }

}

void LogPlayer::slotPlay()
{
    if(ui->mPushButtonPlay->isChecked())
    {
        qint32 minTowBefore;
        const DataSource ds = getNextDataSource(&minTowBefore);

        if(!mTimePlaybackStartReal.isValid())
        {
            // The first time slotPlay() has been called after loading/rewinding logdata.
            // Take note of GNSS-TOW and real time, so we can synchronize them
            mTimePlaybackStartReal = QTime::currentTime();
            mTimePlaybackStartTow = minTowBefore;
            qDebug() << "LogPlayer::slotPlay(): starting playback, realtime" << mTimePlaybackStartReal << "tow" << mTimePlaybackStartTow;
        }

        if(slotStepForward(ds) == DataSource::Source_Invalid)
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

            // How many milliseconds have elapsed between the first packet to be played and the upcoming one?
            qint32 towElapsedAtNextPacket = minTowAfter - mTimePlaybackStartTow;

            // When should this upcoming packet be played in real time?
            QTime timeOfNextPacketReal = mTimePlaybackStartReal.addMSecs(towElapsedAtNextPacket * ui->mSpinBoxTimeFactor->value());
            qint32 timeToSleep = QTime::currentTime().msecsTo(timeOfNextPacketReal);

//            qDebug() << "LogPlayer::slotPlay(): we are are" << mTimePlaybackStartReal.msecsTo(QTime::currentTime()) << "real ms into playing, next packet comes at" << towElapsedAtNextPacket;
//            qDebug() << "LogPlayer::slotPlay(): playback started at real" << mTimePlaybackStartReal << "tow" << mTimePlaybackStartTow << ": tow of upcoming packet is" << minTowAfter << " factor is" << ui->mSpinBoxTimeFactor->value() << "- sleeping for" << timeToSleep << "ms";

            // Bound time to wait for corner and error-cases
            mTimerAnimation->start(qBound(0, timeToSleep, 5000));
        }
        else
        {
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Playback reached end of logfile, stopping.");
            qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next tow is -1";
            ui->mPushButtonPlay->setChecked(false);

            // Make sensorFuser fuse whatever is left in its queue.
            mSensorFuser->slotFlushData();
        }
    }
    else
    {
        mTimePlaybackStartReal = QTime();
        mTimerAnimation->stop();
    }
}

void LogPlayer::slotNewSbfTime(const qint32 tow,const char*,quint16)
{
    mProgressBarTow->setFormat("TOW %v G");
    mProgressBarTow->setValue(tow);
}

void LogPlayer::slotGoToTow(qint32 towTarget)
{
    if(towTarget < 0)
    {
        bool ok = false;
        towTarget = QInputDialog::getInt(
                    this,
                    "Go To TOW", "Where do you want to seek to?",
                    mProgressBarTow->value(),
                    mProgressBarTow->minimum(),
                    mProgressBarTow->maximum(),
                    1000,
                    &ok,
                    /*Qt::WindowFlags flags = */0 );

        if(!ok) return;
    }

    // We want to point all data-cursors to the first packet with TOW >= towTarget.

    // First, do SBF:
    qint32 indexSbf = mDataSbfCopy.size() / 2;
    qint32 stepSize = indexSbf / 2;
    qint32 tow0, tow1, tow2;
    qDebug() << "LogPlayer::slotGoToTow(): towTgt is" << towTarget;
    while(mSbfParser->getNextValidPacketInfo(mDataSbfCopy.mid(indexSbf, 2000), 0, &tow0))
    {
        qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "indexSbf is" << indexSbf << "of" << mDataSbfCopy.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(tow0 == towTarget || (tow1 != tow0 && tow0 == tow2))
        {
            break;
        }

        if(tow0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): back, towSbf is" << tow0;
            indexSbf = qBound(0, indexSbf - stepSize, mDataSbfCopy.size());
        }
        else if(tow0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): frwd, towSbf is" << tow0;
            indexSbf = qBound(0, indexSbf + stepSize, mDataSbfCopy.size());
        }

        tow2 = tow1;
        tow1 = tow0;

        // Step at least one packet, which is at least 16 bytes
        stepSize = std::max(16, stepSize/2);
    }

    mDataSbf = mDataSbfCopy.mid(indexSbf);
    mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &tow0);
    qDebug() << "LogPlayer::slotGoToTow(): SBF: reached TOW" << tow0 << ", targeted was" << towTarget;

    // LASER
    mIndexLaser = mDataLaser.size() / 2;
    stepSize = mIndexLaser / 2;
    tow0 = 0; tow1 = 0; tow2 = 0;
    while(tow0 >= 0)
    {
        tow0 = getNextTow(Source_Laser);

        // We're done if
        // - we reached the target tow
        // - we toggle between TOWs
        // - we reached beginning or end of data
        if(tow0 == towTarget || (tow1 != tow0 && tow0 == tow2) || mIndexLaser == 0 || mIndexLaser == mDataLaser.size())
        {
            break;
        }

        if(tow0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): back, towLaser is" << tow0;

            // We need to go back stepsize bytes, or exactly one laser packet if stepsize
            // bytes is below the packet's size. Thus, determine the minimum size to move.
            const qint32 bytesToMoveForPreviousPacket = mIndexLaser - mDataLaser.lastIndexOf("LASER", mIndexLaser-1);
            const qint32 bytesToMove = std::max(stepSize, bytesToMoveForPreviousPacket);
            qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "bytestomove" << bytesToMove << "indexLaser is" << mIndexLaser << "of" << mDataLaser.size();
            mIndexLaser = qBound(0, mIndexLaser - bytesToMove, mDataLaser.size());
        }
        else if(tow0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): frwd, towLaser is" << tow0;

            // We need to go forward stepsize bytes, or exactly one laser packet if stepsize
            // bytes is below the packet's size. Thus, determine the minimum size to move.
            const qint32 bytesToMoveForNextPacket = mDataLaser.indexOf("LASER", mIndexLaser+1) - mIndexLaser;
            const qint32 bytesToMove = std::max(stepSize, bytesToMoveForNextPacket);
            qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "bytestomove" << bytesToMove << "indexLaser is" << mIndexLaser << "of" << mDataLaser.size();
            mIndexLaser = qBound(0, mIndexLaser + bytesToMove, mDataLaser.size());
        }

        tow2 = tow1;
        tow1 = tow0;

        // Step at least one packet, which is around 1500 bytes
        stepSize = /*std::max(500, */stepSize/2;//);
    }

    qDebug() << "LogPlayer::slotGoToTow(): LASER: reached TOW" << tow0 << ", targeted was" << towTarget;


    // FlightController
    mIndexFlightController = mDataFlightController.size() / 2;
    stepSize = mIndexFlightController / 2;
    tow0 = 0; tow1 = 0; tow2 = 0;
    while(mDataFlightController.size() && tow0 >= 0)
    {
        tow0 = getNextTow(Source_FlightController);
        qDebug() << "LogPlayer::slotGoToTow(): stepsize is" << stepSize << "indexFltClr is" << mIndexFlightController << "of" << mDataFlightController.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(tow0 == towTarget || (tow0 == tow1 && tow0 == tow2))
        {
            break;
        }

        if(tow0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): back, towFltClr is" << tow0;
            mIndexFlightController = qBound(0, mIndexFlightController - stepSize, mDataFlightController.size());
        }
        else if(tow0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoToTow(): frwd, towFltClr is" << tow0;
            mIndexFlightController = qBound(0, mIndexFlightController + stepSize, mDataFlightController.size());
        }

        tow2 = tow1;
        tow1 = tow0;

        // Don't let stepsize go far below packetSize
        stepSize = std::max(200, stepSize/2);
    }

    // Clear SensorFuser data, because otherwise the next data isn't guaranteed to come in in chronological order
    mSensorFuser->slotClearData();

    qDebug() << "LogPlayer::slotGoToTow(): FLTCLR: reached TOW" << tow0 << ", targeted was" << towTarget;

    mProgressBarTow->setValue(towTarget);
}

/*void LogPlayer::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space)
    {
        ui->mPushButtonPlay->setChecked(!ui->mPushButtonPlay->isChecked());
        slotPlay();
    }
}*/

void LogPlayer::slotStepDataSourceChanged(const int datasource)
{
    // When pressing STEP the next time, step util we process a packet from datasource
    mStepUntilDataSource = (DataSource)datasource;
    switch(mStepUntilDataSource)
    {
        case DataSource::Source_Sbf: ui->mPushButtonStepForward->setText("Step/G"); break;
        case DataSource::Source_Laser: ui->mPushButtonStepForward->setText("Step/L"); break;
        case DataSource::Source_FlightController: ui->mPushButtonStepForward->setText("Step/F"); break;
        case DataSource::Source_Invalid: ui->mPushButtonStepForward->setText("Step/A"); break;
    }

    // The menu was used to select a source. Lets process this packet!
    slotStepUntilDataSourceProcessed();
}
