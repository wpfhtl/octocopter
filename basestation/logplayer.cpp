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

    QLocale german(QLocale::German);
    ui->mProgressBarTow->setLocale(german);

    mIndexLaser = -1;
    mIndexFlightController = -1;

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonGoTo, SIGNAL(clicked()), SLOT(slotGoTo()));
    connect(ui->mPushButtonPlay, SIGNAL(clicked()), SLOT(slotPlay()));
    connect(ui->mPushButtonStepForward, SIGNAL(clicked()), SLOT(slotStepForward()));

    connect(ui->mSpinBoxLaserScannerX, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerY, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerZ, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerYaw, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerPitch, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));
    connect(ui->mSpinBoxLaserScannerRoll, SIGNAL(valueChanged(double)), SLOT(slotLaserScannerRelativePoseChanged()));

    // emit fused lidarpoints
    connect(mSensorFuser, SIGNAL(newScannedPoints(const QVector<QVector3D>* const, const QVector3D* const)), SIGNAL(scanData(const QVector<QVector3D>* const, const QVector3D* const)));

    connect(mSbfParser, SIGNAL(status(const GnssStatus* const)), SIGNAL(gnssStatus(const GnssStatus* const)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePose(const Pose* const)), SIGNAL(vehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(newVehiclePoseSensorFuser(const Pose* const)), mSensorFuser, SLOT(slotNewVehiclePose(const Pose* const)));
    connect(mSbfParser, SIGNAL(processedPacket(QByteArray,qint32)), SLOT(slotNewSbfTime(QByteArray,qint32)));
    connect(mSbfParser, SIGNAL(scanFinished(const quint32&)), mSensorFuser, SLOT(slotScanFinished(const quint32&)));

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
        qDebug() << "LogPlayer::getLastTow(): last laser packet startpos is" << lastPos;

        if(lastPos == -1) return -1;

        tow = *((qint32*)(mDataLaser.data() + lastPos + 5 + sizeof(quint16)));

        qDebug() << "LogPlayer::getLastTow(): last laser tow is" << tow;
    }
    break;

    case Source_FlightController:
    {
        qint32 lastSearchIndex = -1;

        // Search backwards as long as we cannot find a valid SBF packe to extract TOW from
        while(lastSearchIndex > -2 && tow < 0)
        {
            QByteArray lastPacket = mDataFlightController.right(mDataFlightController.size() - mDataFlightController.lastIndexOf("FLTCLR", lastSearchIndex));

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

bool LogPlayer::slotStepForward(DataSource source)
{
    if(source == Source_Invalid)
        source = getNextDataSource();

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
        mIndexFlightController += packet.size();
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

//        qDebug() << "LASER:" << tow;

        mSensorFuser->slotNewScanData(tow, data);

        ui->mProgressBarTow->setValue(tow);
    }
    break;

    case Source_FlightController:
    {
        FlightControllerValues* fcv = (FlightControllerValues*)(packet.data() + 6); // FLTCLR
        ui->mProgressBarTow->setValue(mFlightControllerValues.timestamp);

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

        qDebug() << "FLTCLR:" << fcv->timestamp;

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
        }

        if(!slotStepForward(ds))
        {
            // Playback failed, we're probably at the end
            qDebug() << "LogPlayer::slotPlay(): slotStepForward() failed, we're probably at the end, stopping timer.";
            emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Stepping forward failed, stopping playback timer.");
            mTimerAnimation->stop();
            ui->mPushButtonPlay->setChecked(false);
            return;
        }

        qint32 minTowAfter;
        DataSource s = getNextDataSource(&minTowAfter);

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

void LogPlayer::slotGoTo()
{
    bool ok = false;
    const int towTarget = QInputDialog::getInt(
                this,
                "Go To TOW", "Where do you want to seek to?",
                ui->mProgressBarTow->value(),
                ui->mProgressBarTow->minimum(),
                ui->mProgressBarTow->maximum(),
                1000,
                &ok,
                /*Qt::WindowFlags flags = */0 );

    if(!ok) return;

    // We want to point all data-cursors to the first packet with TOW >= towTarget.

    // First, do SBF:
    qint32 indexSbf = mDataSbfCopy.size() / 2;
    qint32 stepSizeSbf = indexSbf / 2;
    qint32 towSbf0, towSbf1, towSbf2;
    qDebug() << "LogPlayer::slotGoTo(): towTgt is" << towTarget;
    while(mSbfParser->getNextValidPacketInfo(mDataSbfCopy.mid(indexSbf, 2000), 0, &towSbf0))
    {
        qDebug() << "LogPlayer::slotGoTo(): stepsize is" << stepSizeSbf << "indexSbf is" << indexSbf << "of" << mDataSbfCopy.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(towSbf0 == towTarget || (towSbf1 != towSbf0 && towSbf0 == towSbf2))
        {
            break;
        }

        if(towSbf0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): back, towSbf is" << towSbf0;
            indexSbf = qBound(0, indexSbf - stepSizeSbf, mDataSbfCopy.size());
        }
        else if(towSbf0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): frwd, towSbf is" << towSbf0;
            indexSbf = qBound(0, indexSbf + stepSizeSbf, mDataSbfCopy.size());
        }

        towSbf2 = towSbf1;
        towSbf1 = towSbf0;

        // Step at least one packet, which is at least 16 bytes
        stepSizeSbf = std::max(16, stepSizeSbf/2);
    }

    mDataSbf = mDataSbfCopy.mid(indexSbf);
    mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towSbf0);
    qDebug() << "LogPlayer::slotGoTo(): SBF: reached TOW" << towSbf0 << ", targeted was" << towTarget;

    // LASER
    mIndexLaser = mDataLaser.size() / 2;
    qint32 stepSizeLaser = mIndexLaser / 2;
    qint32 towLaser0, towLaser1, towLaser2 = 0;
    while(towLaser0 >= 0)
    {
        towLaser0 = getNextTow(Source_Laser);
        qDebug() << "LogPlayer::slotGoTo(): stepsize is" << stepSizeLaser << "indexLaser is" << mIndexLaser << "of" << mDataLaser.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(towLaser0 == towTarget || (towLaser1 != towLaser0 && towLaser0 == towLaser2))
        {
            break;
        }

        if(towLaser0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): back, towLaser is" << towLaser0;
            mIndexLaser = qBound(0, mIndexLaser - stepSizeLaser, mDataLaser.size());
        }
        else if(towLaser0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): frwd, towLaser is" << towLaser0;
            mIndexLaser = qBound(0, mIndexLaser + stepSizeLaser, mDataLaser.size());
        }

        towLaser2 = towLaser1;
        towLaser1 = towLaser0;

        // Step at least one packet, which is around 1500 bytes
        stepSizeLaser = std::max(500, stepSizeLaser/2);
    }

    qDebug() << "LogPlayer::slotGoTo(): LASER: reached TOW" << towLaser0 << ", targeted was" << towTarget;







    // FlightController
    mIndexFlightController = mDataFlightController.size() / 2;
    qint32 stepSizeFltClr = mIndexFlightController / 2;
    qint32 towFltClr0, towFltClr1, towFltClr2 = 0;
    while(mDataFlightController.size() && towFltClr0 >= 0)
    {
        towFltClr0 = getNextTow(Source_FlightController);
        qDebug() << "LogPlayer::slotGoTo(): stepsize is" << stepSizeFltClr << "indexFltClr is" << mIndexFlightController << "of" << mDataFlightController.size();

        // We're done if we reached the target tow OR the tow doesn't change anymore OR we toggle between tow TOWs
        if(towFltClr0 == towTarget || (towFltClr0 == towFltClr1 && towFltClr0 == towFltClr2))
        {
            break;
        }

        if(towFltClr0 > towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): back, towFltClr is" << towFltClr0;
            mIndexFlightController = qBound(0, mIndexFlightController - stepSizeFltClr, mDataFlightController.size());
        }
        else if(towFltClr0 < towTarget)
        {
            qDebug() << "LogPlayer::slotGoTo(): frwd, towFltClr is" << towFltClr0;
            mIndexFlightController = qBound(0, mIndexFlightController + stepSizeFltClr, mDataFlightController.size());
        }

        towFltClr2 = towFltClr1;
        towFltClr1 = towFltClr0;

        // Don't let stepsize go far below packetSize
        stepSizeFltClr = std::max(200, stepSizeFltClr/2);
    }

    qDebug() << "LogPlayer::slotGoTo(): FLTCLR: reached TOW" << towFltClr0 << ", targeted was" << towTarget;




    ui->mProgressBarTow->setValue(towTarget);
}
