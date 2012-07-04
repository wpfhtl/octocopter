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

    connect(ui->mSliderFusionTimeOffset, SIGNAL(valueChanged(int)), SLOT(slotFusionTimeOffsetChanged(int)));

    // emit fused lidarpoints
    connect(mSensorFuser, SIGNAL(newScannedPoints(QVector<QVector3D>,QVector3D)), SIGNAL(scanData(QVector<QVector3D>,QVector3D)));

    connect(mSbfParser, SIGNAL(status(GpsStatusInformation::GpsStatus)), SIGNAL(gpsStatus(GpsStatusInformation::GpsStatus)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePoseLogPlayer(Pose)), SIGNAL(vehiclePose(Pose)));

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

void LogPlayer::slotFusionTimeOffsetChanged(const int offset)
{
    ui->mLabelFusionTimeOffset->setText(QString::number(offset).append(" ms"));
    mSensorFuser->slotSetTimeOffsetFromScanToPose(offset);
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
    const QString fileNameSbf = QFileDialog::getOpenFileName(this, "Select SBF log", QString(), "SBF Data (*.sbf)");
    if(fileNameSbf.isEmpty()) return false;

    QFile fileSbf(fileNameSbf);
    if(!fileSbf.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open SBF log file");
        QMessageBox::critical(this, "Error opening file", "Unable to open SBF log file");
        return false;
    }

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading SBF log file %1...").arg(fileNameSbf));
    // Fill the sbf backup copy. The real data will be filled in slotRewind()
    mDataSbfCopy = fileSbf.readAll();

    // We try to open the laser log. But even if this fails, do not abort, as SBF only is still something we can work with for playing back
    const QString fileNameLaser = QFileDialog::getOpenFileName(this, "Select laser log", QString(), "Laser Data (*.lsr)");
    QFile fileLaser(fileNameLaser);
    if(!fileLaser.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Unable to open Laser log file %1").arg(fileNameLaser));
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), QString("Reading Laser log file %1...").arg(fileNameLaser));
        mDataLaser = fileLaser.readAll();
    }

    slotRewind();

    return true;
}

void LogPlayer::slotRewind()
{
    // The file doesn't always start with valid sbf, so lets seek to the first packet
    mDataSbf = mDataSbfCopy;
    mIndexLaser = 0;
    mTimePlaybackStartReal = QTime(); // set invalid

    // Whats the minimum TOW in our data? Set progressbar accordingly
    qint32 towSbf = 0;

    if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towSbf))
        towSbf = 99999999;

    // getNextTowLaser() can be -1 (e.g. if no laser data present). If so, don't let that harm us
    qint32 towLaser = getNextTowLaser();
    const qint32 towStart = std::min(towSbf, towLaser < 0 ? (qint32)999999999 : towLaser);

    // Whats the maximum TOW in our data? Set progressbar accordingly
    towSbf = getLastTowSbf();
    towLaser = getLastTowLaser();
    const qint32 towStop = std::max(towSbf, towLaser);

    qDebug() << "LogPlayer::slotRewind(): this file contains data between" << towStart << "and" << towStop << "- length in seconds:" << (towStop - towStart)/1000;
    ui->mProgressBarTow->setRange(towStart, towStop);
    ui->mProgressBarTow->setValue(towStart);
}

qint32 LogPlayer::getLastTowSbf()
{
    qint32 towSbf = -1;
    qint32 lastSearchIndex = -1;

    // Search backwards as long as we cannot find a valid SBF packe to extract TOW from
    while(towSbf < 0)
    {
        QByteArray lastPacketSbf = mDataSbf.right(mDataSbf.size() - mDataSbf.lastIndexOf("$@", lastSearchIndex));
        mSbfParser->getNextValidPacketInfo(lastPacketSbf, 0, &towSbf);

        // Where to search next if TOW couldn't be extracted
        lastSearchIndex = mDataSbf.size() - lastPacketSbf.size() - 2;
    }

    return towSbf;
}

qint32 LogPlayer::getNextTowLaser()
{
    qint32 tow = -1;

    const QByteArray nextPacketLaser = getNextPacketLaser();

    // If we can extract something, fine. If not, we might just not have any laser data at all. In that case we'll return -1
    if(nextPacketLaser.size())
        tow = nextPacketLaser.split(' ').at(0).toInt();

    if(tow == 0)
        return -1;
    else
        return tow;
}

qint32 LogPlayer::getLastTowLaser()
{
    qint32 tow = -1;

    const QByteArray lastLaserLine = mDataLaser.right(mDataLaser.size() - mDataLaser.lastIndexOf("\n", mDataLaser.lastIndexOf("\n") - 1) - 1);

    // If we can extract something, fine. If not, we might just not have any laser data at all. In that case we'll return -1
    if(lastLaserLine.size())
        tow = lastLaserLine.split(' ').at(0).toInt();

    if(tow == 0)
        return -1;
    else
        return tow;
}

QByteArray LogPlayer::getNextPacketLaser()
{
//    qDebug() << "LogPlayer::getPacket(): index sbf:" << mIndexSbf << "lsr:" << mIndexLaser;
    QByteArray result;

    // check uninitialized and out-of-bounds conditions
    if(mIndexLaser >= mDataLaser.size() || !mDataLaser.size())
        return result;

    const int indexEndOfNextPacketLaser = mDataLaser.indexOf("\n", mIndexLaser);
    if(indexEndOfNextPacketLaser > 0)
        result = QByteArray(mDataLaser.data() + mIndexLaser, indexEndOfNextPacketLaser - mIndexLaser);

    Q_ASSERT(mIndexLaser == 0 || mDataLaser.at(mIndexLaser-1) == '\n');
    Q_ASSERT(!result.contains("\n") && "contains a newline!");

    return result;
}

bool LogPlayer::slotStepForward()
{
    // Retrieve TOWs from scanner and gnss device to figure out who to process next
    qint32 towSbf;

    if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towSbf))
        towSbf = -1;

    qint32 towLaser = getNextTowLaser();

    if(towSbf > 0 && (towSbf < towLaser || towLaser == -1))
    {
        // process SBF
        mSbfParser->processNextValidPacket(mDataSbf);
    }
    else if(towLaser >= 0)
    {
        // process laser data
//        qDebug() << "LogPlayer::slotStepForward(): processing LSR data from TOW" << towLaser;
        QByteArray packetLaser = getNextPacketLaser();
        mIndexLaser += packetLaser.size() + 1; // skip the newline (\n)
        processLaserData(packetLaser);
    }
    else
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further SBF or Laser packets from logs, towSbf:" << towSbf << "towLaser:" << towLaser;
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached end of log data.");
        return false;
    }

    return true;
}

void LogPlayer::processLaserData(const QByteArray& packetLaser)
{
    const QList<QByteArray> laserScanData = packetLaser.split(' ');
    Q_ASSERT(laserScanData.at(0).size() == 9 && "TOW incorrect for scan!");
    std::vector<long>* data = new std::vector<long>;
    for(int i=1;i<laserScanData.size();i++)
    {
        bool success = false;
        data->push_back(laserScanData.at(i).toInt(&success));
        if(!success) qDebug() << "LogPlayer: couldn't parse scannerdata-distance at index" << i << ", value was:" << laserScanData.at(i);

        // FIXME: This method was changed to emit lengths of 10m for every ray, for debugging.
        //data->push_back(1000); success = true;

        // FIXME: This method was changed to emit raylengths simulating flaying over a flat ground in 5m height.
        //float angle = (540.0f - i) / 4.0f;
        //float distance = 5.0f / cos(DEG2RAD(angle));
        //data->push_back(distance * 1000);
    }

    mSensorFuser->slotNewScanData(laserScanData.at(0).toInt(), data);
}

qint32 LogPlayer::getEarliestValidTow(const qint32& towA, const qint32& towB) const
{
    if(towA <= 0)
        return towB;
    else if(towB <= 0)
        return towA;
    else
        return std::min(towA, towB);
}

void LogPlayer::slotPlay()
{
    if(ui->mPushButtonPlay->isChecked())
    {
        qint32 towBeforeSbf;
        if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towBeforeSbf)) towBeforeSbf = -1;
        const qint32 minTowBefore = getEarliestValidTow(towBeforeSbf, getNextTowLaser());

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

        qint32 towAfterSbf;
        if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towAfterSbf)) towAfterSbf = -1;
        const qint32 minTowAfter = getEarliestValidTow(towAfterSbf, getNextTowLaser());

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
            qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next sbf tow" << towAfterSbf << "next lsr tow" << getNextTowLaser();
            ui->mPushButtonPlay->setChecked(false);
        }
    }
    else
    {
        mTimePlaybackStartReal = QTime();
        mTimerAnimation->stop();
    }
}
/*
void LogPlayer::slotPause()
{
    mTimePlaybackStartReal = QTime();
    mTimerAnimation->stop();
}*/

void LogPlayer::slotNewSbfTime(QByteArray,qint32 tow)
{
    ui->mProgressBarTow->setValue(tow);
}
