#include "logplayer.h"
#include "ui_logplayer.h"

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotPlay()));

    mSensorFuser = new SensorFuser(3);
    mSbfParser = new SbfParser(this);

    mIndexLaser = -1;

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonPause, SIGNAL(clicked()), SLOT(slotPause()));
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

    connect(mSbfParser, SIGNAL(status(GpsStatusInformation::GpsStatus)), SIGNAL(gpsStatus(GpsStatusInformation::GpsStatus)));
    connect(mSbfParser, SIGNAL(message(LogImportance,QString,QString)), SIGNAL(message(LogImportance,QString,QString)));
    connect(mSbfParser, SIGNAL(newVehiclePose(Pose)), SIGNAL(vehiclePose(Pose)));

    connect(mSbfParser, SIGNAL(newVehiclePosePrecise(Pose)), mSensorFuser, SLOT(slotNewVehiclePose(Pose)));
    connect(mSbfParser, SIGNAL(newVehiclePosePrecise(Pose)), SIGNAL(vehiclePose(Pose)));
    connect(mSbfParser, SIGNAL(scanFinished(quint32)), mSensorFuser, SLOT(slotScanFinished(quint32)));

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
    if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towSbf)) towSbf = -1;

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
    qint32 towBeforeSbf;
    if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towBeforeSbf)) towBeforeSbf = -1;
    const qint32 towBeforeLsr = getNextTowLaser();
    const qint32 minTowBefore = getEarliestValidTow(towBeforeSbf, towBeforeLsr);

    if(!slotStepForward())
    {
        // Playback failed, we're probably at the end
        qDebug() << "LogPlayer::slotPlay(): slotStepForward() failed, we're probably at the end, stopping timer.";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Stepping forward failed, stopping playback timer.");
        mTimerAnimation->stop();
        return;
    }

    qint32 towAfterSbf;
    if(!mSbfParser->getNextValidPacketInfo(mDataSbf, 0, &towAfterSbf)) towAfterSbf = -1;
    const qint32 towAfterLsr = getNextTowLaser();
    const qint32 minTowAfter = getEarliestValidTow(towAfterSbf, towAfterLsr);

    mTimerAnimation->stop();

    if(minTowAfter > 0)
    {
        // Packets in the SBF stream are not guaranteed to be in chronological order, especially
        // ExtEvent-packets don't let this assumption hold. For this reason, we might have to deal
        // with negative intervals, which we just set to 0 here.
        //qDebug() << "LogPlayer::slotPlay(): slotStepForward() succeeded, sleeping from minTowBefore" << minTowBefore << "until minTowAfter" << minTowAfter;
        // Wait between 0 and 1 secs, scaled by timefactor
        mTimerAnimation->setInterval(ui->mSpinBoxTimeFactor->value() * qBound(0, minTowAfter - minTowBefore, 1000));
        mTimerAnimation->start();
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Playback reached end of logfile, stopping.");
        qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next sbf tow" << towAfterSbf << "next lsr tow" << getNextTowLaser();
    }
}

void LogPlayer::slotPause()
{
    mTimerAnimation->stop();
}
