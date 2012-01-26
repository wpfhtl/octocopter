#include "logplayer.h"
#include "ui_logplayer.h"

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotPlay()));

    mSensorFuser = new SensorFuser(3);
    mSbfParser = new SbfParser(this);

    mIndexSbf = -1;
    mIndexLaser = -1;

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
    connect(ui->mPushButtonRewind, SIGNAL(clicked()), SLOT(slotRewind()));
    connect(ui->mPushButtonPause, SIGNAL(clicked()), SLOT(slotPause()));
    connect(ui->mPushButtonPlay, SIGNAL(clicked()), SLOT(slotPlay()));
    connect(ui->mPushButtonStepBack, SIGNAL(clicked()), SLOT(slotStepBack()));
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

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reading SBF log file...");
    mDataSbf = fileSbf.readAll();
    // The file doesn't always start with valid sbf, so lets seek to the first packet
    mIndexSbf = mDataSbf.indexOf("$@", 0);

    const QString fileNameLaser = QFileDialog::getOpenFileName(this, "Select laser log", QString(), "Laser Data (*.lsr)");
    QFile fileLaser(fileNameLaser);
    if(!fileLaser.open(QIODevice::ReadOnly))
    {
        if(fileNameLaser.isEmpty()) return false;
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open Laser log file");
        QMessageBox::critical(this, "Error opening file", "Unable to open Laser log file");
        mDataSbf.clear();
        return false;
    }

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reading Laser log file...");
    mDataLaser = fileLaser.readAll();
    mIndexLaser = 0;

    return true;
}

void LogPlayer::slotRewind()
{
    mIndexSbf = mDataSbf.indexOf("$@", 0);
    mIndexLaser = 0;
}

qint32 LogPlayer::getPacketTow(const LogPlayer::DataSource& source, const LogPlayer::Direction& direction)
{
    qint32 tow = 0;
    if(source == DataSource_SBF)
    {
        /*const*/ QByteArray nextPacketSbf = getPacket(source, direction);
        if(nextPacketSbf.size())
            tow = mSbfParser->peekNextTow(nextPacketSbf);

        // sanity check
        if(tow < 0)
        {
//            qDebug() << "LogPlayer::getPacketTow(): tow is" << tow << "packet has"<< nextPacketSbf.size() << "bytes:" << nextPacketSbf;
//            qDebug() << "processing to see contents:";
//            mSbfParser->processSbfData(nextPacketSbf);
            //Q_ASSERT(false);
        }
    }
    else if(source == DataSource_Laser)
    {
        const QByteArray nextPacketLaser = getPacket(source, direction);
        if(nextPacketLaser.size())
            tow = nextPacketLaser.split(' ').at(0).toInt();

        // sanity check
        if(tow < 0)
        {
            qDebug() << "LogPlayer::getPacketTow(): tow is" << tow << "packet is" << nextPacketLaser;
            Q_ASSERT(false);
        }
    }

    return tow;
}

QByteArray LogPlayer::getPacket(const LogPlayer::DataSource& source, const LogPlayer::Direction& direction)
{
//    qDebug() << "LogPlayer::getPacket(): index sbf:" << mIndexSbf << "lsr:" << mIndexLaser;
    QByteArray result;

    // check uninitialized and out-of-bounds conditions
    if(
            direction == Direction_Forward &&
            ((source == DataSource_SBF && (mIndexSbf >= mDataSbf.size() || !mDataSbf.size())) || (source == DataSource_Laser && (mIndexLaser >= mDataLaser.size() || !mDataLaser.size())))
            )
        return result;

    if(
            direction == Direction_Backward &&
            ((source == DataSource_SBF && (mIndexSbf <= 0 || !mDataSbf.size())) || (source == DataSource_Laser && (mIndexLaser <= 0 || !mDataLaser.size())))
            )
        return result;

    if(source == DataSource_SBF && direction == Direction_Forward)
    {
        const int indexEndOfNextPacketSbf = mDataSbf.indexOf("$@", mIndexSbf + 1);
        if(indexEndOfNextPacketSbf > 0)
            result = QByteArray(mDataSbf.data() + mIndexSbf, indexEndOfNextPacketSbf - mIndexSbf);
    }

    if(source == DataSource_SBF && direction == Direction_Backward)
    {
        const int indexBeginningOfPreviousPacketSbf = mDataSbf.lastIndexOf("$@", mIndexSbf - 1);
        if(indexBeginningOfPreviousPacketSbf > 0)
            result = QByteArray(mDataSbf.data() + indexBeginningOfPreviousPacketSbf, mIndexSbf - indexBeginningOfPreviousPacketSbf);
    }

    if(source == DataSource_Laser && direction == Direction_Forward)
    {
        const int indexEndOfNextPacketLaser = mDataLaser.indexOf("\n", mIndexLaser);
        if(indexEndOfNextPacketLaser > 0)
            result = QByteArray(mDataLaser.data() + mIndexLaser, indexEndOfNextPacketLaser - mIndexLaser);
    }

    if(source == DataSource_Laser && direction == Direction_Backward)
    {
        const int indexBeginningOfPreviousPacketLaser = mDataLaser.lastIndexOf("\n", mIndexLaser - 2); // the -2 is to skip the newline
        if(indexBeginningOfPreviousPacketLaser >= 0)
            result = QByteArray(mDataLaser.data() + indexBeginningOfPreviousPacketLaser + 1, mIndexLaser - indexBeginningOfPreviousPacketLaser - 2);
        else
        {
            // We couldn't reverse-find a \n, this can happen if:
            //  - mIndexLaser points between first and second line
            //  - mIndexLaser is 0.
            // In the first case, return the first line, in the
            // second case, return an empty QByteArray (at the end)
            if(mIndexLaser != 0)
            {
                result = mDataLaser.left(mDataLaser.indexOf("\n")); // without \n at the end.
                Q_ASSERT(result.right(1) != "\n");
            }
        }
    }

    // sanity checks
    Q_ASSERT(mDataSbf.at(mIndexSbf) == '$' || mIndexSbf == mDataSbf.size());
    if(source == DataSource_SBF)
    {
        Q_ASSERT(result.isEmpty() || result.startsWith("$@"));
    }

    Q_ASSERT(mIndexLaser == 0 || mDataLaser.at(mIndexLaser-1) == '\n');
    if(source == DataSource_Laser && result.contains("\n"))
    {
//        qDebug() << "LogPlayer::getPacketTow():" << result;
        Q_ASSERT(false && "contains a newline!");
    }

    return result;
}

bool LogPlayer::slotStepForward()
{
    QByteArray packetSbf = getPacket(DataSource_SBF, Direction_Forward);
    QByteArray packetLaser = getPacket(DataSource_Laser, Direction_Forward);

    qint32 towSbf = mSbfParser->peekNextTow(packetSbf);
    qint32 towLaser = packetLaser.left(15).split(' ').at(0).toInt(); // tow is in first 9 bytes, use 15 to be sure. QByteArray::toInt() returns 0 on failure

    if(towSbf != 0 && (towSbf < towLaser || towLaser == 0))
    {
        // process SBF
        mIndexSbf += packetSbf.size();
        mSbfParser->processSbfData(packetSbf);
    }
    else if(towLaser != 0)
    {
        // process laser data
//        qDebug() << "LogPlayer::slotStepForward(): processing LSR data from TOW" << towLaser;
        mIndexLaser += packetLaser.size() + 1; // skip the newline (\n)

        processLaserData(packetLaser);
    }
    else if(towSbf < 0)
    {
//        qDebug() << "LogPlayer::slotStepForward(): caught negative TOW in SBF packet, ignoring packet of size" << packetSbf.size();
        mIndexSbf += packetSbf.size();
    }
    else
    {
//        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further SBF or Laser packets from logs.";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached end of log data.");
        return false;
    }

    return true;
}

bool LogPlayer::slotStepBack()
{
//    qDebug() << "stepping backwards: indexSbf" << mIndexSbf << "indexLsr" << mIndexLaser;
    QByteArray packetSbf = getPacket(DataSource_SBF, Direction_Backward);
    QByteArray packetLaser = getPacket(DataSource_Laser, Direction_Backward);

    qint32 towSbf = mSbfParser->peekNextTow(packetSbf);
    qint32 towLaser = packetLaser.left(15).split(' ').at(0).toInt(); // tow is in first 9 bytes, use 15 to be sure. QByteArray::toInt() returns 0 on failure

//    qDebug() << "stepping backwards: sbf has tow" << towSbf << "and" << packetSbf.size() << "bytes, lsr has tow" << towLaser << "and" << packetLaser.size() << "bytes";

    if(towSbf > towLaser || (towLaser == 0 && towSbf != 0))
    {
        // process SBF
//        qDebug() << "LogPlayer::slotStepForward(): processing SBF data...";
        mIndexSbf -= packetSbf.size();
        mSbfParser->processSbfData(packetSbf);
    }
    else if(towLaser != 0)
    {
        // process laser data
//        qDebug() << "LogPlayer::slotStepForward(): processing laser data...";
        mIndexLaser -= packetLaser.size() + 1;
        processLaserData(packetLaser);
    }
    else
    {
//        qDebug() << "LogPlayer::slotStepBack(): seems I'm at the start, cannot fetch further SBF or Laser packets from logs.";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reached beginning of log data.");
        slotPause();
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

qint32 LogPlayer::getSmallestValidTow(const qint32& towA, const qint32& towB)
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
    const qint32 minTowBefore = getSmallestValidTow(getPacketTow(DataSource_SBF, Direction_Forward), getPacketTow(DataSource_Laser, Direction_Forward));

    if(!slotStepForward())
    {
        // Playback failed, we're probably at the end
        qDebug() << "LogPlayer::slotPlay(): slotStepForward() failed, we're probably at the end, stopping timer.";
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Stepping forward failed, stopping playback timer.");
        mTimerAnimation->stop();
        return;
    }

    const qint32 minTowAfter = getSmallestValidTow(getPacketTow(DataSource_SBF, Direction_Forward), getPacketTow(DataSource_Laser, Direction_Forward));

    mTimerAnimation->stop();

    // There
    if(minTowAfter != 0)
    {
        // Packets in the SBF stream are not guaranteed to be in chronological order, especially
        // ExtEvent-packets don't let this assumption hold. For this reason, we might have to deal
        // with negative intervals, which we just set to 0 here.
        qDebug() << "LogPlayer::slotPlay(): slotStepForward() succeeded, sleeping from minTowBefore" << minTowBefore << "until minTowAfter" << minTowAfter;
        mTimerAnimation->setInterval(ui->mSpinBoxTimeFactor->value() * std::max(0, minTowAfter - minTowBefore));
        mTimerAnimation->start();
    }
    else
    {
        emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Playback reached end of logfile, stopping.");
        qDebug() << "LogPlayer::slotPlay(): not restarting playtimer, next sbf tow" << getPacketTow(DataSource_SBF, Direction_Forward) << "next lsr tow" << getPacketTow(DataSource_Laser, Direction_Forward);
    }
}

void LogPlayer::slotPause()
{
    mTimerAnimation->stop();
}
