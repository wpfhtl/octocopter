#include "logplayer.h"
#include "ui_logplayer.h"

LogPlayer::LogPlayer(QWidget *parent) : QDockWidget(parent), ui(new Ui::LogPlayer)
{
    ui->setupUi(this);

    mTimerAnimation = new QTimer(this);
    connect(mTimerAnimation, SIGNAL(timeout()), SLOT(slotStepForward()));

    mSensorFuser = new SensorFuser;
    mSbfParser = new SbfParser(this);

    mIndexSbf = -1;
    mIndexLaser = -1;

    // Connect UI...
    connect(ui->mPushButtonOpenLogs, SIGNAL(clicked()), SLOT(slotOpenLogFiles()));
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
    const QString fileNameSbf = QFileDialog::getOpenFileName(this, "Select SBF log", QString(), "*.sbf");
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
    mIndexSbf = 0;

    const QString fileNameLaser = QFileDialog::getOpenFileName(this, "Select laser log", QString(), "*.laser");
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

void LogPlayer::slotStepForward()
{
    // check uninitialized and out-of-bounds conditions
    if(mIndexSbf < 0 || mIndexSbf >= mDataSbf.size() || !mDataSbf.size() || mIndexLaser < 0 || mIndexLaser >= mDataLaser.size() || !mDataLaser.size())
        return;

    // We're stepping forward, so feed the temporally-next data into sensorfuser.
    // First, find out whether the next data comes from SBF or LaserScanner
    qint32 towSbf = 0;
    qint32 towLaser = 0;
    QByteArray nextPacketSbf, nextPacketLaser;

    // To do this, extract the next SBF packet and its TOW
    const int indexEndOfNextPacketSbf = mDataSbf.indexOf("$@", mIndexSbf);
    if(indexEndOfNextPacketSbf > 0)
    {
        nextPacketSbf = QByteArray(mDataSbf.data() + mIndexSbf, indexEndOfNextPacketSbf - mIndexSbf);
        towSbf = mSbfParser->peekNextTow(nextPacketSbf);
    }

    // Now fetch the next LaserScan
    const int indexEndOfNextPacketLaser = mDataLaser.indexOf("\n", mIndexLaser);
    if(indexEndOfNextPacketLaser > 0)
    {
        nextPacketLaser = QByteArray(mDataLaser.data() + mIndexLaser, indexEndOfNextPacketLaser - mIndexLaser);
        bool success = false;
        towLaser = nextPacketLaser.split(' ').at(0).toInt(&success);
        Q_ASSERT(success);
    }

    if(towSbf < towLaser && towSbf != 0)
    {
        // process SBF
        mIndexSbf = indexEndOfNextPacketSbf;
        mSbfParser->processSbfData(nextPacketSbf);
    }
    else if(towLaser != 0)
    {
        // process laser data
        mIndexLaser = indexEndOfNextPacketLaser + 1; // skip the newline (\n)

        const QList<QByteArray> laserScanData = nextPacketLaser.split(' ');
        std::vector<long>* data = new std::vector<long>;
        for(int i=1;i<laserScanData.size();i++)
        {
            bool success = false;
            data->push_back(laserScanData.at(i).toInt(&success));
            if(!success) qDebug() << "LogPlayer: couldn't parse scannerdata-distance at index" << i;
        }

        mSensorFuser->slotNewScanData(laserScanData.at(0).toInt(), data);
    }
    else
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further SBF or Laser packets from logs.";
        slotPause();
    }
}

void LogPlayer::slotStepBack()
{
    // check uninitialized and out-of-bounds conditions
    if(mIndexSbf <= 0 || !mDataSbf.size() || mIndexLaser <= 0 || !mDataLaser.size())
        return;

    // We're stepping backward, so feed the temporally-previous data into sensorfuser.
    // First, find out whether the previous data comes from SBF or LaserScanner
    qint32 towSbf = 0;
    qint32 towLaser = 0;
    QByteArray previousPacketSbf, previousPacketLaser;

    // To do this, extract the previous SBF packet and its TOW
    const int indexBeginningOfPreviousPacketSbf = mDataSbf.lastIndexOf("$@", mIndexSbf);
    if(indexBeginningOfPreviousPacketSbf > 0)
    {
        previousPacketSbf = QByteArray(mDataSbf.data() + indexBeginningOfPreviousPacketSbf, mIndexSbf - indexBeginningOfPreviousPacketSbf);
        towSbf = mSbfParser->peekNextTow(previousPacketSbf);
    }

    // Now fetch the next LaserScan
    const int indexBeginningOfPreviousPacketLaser = mDataLaser.lastIndexOf("\n", mIndexLaser - 1); // the -1 is to skip the newline
    if(indexBeginningOfPreviousPacketLaser > 0)
    {
        previousPacketLaser = QByteArray(mDataLaser.data() + indexBeginningOfPreviousPacketLaser, mIndexLaser - indexBeginningOfPreviousPacketLaser);
        bool success = false;
        towLaser = previousPacketLaser.split(' ').at(0).toInt(&success);
        Q_ASSERT(success);
    }

    if(towSbf > towLaser && towSbf != 0)
    {
        // process SBF
        mIndexSbf = indexBeginningOfPreviousPacketSbf;
        mSbfParser->processSbfData(previousPacketSbf);
    }
    else if(towLaser != 0)
    {
        // process laser data
        mIndexLaser = indexBeginningOfPreviousPacketLaser + 1; // skip the newline (\n)

        const QList<QByteArray> laserScanData = previousPacketLaser.split(' ');
        std::vector<long>* data = new std::vector<long>;
        for(int i=1;i<laserScanData.size();i++)
        {
            bool success = false;
            data->push_back(laserScanData.at(i).toInt(&success));
            if(!success) qDebug() << "LogPlayer: couldn't parse scannerdata-distance at index" << i;
        }

        mSensorFuser->slotNewScanData(laserScanData.at(0).toInt(), data);
    }
    else
    {
        qDebug() << "LogPlayer::slotStepForward(): seems I'm at the end, cannot fetch further SBF or Laser packets from logs.";
        slotPause();
    }
}

void LogPlayer::slotPlay()
{
    // .....
    mTimerAnimation->setInterval(1234);
    mTimerAnimation->start();
}

void LogPlayer::slotPause()
{
    mTimerAnimation->stop();
}
