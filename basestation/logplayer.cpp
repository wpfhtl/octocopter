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

bool LogPlayer::slotOpenLogFiles()
{
    const QString fileNameSbf = QFileDialog::getOpenFileName(this, "Select SBF log", QString(), "*.sbf");

    QFile fileSbf(fileNameSbf);
    if(!fileSbf.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open SBF log file");
        QMessageBox::critical(this, "Error opening file", "Unable to open SBF log file");
        return false;
    }

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reading SBF log file...");
    mDataSbf = fileSbf.readAll();

    const QString fileNameLaser = QFileDialog::getOpenFileName(this, "Select laser log", QString(), "*.laser");
    QFile fileLaser(fileNameLaser);
    if(!fileLaser.open(QIODevice::ReadOnly))
    {
        emit message(Error, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Unable to open Laser log file");
        QMessageBox::critical(this, "Error opening file", "Unable to open Laser log file");
        mDataSbf.clear();
        return false;
    }

    emit message(Information, QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__), "Reading Laser log file...");
    mDataLaser = fileLaser.readAll();

    return true;
}

void LogPlayer::slotStepForward()
{
    // check uninitialized and out-of-bounds conditions
    if(mIndexSbf < 0 || mIndexSbf >= mDataSbf.size() || !mDataSbf.size() || mIndexLaser < 0 || mIndexLaser >= mDataLaser.size() || !mDataLaser.size())
        return;

    // We're stepping forward, so feed the temporally-next data into sensorfuser.
}

void LogPlayer::slotStepBack()
{
    // check uninitialized and out-of-bounds conditions
    if(mIndexSbf <= 0 || !mDataSbf.size() || mIndexLaser <= 0 || !mDataLaser.size())
        return;
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
