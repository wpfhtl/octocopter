#include "ptucontroller.h"
#include "ui_ptucontroller.h"

PtuController::PtuController(const QString& deviceFile, QWidget *parent) : QDockWidget(parent), ui(new Ui::PtuController)
{
    ui->setupUi(this);

    // Connect UI...
    connect(ui->mPushButtonSetPositionCamera, SIGNAL(clicked()), SLOT(slotSetPositionCamera()));
    connect(ui->mPushButtonSetPositionLens, SIGNAL(clicked()), SLOT(slotSetPositionFrustumCenter()));

    mSerialPortPtu = new AbstractSerial();
    mSerialPortPtu->enableEmitStatus(true);
    connect(mSerialPortPtu, SIGNAL(signalStatus(QString,QDateTime)), SLOT(slotSerialPortStatusChanged(QString,QDateTime)));
    mSerialPortPtu->setDeviceName(deviceFile);
    if(!mSerialPortPtu->open(AbstractSerial::ReadWrite))
    {
        mSerialPortPtu->close();
        qFatal("GpsDevice::GpsDevice(): Opening serial usb port %s failed, exiting.", qPrintable(deviceFile));
    }
    mSerialPortPtu->setBaudRate(AbstractSerial::BaudRate9600); // PTU can go faster, but do we need to?
    mSerialPortPtu->setDataBits(AbstractSerial::DataBits8);
    mSerialPortPtu->setParity(AbstractSerial::ParityNone);
    mSerialPortPtu->setStopBits(AbstractSerial::StopBits1);
    mSerialPortPtu->setFlowControl(AbstractSerial::FlowControlOff);
    connect(mSerialPortPtu, SIGNAL(readyRead()), SLOT(slotDataReady()));

    mTimerUpdateStatus = new QTimer(this);
    mTimerUpdateStatus->start(5000);
    connect(mTimerUpdateStatus, SIGNAL(timeout()), SLOT(slotRetrieveStatus()));

    QTimer::singleShot(1000, this, SLOT(slotTryConnecting()));
}

PtuController::~PtuController()
{
    slotSendCommandToPtu("H"); // send halt
    delete ui;
    mSerialPortPtu->deleteLater();
}

void PtuController::slotSetSpeed(Axis axis, quint16 speed)
{
    const QString command = axis == AXIS_PAN ? "PS" : "TS";
    slotSendCommandToPtu(command + QString::number(speed));
}

void PtuController::slotRetrieveStatus()
{
    slotSendCommandToPtu("O");
}

void PtuController::slotSendCommandToPtu(const QString& command)
{
    mSerialPortPtu->write(QString(command + "\r\n").toAscii());
}

void PtuController::slotInitialize()
{
    // Enable host command echoing EE, disable ED
    slotSendCommandToPtu("EE");

    // Get firmware version
    slotSendCommandToPtu("V");

    // Set hold power: (R)egular, (L)ow, (O)ff
    slotSendCommandToPtu("PHO"); //PanHold PowerMode: Off
    slotSendCommandToPtu("THL"); //TiltHold PowerMode: Off

    // Set move power: (H)igh, (R)egular, (L)ow, (O)ff
    // WARNING: Don't use High, manual says its not good
    slotSendCommandToPtu("PML"); //PanHold PowerMode: Off
    slotSendCommandToPtu("TML"); //TiltHold PowerMode: Off

    // Set step size
    slotSendCommandToPtu("");

    // Get pan and tilt resolution
    slotSendCommandToPtu("PR");
    slotSendCommandToPtu("TR");

    // Set tilt position limits (don't damage the PTU itself or the camera or the lens)
    slotSendCommandToPtu("");

    // Enable continuous pan mode
    slotSendCommandToPtu("PCE");

    // Go to immediate mode
    slotSendCommandToPtu("I");

    // Enable factory limits
    slotSendCommandToPtu("LE");
}

void PtuController::slotSerialPortStatusChanged(const QString& status, const QDateTime& time)
{
     qDebug() << "PtuController::slotSerialPortStatusChanged(): usb port status" << status << "errorstring" << mSerialPortPtu->errorString();
}

void PtuController::slotVehiclePoseChanged(const Pose& pose)
{
    mLastKnownVehiclePose = pose;

    if(ui->mPushButtonToggleControllerState->isChecked() && mSerialPortPtu->isOpen())
    {
        // make ptu point at pose
    }
}

void PtuController::slotSetPosition(float degreePan, float degreeTilt)
{

}

void PtuController::slotSetPositionCamera()
{
    mPositionCameraSensor = mLastKnownVehiclePose.position;
    if(!mPositionInFrustumCenter.isNull()) determinePtuPose();
}

void PtuController::slotSetPositionFrustumCenter()
{
    mPositionInFrustumCenter = mLastKnownVehiclePose.position;
    if(!mPositionCameraSensor.isNull()) determinePtuPose();
}

void PtuController::determinePtuPose()
{
    // Both mPositionCameraSensor and mPositionInFrustumCenter are known, so we have a ray defining the camera setup
}

void PtuController::slotDataReady()
{
    mDataFromPtu.append(mSerialPortPtu->readAll());
    qDebug() << "PtuController::slotDataReady(): receive buffer contains:" << mDataFromPtu;

    // process replies, but I'll have to see those first.
}
