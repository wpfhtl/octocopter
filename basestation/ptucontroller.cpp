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
        qDebug("PtuController::PtuController(): Opening serial usb port %s failed.", qPrintable(deviceFile));
        mIsOpened = false;
    }
    else
    {
        mIsOpened = true;
        mSerialPortPtu->setBaudRate(AbstractSerial::BaudRate9600); // PTU can go faster, but do we need to?
        mSerialPortPtu->setDataBits(AbstractSerial::DataBits8);
        mSerialPortPtu->setParity(AbstractSerial::ParityNone);
        mSerialPortPtu->setStopBits(AbstractSerial::StopBits1);
        mSerialPortPtu->setFlowControl(AbstractSerial::FlowControlOff);
        connect(mSerialPortPtu, SIGNAL(readyRead()), SLOT(slotDataReady()));

        mTimerUpdateStatus = new QTimer(this);
        mTimerUpdateStatus->start(5000);
        connect(mTimerUpdateStatus, SIGNAL(timeout()), SLOT(slotRetrieveStatus()));

        mPositionsPerDegreePan = 0.0f;
        mPositionsPerDegreeTilt = 0.0f;

        QTimer::singleShot(1000, this, SLOT(slotInitialize()));
    }
}

PtuController::~PtuController()
{
    //slotSendCommandToPtu("H"); // send halt
    delete ui;
    mSerialPortPtu->close();
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
    mSerialPortPtu->write(QString(command + " ").toAscii());
}

void PtuController::slotInitialize()
{
    slotSendCommandToPtu("DF");

    slotSendCommandToPtu("R");
    slotSendCommandToPtu("A");

    // Enable host command echoing EE, disable ED
    slotSendCommandToPtu("EE");

    // Enable terse mode for better parsing
    slotSendCommandToPtu("FT");

    // Get firmware version
    slotSendCommandToPtu("V");

    // Set hold power: (R)egular, (L)ow, (O)ff
    //slotSendCommandToPtu("PHO"); //PanHold PowerMode: Off
    //slotSendCommandToPtu("THL"); //TiltHold PowerMode: Off

    // Set move power: (H)igh, (R)egular, (L)ow, (O)ff
    // WARNING: Don't use High, manual says its not good
    //slotSendCommandToPtu("PML"); //PanHold PowerMode: Off
    //slotSendCommandToPtu("TML"); //TiltHold PowerMode: Off

    // Set step modes
    slotSendCommandToPtu("WPF"); // pan axis full step
    slotSendCommandToPtu("WTF"); // tilt axis full step

    // Set speeds
    //slotSendCommandToPtu("PS800"); // pan speed, 800 seems to be max
    //slotSendCommandToPtu("TS1000"); // tilt speed, 1500 loses sync

    // Set accelerations
    //slotSendCommandToPtu("PA1000"); // pan acceleration, default
    //slotSendCommandToPtu("TA500"); // tilt acceleration

    // Get pan and tilt resolution - these values depend
    // on the step modes, so they need to be defined first.
    slotSendCommandToPtu("PR");
    slotSendCommandToPtu("TR");

    // Set tilt position limits (don't damage the PTU itself or the camera or the lens)
    slotSendCommandToPtu("");

    // Enable continuous pan mode
    slotSendCommandToPtu("PCE");

    // Go to immediate mode
    slotSendCommandToPtu("I");

    // Enable factory limits
    //slotSendCommandToPtu("LE");
}

void PtuController::slotSerialPortStatusChanged(const QString& status, const QDateTime& time)
{
     qDebug() << "PtuController::slotSerialPortStatusChanged(): usb port status" << status << "errorstring" << mSerialPortPtu->errorString();
}

void PtuController::slotVehiclePoseChanged(const Pose& pose)
{
    mLastKnownVehiclePose = pose;
    qDebug() << pose;

    if(ui->mPushButtonToggleControllerState->isChecked() && mSerialPortPtu->isOpen())
    {
        // make ptu point at pose
    }
}

void PtuController::slotSetPosition(float degreePan, float degreeTilt)
{
    int ptuPan = mPositionsPerDegreePan * degreePan;
    qDebug() << "sending: " << "PP"+QString::number(ptuPan);
    slotSendCommandToPtu("PP"+QString::number(ptuPan));

    int ptuTilt = mPositionsPerDegreeTilt * degreeTilt;
    qDebug() << "sending: " << "PP"+QString::number(ptuTilt);
    slotSendCommandToPtu("TP"+QString::number(ptuTilt));
}

void PtuController::slotSetPositionCamera()
{
    //slotSetPosition(90, -50);
    mPositionCameraSensor = mLastKnownVehiclePose.getPosition();
    ui->mLabelPositionCamera->setText(QString("%1/%2/%3").arg(mPositionCameraSensor.x()).arg(mPositionCameraSensor.y()).arg(mPositionCameraSensor.z()));
    if(!mPositionInFrustumCenter.isNull())
        determinePtuPose();
}

void PtuController::slotSetPositionFrustumCenter()
{
    //slotSetPosition(0, 0);
    mPositionInFrustumCenter = mLastKnownVehiclePose.getPosition();
    ui->mLabelPositionLens->setText(QString("%1/%2/%3").arg(mPositionInFrustumCenter.x()).arg(mPositionInFrustumCenter.y()).arg(mPositionInFrustumCenter.z()));
    if(!mPositionCameraSensor.isNull())
        determinePtuPose();
}

void PtuController::determinePtuPose()
{
    // Both mPositionCameraSensor and mPositionInFrustumCenter are known, so we have a ray defining the camera setup
}

void PtuController::slotDataReady()
{
    mDataFromPtu.append(mSerialPortPtu->readAll());
    QString qstring_byte = QString(mDataFromPtu);
    QStringList qstring_list = qstring_byte.split("\n");

    // Maybe parse in another method?

    // We know a command is done if the last element in the list is empty
    if(qstring_list.last() == "")
    {
        qDebug() << "parse";
        // Pan Resolution
        if(qstring_list.first().contains("PR"))
        {
            qstring_list.first().remove("PR * ");
            mPositionsPerDegreePan = 3600 / qstring_list.first().toDouble();
        }

        // Tilt Resolution
        if(qstring_list.first().contains("TR"))
        {
            qstring_list.first().remove("TR * ");
            mPositionsPerDegreeTilt = 3600 / qstring_list.first().toDouble();
        }

        mDataFromPtu.clear();
    }

    qDebug() << "PtuController::slotDataReady(): receive buffer contains: \n" << qstring_list;
}
