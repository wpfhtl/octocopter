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
    }
    else
    {
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

    if(mSerialPortPtu->isOpen())
        mSerialPortPtu->close();

    mSerialPortPtu->deleteLater();

    mTimerUpdateStatus->deleteLater();

    delete ui;
}

void PtuController::slotSetSpeed(Axis axis, quint16 speed)
{
    if(mSerialPortPtu->isOpen())
    {
        const QString command = axis == AXIS_PAN ? "PS" : "TS";
        slotSendCommandToPtu(command + QString::number(speed));
    }
}

void PtuController::slotRetrieveStatus()
{
    // TODO: Parse status
    //slotSendCommandToPtu("O");
}

void PtuController::slotSendCommandToPtu(const QString& command)
{
    mSerialPortPtu->write(QString(command + " ").toAscii());
}

void PtuController::slotInitialize()
{
    // Factory settings reset
    slotSendCommandToPtu("DF");

    // Reset position
    slotSendCommandToPtu("R");

    // Wait before proceeding
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
     qDebug() << "PtuController::slotSerialPortStatusChanged(): usb port status" << status;// << "errorstring" << mSerialPortPtu->errorString();
}

void PtuController::slotVehiclePoseChanged(const Pose& pose)
{
    mLastKnownVehiclePose = pose;
    //qDebug() << pose;

    if(ui->mPushButtonToggleControllerState->isChecked())
    {
        // make ptu point at pose
        QLineF yawPosLine1(mPosePtuBase.getPosition().x(), mPosePtuBase.getPosition().z(),
                           mPosePtuBase.getPosition().x(), mLastKnownVehiclePose.getPosition().z());
        QLineF yawPosLine2(mPosePtuBase.getPosition().x(), mPosePtuBase.getPosition().z(),
                           mLastKnownVehiclePose.getPosition().x(), mLastKnownVehiclePose.getPosition().z());
        float yawAngle = yawPosLine1.angleTo(yawPosLine2);
        qDebug() << yawAngle;

        QLineF pitchPosLine1(mPosePtuBase.getPosition().z(), mPosePtuBase.getPosition().y(),
                             mLastKnownVehiclePose.getPosition().z(), mPosePtuBase.getPosition().y());
        QLineF pitchPosLine2(mPosePtuBase.getPosition().z(), mPosePtuBase.getPosition().y(),
                             mLastKnownVehiclePose.getPosition().x(), mLastKnownVehiclePose.getPosition().y());
        float pitchAngle = pitchPosLine1.angleTo(pitchPosLine2);
        qDebug() << pitchAngle;

        slotSetPosition(yawAngle, pitchAngle);
    }
}

void PtuController::slotSetPosition(float degreePan, float degreeTilt)
{
    if(mSerialPortPtu->isOpen())
    {
        int ptuPan = mPositionsPerDegreePan * degreePan;
        //if(ptuPan < mMaxPanPositionsClockwise && ptuPan > mMaxPanPositionsCounterClockwise)
        //{
            qDebug() << "sending: " << "PP"+QString::number(ptuPan);
            slotSendCommandToPtu("PP"+QString::number(ptuPan));
        //}
        //else
        //{
            // Handle behavior if this exceeds the specified limits. For pan, perhaps we need to wrap because
            // pan is actually continuous.
        //}

        int ptuTilt = mPositionsPerDegreeTilt * degreeTilt;
        //if(ptuPan < mMaxPanPositionsClockwise && ptuPan > mMaxPanPositionsCounterClockwise)
        //{
            qDebug() << "sending: " << "TP"+QString::number(ptuTilt);
            slotSendCommandToPtu("TP"+QString::number(ptuTilt));
        //}
        //else
        //{
            // Handle behavior if this exceeds the specified limits. Simply cut send the max/min for tilt?
        //}
    }
}

// Perhaps use Axis enum?
void PtuController::slotSetMaxPan(float degreeMaxClockwise, float degreeMaxCounterClockwise)
{
}

// Perhaps use Axis enum?
void PtuController::slotSetMaxTilt(float degreeMaxUpwards, float degreeMaxDownwards)
{
}

void PtuController::slotSetPositionCamera()
{
    //slotSetPosition(90, -50);
    mPositionCameraSensor = mLastKnownVehiclePose.getPosition();
    ui->mLabelPositionCamera->setText(QString("%1/%2/%3").arg(mPositionCameraSensor.x()).arg(mPositionCameraSensor.y()).arg(mPositionCameraSensor.z()));
    if(!mPositionInFrustumCenter.isNull())
    {
        determinePtuPose();
        ui->mPushButtonToggleControllerState->setEnabled(true);
    }
}

void PtuController::slotSetPositionFrustumCenter()
{
    //slotSetPosition(0, 0);
    mPositionInFrustumCenter = mLastKnownVehiclePose.getPosition();
    ui->mLabelPositionLens->setText(QString("%1/%2/%3").arg(mPositionInFrustumCenter.x()).arg(mPositionInFrustumCenter.y()).arg(mPositionInFrustumCenter.z()));
    if(!mPositionCameraSensor.isNull())
    {
        determinePtuPose();
        ui->mPushButtonToggleControllerState->setEnabled(true);
    }
}

void PtuController::determinePtuPose()
{
    // Both mPositionCameraSensor and mPositionInFrustumCenter are known, so we have a ray defining the camera setup
    QLineF yawPosLine1(mPositionCameraSensor.x(), mPositionCameraSensor.z(),
                       mPositionCameraSensor.x(), mPositionInFrustumCenter.z());
    QLineF yawPosLine2(mPositionCameraSensor.x(), mPositionCameraSensor.z(),
                       mPositionInFrustumCenter.x(), mPositionInFrustumCenter.z());
    float yawAngle = yawPosLine1.angleTo(yawPosLine2);
    qDebug() << yawAngle;

    QLineF pitchPosLine1(mPositionCameraSensor.z(), mPositionCameraSensor.y(),
                         mPositionInFrustumCenter.z(), mPositionCameraSensor.y());
    QLineF pitchPosLine2(mPositionCameraSensor.z(), mPositionCameraSensor.y(),
                         mPositionInFrustumCenter.x(), mPositionInFrustumCenter.y());
    float pitchAngle = pitchPosLine1.angleTo(pitchPosLine2);
    qDebug() << pitchAngle;

    mPosePtuBase = Pose(mPositionCameraSensor, yawAngle, pitchAngle, 0);
}

void PtuController::slotDataReady()
{
    mDataFromPtu.append(mSerialPortPtu->readAll());
    QString dataFromPtu = QString(mDataFromPtu);
    QStringList splitDataFromPtu = dataFromPtu.split("\n");

    // Maybe parse in another method?

    // We know a command is done if the last element in the list is empty
    if(splitDataFromPtu.last() == "")
    {
        qDebug() << "PtuController::slotDataReady(): begin parsing";

        // Pan resolution
        if(splitDataFromPtu.first().contains("PR"))
        {
            splitDataFromPtu.first().remove("PR * ");
            mPositionsPerDegreePan = 3600 / splitDataFromPtu.first().toDouble();
        }

        // Tilt resolution
        if(splitDataFromPtu.first().contains("TR"))
        {
            splitDataFromPtu.first().remove("TR * ");
            mPositionsPerDegreeTilt = 3600 / splitDataFromPtu.first().toDouble();
        }

        // Max pan in degrees (handle for both directions)
        //if(splitDataFromPtu.first().contains("MP"))
        //{
        //    splitDataFromPtu.first().remove("MP * ");
              // Pan is actually continuous but let's just get this data while we're at it
        //    mMaxPanPositions = splitDataFromPtu.first().toDouble();
        //}

        // Max tilt in degrees (handle for both directions)
        //if(splitDataFromPtu.first().contains("MT"))
        //{
        //    splitDataFromPtu.first().remove("MT * ");
              // This should also be manually limitable
              // Also the limits per direction are different
        //    mMaxTiltPositions = splitDataFromPtu.first().toDouble();
        //}

        mDataFromPtu.clear();
    }

    qDebug() << "PtuController::slotDataReady(): receive buffer contains: \n" << splitDataFromPtu;
}
