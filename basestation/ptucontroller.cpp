#include "ptucontroller.h"
#include "ui_ptucontroller.h"

PtuController::PtuController(const QString& deviceFile, QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::PtuController),
    mTimerUpdateStatus(0)
{
    ui->setupUi(this);

    // Connect UI...
    connect(ui->mPushButtonSetPositionCamera, SIGNAL(clicked()), SLOT(slotSetPositionCamera()));
    connect(ui->mPushButtonSetPositionLens, SIGNAL(clicked()), SLOT(slotSetPositionFrustumCenter()));
    connect(ui->mDirectInput, SIGNAL(returnPressed()), SLOT(slotSendDirectCommand()));

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

    if(mTimerUpdateStatus)
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

void PtuController::slotSendDirectCommand()
{
    QString command = ui->mDirectInput->text();

    // Parse command
    if(command.contains("/"))
    {
        // For sending fake vehicle poses
        QStringList coordinates = command.split("/");
        qDebug() << coordinates.size();
        if(coordinates.size() == 3)
        {
            Pose pose(QVector3D(coordinates[0].toDouble(),
                                coordinates[1].toDouble(),
                                coordinates[2].toDouble()),
                      0, 0, 0);
            mPositionCameraSensor = QVector3D(0, 1, 0);
            mPositionInFrustumCenter = QVector3D(5, 1, 0);
            determinePtuPose(mPositionInFrustumCenter, mPositionCameraSensor);
            ui->mPushButtonToggleControllerState->setEnabled(true);
            ui->mPushButtonToggleControllerState->setChecked(true);
            slotVehiclePoseChanged(pose);
            qDebug() << "PtuController::slotSendDirectCommand(): sending debug pose: " << pose;
        }
    }
    else
    {
        slotSendCommandToPtu(command);
    }
    ui->mDirectInput->clear();
}

void PtuController::slotInitialize()
{
    // Somehow first command gets eaten, so send an empty one first
    slotSendCommandToPtu(" ");

    // Factory settings reset
    slotSendCommandToPtu("DF");

    // Enable terse mode for better parsing
    slotSendCommandToPtu("FT");

    // Reset position
    slotSendCommandToPtu("R");

    // Wait before proceeding
    slotSendCommandToPtu("A");

    // Enable host command echoing EE, disable ED
    slotSendCommandToPtu("EE");

    // Get firmware version
    slotSendCommandToPtu("V");

    // Enable continuous pan mode
    //slotSendCommandToPtu("PCE");

    // Go to immediate mode
    slotSendCommandToPtu("I");

    // Set hold power: (R)egular, (L)ow, (O)ff
    //slotSendCommandToPtu("PHO"); //PanHold PowerMode: Off
    //slotSendCommandToPtu("THL"); //TiltHold PowerMode: Off

    // Set move power: (H)igh, (R)egular, (L)ow, (O)ff
    // WARNING: Don't use High, manual says its not good
    //slotSendCommandToPtu("PML"); //PanHold PowerMode: Off
    //slotSendCommandToPtu("TML"); //TiltHold PowerMode: Off

    // Set step modes
    //slotSendCommandToPtu("WPF"); // pan axis full step
    //slotSendCommandToPtu("WTF"); // tilt axis full step

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

    // Disable factory limits
    slotSendCommandToPtu("LD");

    // Get pan and tilt limits
    slotSendCommandToPtu("PN");
    slotSendCommandToPtu("PX");
    slotSendCommandToPtu("TN");
    slotSendCommandToPtu("TX");

    // Set user tilt position limits (don't damage the PTU itself or the camera or the lens)
    //slotSendCommandToPtu("");
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
        const QVector3D ptuToVehicle = pose.getPosition() - mPosePtuBase.getPosition();
        float yawAngle = RAD2DEG(atan2(ptuToVehicle.x(), ptuToVehicle.z()));
        yawAngle -= mPosePtuBase.getYawDegrees();
        qDebug() << "ptuToVehicle: " << yawAngle;

        // Both mPositionCameraSensor and mPositionInFrustumCenter are known, so we have a ray defining the camera setup

        float pitchAngle = RAD2DEG(asin(ptuToVehicle.y()/ptuToVehicle.length()));
        pitchAngle -= mPosePtuBase.getPitchDegrees();
        qDebug() << "ptuOrientation pitch: " << pitchAngle;
        //Q_ASSERT(abs(pitchAngle) < 90 && "Invalid pitch");
//
        //mPosePtuBase = Pose(mPositionCameraSensor, yawAngle, pitchAngle, 0);
       // qDebug() << "PtuController::determinePtuPose(): " << mPosePtuBase;

        //float pitchAngle = pitchPosLine1.angleTo(pitchPosLine2);
        //pitchAngle = Pose::getShortestTurnDegrees(pitchAngle);
        //qDebug() << "PtuController::slotVehiclePoseChanged(): yaw: " << yawAngle << " pitch: " << pitchAngle;
//
        slotSetPosition(yawAngle, pitchAngle);
    }
}

void PtuController::slotSetPosition(float degreePan, float degreeTilt)
{
    if(mSerialPortPtu->isOpen())
    {
        int ptuPan = mPositionsPerDegreePan * -degreePan;
        ptuPan = Pose::getShortestTurnDegrees(180 + ptuPan); // Camera mounted backwards
        //if(ptuPan < mMaxPanPositionsClockwise && ptuPan > mMaxPanPositionsCounterClockwise)
        //{
            qDebug() << "PtuController::slotSetPosition(): " << "sending PP"+QString::number(ptuPan);
            slotSendCommandToPtu("PP"+QString::number(ptuPan));
        //}
        //else
        //{
            // Handle behavior if this exceeds the specified limits. For pan, perhaps we need to wrap because
            // pan is actually continuous.
        //}

        int ptuTilt = mPositionsPerDegreeTilt * -degreeTilt;
        if(ptuTilt > mMaxTiltPositions)
        {
            qDebug() << "PtuController::slotSetPosition(): " << "sending TP"+QString::number(mMaxTiltPositions);
            slotSendCommandToPtu("TP"+QString::number(mMaxTiltPositions));
        }
        else if(ptuTilt < mMinTiltPositions) {
            qDebug() << "PtuController::slotSetPosition(): " << "sending TP"+QString::number(mMinTiltPositions);
            slotSendCommandToPtu("TP"+QString::number(mMinTiltPositions));
        }
        else
        {
            qDebug() << "PtuController::slotSetPosition(): " << "sending TP"+QString::number(ptuTilt);
            slotSendCommandToPtu("TP"+QString::number(ptuTilt));
        }
    }
}

// Perhaps use Axis enum?
void PtuController::slotSetPanLimits(float degreeMinimum, float degreeMaximum)
{
}

// Perhaps use Axis enum?
void PtuController::slotSetTiltLimits(float degreeMinimum, float degreeMaximum)
{
}

void PtuController::slotSetPositionCamera()
{
    //slotSetPosition(90, -50);
    mPositionCameraSensor = mLastKnownVehiclePose.getPosition();
    ui->mLabelPositionCamera->setText(QString("%1/%2/%3").arg(mPositionCameraSensor.x()).arg(mPositionCameraSensor.y()).arg(mPositionCameraSensor.z()));
    if(!mPositionInFrustumCenter.isNull())
    {
        determinePtuPose(mPositionInFrustumCenter, mPositionCameraSensor);
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
        determinePtuPose(mPositionInFrustumCenter, mPositionCameraSensor);
        ui->mPushButtonToggleControllerState->setEnabled(true);
    }
}

//float PtuController::getAngleBetween() {}

Pose PtuController::determinePtuPose(QVector3D positionInFrustumCenter, QVector3D positionCameraSensor)
{
    // Both positionCameraSensor and positionInFrustumCenter are known, so we have a ray defining the camera setup
    const QVector3D ptuOrientation = positionInFrustumCenter - positionCameraSensor;

    float yawAngle = RAD2DEG(atan2(ptuOrientation.x(), ptuOrientation.z()));
    //qDebug() << "ptuOrientation yaw: " << yawAngle;

    QVector2D basePlaneVector = QVector2D(ptuOrientation.x(), ptuOrientation.z());
    float pitchAngle = RAD2DEG(atan2(ptuOrientation.y(), basePlaneVector.length()));
    //qDebug() << "ptuOrientation pitch: " << pitchAngle;
    Q_ASSERT(abs(pitchAngle) < 90 && "Invalid pitch");

    Pose ptuPose(positionCameraSensor, yawAngle, pitchAngle, 0);
    //qDebug() << "PtuController::determinePtuPose(): " << ptuPose;
    return ptuPose;
}

void PtuController::slotDataReady()
{
    mDataFromPtu.append(mSerialPortPtu->readAll());
    QString dataFromPtu = QString(mDataFromPtu) ;
    QStringList splitDataFromPtu = dataFromPtu.split("\n");

    // Maybe parse in another method?

    // We know a command is done if the last element in the list is empty
    if(splitDataFromPtu.last() == "")
    {
        //qDebug() << "PtuController::slotDataReady(): begin parsing";

        // Errors
        if(splitDataFromPtu.first().contains("!"))
        {
            qDebug() << "PtuController::slotDataReady(): got error: " << splitDataFromPtu.first();
            splitDataFromPtu.removeFirst();
        }

        // Pan resolution
        if(splitDataFromPtu.first().contains("PR"))
        {
            splitDataFromPtu.first().remove("PR * ");
            mPositionsPerDegreePan = 3600 / splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): pan resolution " << mPositionsPerDegreePan << " positions per degree";
        }

        // Tilt resolution
        if(splitDataFromPtu.first().contains("TR"))
        {
            splitDataFromPtu.first().remove("TR * ");
            mPositionsPerDegreeTilt = 3600 / splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): tilt resolution " << mPositionsPerDegreeTilt << " positions per degree";
        }

        // Minimum pan limit
        if(splitDataFromPtu.first().contains("PN"))
        {
            splitDataFromPtu.first().remove("PN * ");
            mMinPanPositions = splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): minimum pan: " << mMinPanPositions;
        }

        // Maximum pan limit
        if(splitDataFromPtu.first().contains("PX"))
        {
            splitDataFromPtu.first().remove("PX * ");
            mMaxPanPositions = splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): maximum pan: " << mMaxPanPositions;
        }

        // Minimum tilt limit
        if(splitDataFromPtu.first().contains("TN"))
        {
            splitDataFromPtu.first().remove("TN * ");
            mMinTiltPositions = splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): minimum tilt: " << mMinTiltPositions;
        }

        // Maximum tilt limit
        if(splitDataFromPtu.first().contains("TX"))
        {
            splitDataFromPtu.first().remove("TX * ");
            mMaxTiltPositions = splitDataFromPtu.first().toDouble();
            qDebug() << "PtuController::slotDataReady(): maximum tilt: " << mMaxTiltPositions;
        }

        mDataFromPtu.clear();
    }

    //qDebug() << "PtuController::slotDataReady(): receive buffer contains: \n" << splitDataFromPtu;
}
