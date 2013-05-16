#include "ptucontroller.h"
#include "ui_ptucontroller.h"

#include <QRegExp>

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
    connect(ui->mPushButtonToggleControllerState, SIGNAL(pressed()), SLOT(slotCalculateBasePose()));

    mSerialPortPtu = new QSerialPort(deviceFile, this);
    connect(mSerialPortPtu, SIGNAL(error(QSerialPort::SerialPortError)), SLOT(slotSerialPortError(QSerialPort::SerialPortError)));
    if(!mSerialPortPtu->open(QIODevice::ReadWrite))
    {
        mSerialPortPtu->close();
        qDebug("PtuController::PtuController(): Opening serial usb port %s failed.", qPrintable(deviceFile));
    }
    else
    {
        mSerialPortPtu->setBaudRate(9600); // PTU can go faster, but do we need to?
        mSerialPortPtu->setDataBits(QSerialPort::Data8);
        mSerialPortPtu->setParity(QSerialPort::NoParity);
        mSerialPortPtu->setStopBits(QSerialPort::OneStop);
        mSerialPortPtu->setFlowControl(QSerialPort::NoFlowControl);
        connect(mSerialPortPtu, SIGNAL(readyRead()), SLOT(slotDataReady()));

        mTimerUpdateStatus = new QTimer(this);
        mTimerUpdateStatus->start(5000);
        connect(mTimerUpdateStatus, SIGNAL(timeout()), SLOT(slotRetrieveStatus()));

        mPositionsPerDegreePan = 0.0f;
        mPositionsPerDegreeTilt = 0.0f;

        QTimer::singleShot(1000, this, SLOT(slotInitialize()));
    }

    mPanToVehicle = 0;
    mTiltToVehicle = 0;


    mModelPtuBase = 0;
    mModelPtuPan = 0;
    mModelPtuTilt = 0;
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
    mSerialPortPtu->write(QString(command + " ").toLatin1());
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
//            slotVehiclePoseChanged(pose);
            qDebug() << "PtuController::slotSendDirectCommand(): sending debug pose: " << pose;
        }
    }
    else if(command.contains(QRegExp("^-?[0-9]+p?$"))) {
        // For sending pan in degrees

        command.remove("p");
        float degrees = command.toFloat();
        slotSetPanDegrees(degrees);
    }
    else if(command.contains(QRegExp("^-?[0-9]+t?$"))) {
        // For sending tilt in degrees

        command.remove("t");
        float degrees = command.toFloat();
        slotSetTiltDegrees(degrees);
    }
    else
    {
        // Send PTU command directly (see PTU command reference)

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

    // Wait before proceeding
    slotSendCommandToPtu("A");

    // Enable host command echoing EE, disable ED
    slotSendCommandToPtu("EE");

    // Get firmware version
    slotSendCommandToPtu("V");

    // Enable continuous pan mode
    slotSendCommandToPtu("PCE");

    // Reset position
    slotSendCommandToPtu("R");

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

    mElapsedTimer.start();
}

void PtuController::slotSerialPortError(const QSerialPort::SerialPortError& error)
{
     qDebug() << "PtuController::slotSerialPortError(): usb port error" << error;
}

void PtuController::slotVehiclePoseChanged(const Pose* const pose)
{
    mLastKnownVehiclePose = pose;

    if(ui->mPushButtonToggleControllerState->isChecked() && mElapsedTimer.elapsed() > 200)
    {
        mElapsedTimer.restart();

        Q_ASSERT(!mPositionInFrustumCenter.isNull() && !mPositionCameraSensor.isNull());

        // Change orientation of mPosePtuBase to look at vehicle
        float pan, tilt;
        getPanTilt(Pose::extrapolateLinear(mPreviousKnownVehiclePose, *mLastKnownVehiclePose, mLastKnownVehiclePose->timestamp + 600).getPosition(), mPosePtuBase, pan, tilt);
        //Pose tempPose = determinePtuPose(mLastKnownVehiclePose.getPosition(), mPosePtuBase.getPosition());

        mPreviousKnownVehiclePose = *mLastKnownVehiclePose;

        //mPanToVehicle = tempPose.getYawDegrees();
        mPanToVehicle = pan;
        //mTiltToVehicle = tempPose.getPitchDegrees();
        mTiltToVehicle = tilt;
        //qDebug() << "PoseBase " << mPosePtuBase;
        //qDebug() << "pan :" << mPanToVehicle << " tilt: " << mTiltToVehicle;
        slotSetPanDegrees(mPanToVehicle);
        slotSetTiltDegrees(mTiltToVehicle);
    }
}

void PtuController::slotSetPanDegrees(float degreePan)
{
    if(mSerialPortPtu->isOpen())
    {
        float degrees = Pose::getShortestTurnDegrees(degreePan) * mPositionsPerDegreePan * -1; // -180 due to back-mounted camera
        slotSendCommandToPtu("PP"+QString::number(degrees));
    }
}

void PtuController::slotSetTiltDegrees(float degreeTilt)
{
    if(mSerialPortPtu->isOpen())
    {
        float degrees = degreeTilt * mPositionsPerDegreeTilt * -1; // * -1 due to back-mounted camera
        slotSendCommandToPtu("TP"+QString::number(degrees));
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
    // mLastKnown is empty?!
    mPositionCameraSensor = mLastKnownVehiclePose->getPosition();
    ui->mLabelPositionCamera->setText(QString("%1/%2/%3").arg(mPositionCameraSensor.x()).arg(mPositionCameraSensor.y()).arg(mPositionCameraSensor.z()));
    if(!mPositionInFrustumCenter.isNull())
    {
        ui->mPushButtonToggleControllerState->setEnabled(true);
    }
}

void PtuController::slotSetPositionFrustumCenter()
{
    mPositionInFrustumCenter = mLastKnownVehiclePose->getPosition();
    ui->mLabelPositionLens->setText(QString("%1/%2/%3").arg(mPositionInFrustumCenter.x()).arg(mPositionInFrustumCenter.y()).arg(mPositionInFrustumCenter.z()));
    if(!mPositionCameraSensor.isNull())
    {
        ui->mPushButtonToggleControllerState->setEnabled(true);
    }
}

Pose PtuController::determinePtuPose(QVector3D positionInFrustumCenter, QVector3D positionCameraSensor)
{
    // Both positionCameraSensor and positionInFrustumCenter are known, so we have a ray defining the camera setup
    const QVector3D ptuOrientation = positionCameraSensor - positionInFrustumCenter;

    //qDebug() << "positionInFrustumCenter" << positionInFrustumCenter << "positionCameraSensor" << positionCameraSensor;

    float yawAngle = RAD2DEG(atan2(ptuOrientation.x(), ptuOrientation.z()));

    QVector2D basePlaneVector = QVector2D(ptuOrientation.x(), ptuOrientation.z());
    float pitchAngle = -RAD2DEG(atan2(ptuOrientation.y(), basePlaneVector.length()));

    Pose ptuPose(positionCameraSensor, yawAngle, pitchAngle, 0);
    return ptuPose;
}

void PtuController::getPanTilt(QVector3D vehiclePosition, Pose ptuBase, float& pan, float& tilt)
{
    QVector3D ptuBaseToVehicle = vehiclePosition - ptuBase.getPosition();
    float howToTurnFromNorth = RAD2DEG(atan2(-ptuBaseToVehicle.x(), -ptuBaseToVehicle.z()));
    float whereToPan = howToTurnFromNorth - ptuBase.getYawDegrees();

    pan = Pose::getShortestTurnDegrees(whereToPan);
    const QVector3D ptuOrientation = vehiclePosition - ptuBase.getPosition();
    QVector2D basePlaneVector = QVector2D(ptuOrientation.x(), ptuOrientation.z());
    tilt = RAD2DEG(atan2(ptuOrientation.y(), basePlaneVector.length())) - ptuBase.getPitchDegrees();

    //qDebug() << ptuBase;
    //qDebug() << "pan:" << pan << "tilt:" << tilt;
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

void PtuController::slotVisualize()
{
    if(!isVisible()) return;

    if(mModelPtuBase == 0 && mModelPtuPan == 0 && mModelPtuTilt == 0)
    {
        QDir modelPath = QDir::current();
        modelPath.cdUp();

        mModelPtuBase = new Model(QFile(modelPath.absolutePath() + "/media/ptu-base.obj"), QString("../media/"), this);
        mModelPtuPan = new Model(QFile(modelPath.absolutePath() + "/media/ptu-pan.obj"), QString("../media/"), this);
        mModelPtuTilt = new Model(QFile(modelPath.absolutePath() + "/media/ptu-tilt.obj"), QString("../media/"), this);
    }

    const QMatrix4x4& transform = mPosePtuBase.getMatrixRef();

    // Set initial starting position
    mModelPtuBase->slotSetModelTransform(transform);
    mModelPtuPan->slotSetModelTransform(transform);
    mModelPtuTilt->slotSetModelTransform(transform);

    // Somehow the model is tilted the wrong way so let's reverse that
    //QMatrix4x4 trBase = transform;
    //trBase.rotate(-mPosePtuBase.getPitchDegrees() * 2, QVector3D(1, 0, 0));
    //mModelPtuBase->slotSetModelTransform(trBase);

    // For pan we need to only apply the yaw
    QMatrix4x4 trPan = transform;
    trPan.rotate(mPanToVehicle, QVector3D(0,1,0));
    mModelPtuPan->slotSetModelTransform(trPan);
    //qDebug() << "pan transform: " << trPan;

    // For tilt we need to first apply yaw and then pitch
    QMatrix4x4 trTilt = transform;
    trTilt.translate(0.0f, -0.092f, 0.0f);
    trTilt.rotate(mPanToVehicle, QVector3D(0,1,0));
    trTilt.rotate(mTiltToVehicle, QVector3D(1,0,0));
    trTilt.translate(0.0f, +0.092f, 0.0f);
    mModelPtuTilt->slotSetModelTransform(trTilt);
    //qDebug() << "tilt transform: " << trTilt;

    mModelPtuBase->render();
    mModelPtuPan->render();
    mModelPtuTilt->render();
}

void PtuController::slotCalculateBasePose()
{
    mPosePtuBase = determinePtuPose(mPositionInFrustumCenter, mPositionCameraSensor);
}
