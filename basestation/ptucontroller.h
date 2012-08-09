#ifndef PTUCONTROLLER_H
#define PTUCONTROLLER_H

#include <QDockWidget>
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QString>
#include <QByteArray>

#include <abstractserial.h>
#include "pose.h"
#include "model.h"
#include "shaderprogram.h"

namespace Ui {
    class PtuController;
}

class PtuController : public QDockWidget
{
    Q_OBJECT

public:

    enum Axis
    {
        AXIS_PAN,
        AXIS_TILT
    };

    PtuController(const QString& deviceFile, QWidget *parent = 0);
    ~PtuController();

    const Pose* getPosePtuBase() const {return &mPosePtuBase;}
    bool isOpened() const {return mSerialPortPtu->isOpen();}

    static Pose determinePtuPose(QVector3D positionCameraSensor, QVector3D positionInFrustrumCenter);

private:
    Ui::PtuController *ui;
    QTimer* mTimerUpdateStatus;
    AbstractSerial *mSerialPortPtu;

    // This first pose as calculated by the very first determinePtuPose when the enable-button is clicked.
    // Further updates on vehiclePoseChanged will only change mTiltToVehicle and mPanToVehicle.
    Pose mPosePtuBase;

    double mPanToVehicle;
    double mTiltToVehicle;

    Pose mLastKnownVehiclePose;
    QVector3D mPositionCameraSensor;
    QVector3D mPositionInFrustumCenter;
    QByteArray mDataFromPtu;
    double mPositionsPerDegreePan;
    double mPositionsPerDegreeTilt;
    double mMinPanPositions;
    double mMaxPanPositions;
    double mMinTiltPositions;
    double mMaxTiltPositions;
    Model* mModelPtuBase;
    Model* mModelPtuPan;
    Model* mModelPtuTilt;

private slots:
    void slotSerialPortStatusChanged(const QString& status, const QDateTime& time);
    void slotSendDirectCommand();
    void slotInitialize();
    void slotDataReady();
    void slotSetPositionCamera();
    void slotSetPositionFrustumCenter();
    void slotRetrieveStatus();
    void slotSetPosition(float degreePan, float degreeTilt);
    void slotSetPanLimits(float degreeMinimum, float degreeMaximum);
    void slotSetTiltLimits(float degreeMinimum, float degreeMaximum);
    void slotSendCommandToPtu(const QString& command);
    void slotCalculateBasePose();

public slots:
    void slotVehiclePoseChanged(const Pose& pose);
    void slotSetSpeed(Axis axis, quint16 speed);
    void slotVisualize();

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);
};

#endif
