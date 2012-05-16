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

private:
    Ui::PtuController *ui;
    QTimer* mTimerUpdateStatus;
    AbstractSerial *mSerialPortPtu;
    Pose mPosePtuBase;
    Pose mLastKnownVehiclePose;
    QVector3D mPositionCameraSensor;
    QVector3D mPositionInFrustumCenter;
    QByteArray mDataFromPtu;
    double mPositionsPerDegreePan;
    double mPositionsPerDegreeTilt;
    double mMaxPanPositionsClockwise;
    double mMaxPanPositionsCounterClockwise;
    double mMaxTiltPositionsUpwards;
    double mMaxTiltPositionsDownwards;

    void determinePtuPose();

private slots:
    void slotSerialPortStatusChanged(const QString& status, const QDateTime& time);
    void slotInitialize();
    void slotDataReady();
    void slotSetPositionCamera();
    void slotSetPositionFrustumCenter();
    void slotRetrieveStatus();
    void slotSetPosition(float degreePan, float degreeTilt);
    void slotSetMaxPan(float degreeMaxClockwise, float degreeMaxCounterClockwise);
    void slotSetMaxTilt(float degreeMaxUpwards, float degreeMaxDownwards);
    void slotSendCommandToPtu(const QString& command);

public slots:
    void slotVehiclePoseChanged(const Pose& pose);
    void slotSetSpeed(Axis axis, quint16 speed);

signals:
    void message(const LogImportance& importance, const QString& source, const QString& message);
};

#endif
