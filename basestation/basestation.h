#ifndef BASESTATION_H
#define BASESTATION_H

#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QInputDialog>
#include <QMainWindow>
#include <QWidgetAction>

#include "messagehandler.h"
#include "glwindow.h"
#include "audioplayer.h"
#include "logplayer.h"
#include "ptucontroller.h"
#include "pointcloudoctree.h"
#include "joystick.h"
#include "diffcorrfetcher.h"
#include "pidcontrollerwidget.h"
#include "lidarpoint.h"
#include "flightplannerparticles.h"
#include "camerawidget.h"
#include "connectiondialog.h"
#include "controlwidget.h"
#include "satellitewidget.h"
#include "logwidget.h"
#include "plymanager.h"
#include "roverconnection.h"
#include "wirelessdevice.h"
//#include <plotwidget.h>
#include <waypoint.h>
#include <pose.h>
#include "pointcloudcuda.h"

class FlightPlannerInterface;
class GlWindow;

class MenuSlider : public QWidgetAction
{
    Q_OBJECT

    float mMin, mMax, mStartValue;
    float mTickInterval;
    QString mLabelString;
    QHBoxLayout* mLayout;
    QWidget* mWidget;
    QLabel* mLabel;
    QSlider* mSlider;

public:
    MenuSlider(const QString& label, const float min, const float startValue, const float max, QWidget* parent, const float tickInterval = 0) : QWidgetAction(parent)
    {
        mWidget = 0;
        mMin = min;
        mMax = max;
        mStartValue = qBound(mMin, startValue, mMax);
        mLabelString = label;
        mTickInterval = tickInterval;
    }

protected:
    virtual QWidget* createWidget(QWidget *parent)
    {
        if(mWidget == 0)
        {
            mWidget = new QWidget(parent);
            //mWidget->setMinimumWidth(250);

            mLabel = new QLabel(mLabelString, parent);
            mLabel->setMinimumWidth(100);
            mSlider = new QSlider(Qt::Horizontal, parent);
            if(fabs(mTickInterval) > 0.00001f)
            {
                qDebug() << "tickinterval" << mTickInterval << mTickInterval * 1000;
                mSlider->setTickInterval(mTickInterval * 1000);
            }
            mSlider->setMinimumWidth(190);
            mSlider->setRange(mMin * 1000.0f, mMax * 1000.0f);
            mSlider->setValue(mStartValue * 1000.0f);

            mLayout = new QHBoxLayout(mWidget);
            mLayout->setMargin(0);
            mLayout->setContentsMargins(7, 4, 2, 4);
            mLayout->addWidget(mLabel);
            mLayout->addWidget(mSlider);
            mWidget->setLayout(mLayout);
            //setDefaultWidget(mWidget);

            connect(mSlider, SIGNAL(sliderMoved(int)), SLOT(slotValueChanged(int)));
            connect(mSlider, SIGNAL(valueChanged(int)), SLOT(slotValueChanged(int)));
        }

        return mWidget;
    }

    virtual void deleteWidget(QWidget *widget)
    {
        if(widget == mWidget)
        {
            mWidget = 0;
            QWidgetAction::deleteWidget(widget);
        }
    }

signals:
    void value(float);

private slots:
    void slotValueChanged(int v)
    {
        emit value(v/1000.0f);
    }
};

class BaseStation : public QMainWindow
{
    Q_OBJECT

public:
    BaseStation(void);
    ~BaseStation();

    OperatingMode getOperatingMode() {return mOperatingMode;}
    GlScene* getGlScene() {return mGlScene;}

private:
    MessageHandler* mMessageHandler;
    QMenu *mMenuWindowList, *mMenuFile, *mMenuView;
    QTimer* mTimerJoystick;
    WirelessDevice* mWirelessDevice;
    ConnectionDialog* mConnectionDialog;
    RoverConnection* mRoverConnection;
    FlightPlannerParticles* mFlightPlanner;
    Joystick* mJoystick;
    DiffCorrFetcher* mDiffCorrFetcher;
    ControlWidget* mControlWidget;
    LogWidget* mLogWidget;
    SatelliteWidget* mSatelliteWidget;
    PidControllerWidget* mPidControllerWidget;
//    PlotWidget* mPlotWidget;
    LogPlayer* mLogPlayer;
    PtuController* mPtuController;
    AudioPlayer* mAudioPlayer;
    QAction* mActionEnableAudio;

    // a container for collected rays, or rather the world coordinates of where they ended
    PointCloudCuda* mPointCloud;

    GlWindow *mGlWindow;
    GlScene *mGlScene;

    QMap<QString, CameraWidget*> mCameraWidgets;

    void closeEvent(QCloseEvent *event);

    OperatingMode mOperatingMode;

signals:


private slots:


    void slotExportCloud(void);
    void slotImportCloud(void);
    void slotToggleLogWidget(void) {if(mLogWidget) mLogWidget->setVisible(!mLogWidget->isVisible());}
    void slotToggleLogPlayer(void) {if(mLogPlayer) mLogPlayer->setVisible(!mLogPlayer->isVisible());}
    void slotToggleSatelliteWidget(void) {if(mSatelliteWidget) mSatelliteWidget->setVisible(!mSatelliteWidget->isVisible());}
    void slotToggleControlWidget(void) {if(mControlWidget) mControlWidget->setVisible(!mControlWidget->isVisible());}
    void slotTogglePidControllerWidget() {if(mPidControllerWidget) mPidControllerWidget->setVisible(! mPidControllerWidget->isVisible());}
    void slotTogglePtuControllerWidget() {if(mPtuController) mPtuController->setVisible(! mPtuController->isVisible());}
//    void slotToggleViewPointCloudDense() {if(mGlWindow->isPointCloudRegistered(mPointCloud)) mGlWindow->slotPointCloudUnregister(mPointCloud); else mGlWindow->slotPointCloudRegister(mPointCloud);}

    void slotSetFlightControllerValues(const FlightControllerValues *const fcv);
    void slotSpeakGnssStatus(const GnssStatus *const status);

//    void slotClearCloud();

    // Send motion commands to rover WHILE button 1 is pressed
    void slotManageJoystick(quint8 button, bool pressed);

    // These are called by ConnectionRover when new data arrived
//    void slotNewScanData(const QVector<QVector3D> *const pointList, const QVector3D *const scannerPosition);
    void slotNewImage(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);

};

#endif
