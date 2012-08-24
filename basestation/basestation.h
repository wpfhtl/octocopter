#ifndef BASESTATION_H
#define BASESTATION_H

#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QInputDialog>

#include "glwidget.h"
#include "audioplayer.h"
#include "flightplannercuda.h"
#include "logplayer.h"
#include "ptucontroller.h"
#include "octree.h"
#include "joystick.h"
#include "diffcorrfetcher.h"
#include "pidcontrollerwidget.h"
#include "lidarpoint.h"
#include "flightplannerbasic.h"
#include "flightplannerphysics.h"
#include "flightplannerparticles.h"
#include "flightplannerinterface.h"
#include "camerawidget.h"
#include "connectiondialog.h"
#include "controlwidget.h"
#include "logwidget.h"
#include "plymanager.h"
#include "roverconnection.h"
#include "wirelessdevice.h"
#include <plotwidget.h>
#include <waypoint.h>
#include <pose.h>

class FlightPlannerInterface;
class GlWidget;

class BaseStation : public QMainWindow
{
    Q_OBJECT

private:
    QMenu *mMenuWindowList, *mMenuFile, *mMenuView;
    QTimer* mTimerJoystick;
    WirelessDevice* mWirelessDevice;
    ConnectionDialog* mConnectionDialog;
    RoverConnection* mRoverConnection;
    FlightPlannerInterface* mFlightPlanner;
    Joystick* mJoystick;
    DiffCorrFetcher* mDiffCorrFetcher;
    ControlWidget* mControlWidget;
    LogWidget* mLogWidget;
    PidControllerWidget* mPidControllerWidget;
    PlotWidget* mPlotWidget;
    LogPlayer* mLogPlayer;
    PtuController* mPtuController;
    AudioPlayer* mAudioPlayer;
    QAction* mActionEnableAudio;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

    GlWidget *mGlWidget;

    void keyPressEvent(QKeyEvent* event);

    void processIncomingPoints();
    void processIncomingImages();

    QMap<QString, CameraWidget*> mCameraWidgets;

    QProgressDialog* mProgress;

signals:


private slots:
    void slotExportCloud(void);
    void slotImportCloud(void);
    void slotToggleLogWidget(void) {if(mLogWidget) mLogWidget->setVisible(!mLogWidget->isVisible());}
    void slotTogglePlotWidget(void) {if(mPlotWidget) mPlotWidget->setVisible(!mPlotWidget->isVisible());}
    void slotToggleControlWidget(void) {if(mControlWidget) mControlWidget->setVisible(!mControlWidget->isVisible());}
    void slotTogglePidControllerWidget() {if(mPidControllerWidget) mPidControllerWidget->setVisible(! mPidControllerWidget->isVisible());}
    void slotTogglePtuControllerWidget() {if(mPtuController) mPtuController->setVisible(! mPtuController->isVisible());}

    void slotSetFlightControllerValues(const FlightControllerValues& fcv);
    void slotSpeakGnssStatus(const GnssStatus &status);

    void slotClearOctree();

    // Send motion commands to rover WHILE button 1 is pressed
    void slotManageJoystick(quint8 button, bool pressed);

    // These are called by ConnectionRover when new data arrived
    void slotNewScanData(const QVector<QVector3D>& pointList, const QVector3D& scannerPosition);
    void slotNewImage(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);
    void slotNewVehicleStatus(const quint32& missionRunTime, const float& batteryVoltage, const qint16& barometricHeight, const qint8& wirelessRssi);

public:
    BaseStation(void);
    ~BaseStation();
};

#endif
