#ifndef BASESTATION_H
#define BASESTATION_H

#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QInputDialog>

#include "flightplannercuda.h"
#include "glwidget.h"
#include "logplayer.h"
#include "octree.h"
#include "rtkfetcher.h"
#include "lidarpoint.h"
#include "flightplannerbasic.h"
#include "flightplannerphysics.h"
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

class ControlWidget;
class FlightPlannerInterface;
class GlWidget;

class BaseStation : public QMainWindow
{
    Q_OBJECT

private:
    //Pose mVehiclePose;

    WirelessDevice* mWirelessDevice;

    ConnectionDialog* mConnectionDialog;

    RoverConnection* mRoverConnection;

    FlightPlannerInterface* mFlightPlanner;

    RtkFetcher* mRtkFetcher;
    ControlWidget* mControlWidget;
    LogWidget* mLogWidget;
    PlotWidget* mPlotWidget;
    LogPlayer* mLogPlayer;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

    GlWidget *mGlWidget;

    QTimer *mTimerStats;
    QFile *mStatsFile;
    QDateTime mDateTimeLastLidarInput;

    void keyPressEvent(QKeyEvent* event);

    void processIncomingPoints();
    void processIncomingImages();

    QMap<QString, CameraWidget*> mCameraWidgets;

    QProgressDialog* mProgress;

signals:


private slots:
    void slotExportCloud(void);
    void slotImportCloud(void);
    void slotTogglePlot(void);

    void slotWriteStats();
    void slotAddLogFileMarkForPaper(QList<WayPoint> wptList);

    // These are called by ConnectionRover when new data arrived
    void slotNewScanData(const QVector<QVector3D>& pointList, const QVector3D& scannerPosition);
    void slotNewImage(const QString& cameraName, const QSize& imageSize, const Pose& cameraPose, const QByteArray& imageData);
    void slotNewVehicleStatus(const quint32& missionRunTime, const float& batteryVoltage, const qint16& barometricHeight, const qint8& wirelessRssi);

public:
    BaseStation(void);
    ~BaseStation();
};

#endif
