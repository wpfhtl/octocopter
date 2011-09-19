#ifndef BASESTATION_H
#define BASESTATION_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>
#include <QInputDialog>
#include <QTcpSocket>
#include <QMutex>

#include "glwidget.h"
#include "octree.h"
#include "rtkfetcher.h"
#include "lidarpoint.h"
#include "flightplannerinterface.h"
#include "camerawindow.h"
#include "connectiondialog.h"
#include "controlwidget.h"
#include "logwidget.h"
#include "cloudexporter.h"
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
    QTcpSocket* mTcpSocket;
    //Pose mVehiclePose;

    WirelessDevice* mWirelessDevice;

    ConnectionDialog* mConnectionDialog;

    QStringList mHostNames;

    QByteArray mIncomingDataBuffer;

    FlightPlannerInterface* mFlightPlanner;

    RtkFetcher* mRtkFetcher;
    ControlWidget* mControlWidget;
    LogWidget* mLogWidget;
    PlotWidget* mPlotWidget;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

    GlWidget *mGlWidget;

    QTimer *mTimerStats;
    QFile *mStatsFile;
    QDateTime mDateTimeLastLidarInput;

    void addRandomPoint();
    void keyPressEvent(QKeyEvent* event);

    void processIncomingPoints();
    void processIncomingImages();

    QMap<QString, CameraWindow*> mCameraWindows;

    QProgressDialog* mProgress;

    void processPacket(QByteArray data);

signals:
    void vehiclePoseChanged(Pose);

private slots:
    void slotAskForConnectionHostNames();
    void slotSocketConnected(void);
    void slotSocketDisconnected(void);
    void slotReadSocket(void);
    void slotSocketError(QAbstractSocket::SocketError socketError);
    void slotConnectToRover(void);
    void slotExportCloud(void);

    void slotRoverWayPointInsert(const quint16&, const WayPoint&);
    void slotRoverWayPointDelete(const quint16&);
    void slotRoverWayPointsSet(const QList<WayPoint>&);

    void slotSendRtkDataToRover(const QByteArray& rtkData);
    void slotSendData(const QByteArray &data);

    void slotWriteStats();
    void slotAddLogFileMarkForPaper(QList<WayPoint> wptList);

public:
    BaseStation(void);
    ~BaseStation();

//    const WayPoint getNextWayPoint(void) const;
};

#endif
