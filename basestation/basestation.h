#ifndef BASESTATION_H
#define BASESTATION_H

#include <QtCore>
#include <QtGui>
#include <QtNetwork>
#include <QDebug>

#include <QUdpSocket>
#include <QMutex>

#include "glwidget.h"
#include "octree.h"
#include "lidarpoint.h"
#include "flightplannerinterface.h"
#include "camerawindow.h"
#include "controlwidget.h"
#include "logwidget.h"
#include "cloudexporter.h"
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
//    QVector3D mVehiclePosition;
//    QQuaternion mVehicleOrientation;
    Pose mVehiclePose;

    QByteArray mIncomingDataBuffer;

    FlightPlannerInterface* mFlightPlanner;

    ControlWidget* mControlWidget;
    LogWidget* mLogWidget;
    PlotWidget* mPlotWidget;

    QTimer* mTimerUpdateStatus;

    // a container for collected rays, or rather the world coordinates of where they ended
    Octree* mOctree;

    GlWidget *mGlWidget;

    void addRandomPoint();
    void keyPressEvent(QKeyEvent* event);

    void processIncomingPoints();
    void processIncomingImages();

    QMap<QString, CameraWindow*> mCameraWindows;

    void processPacket(QByteArray data);

private slots:
    void slotSocketConnected(void);
    void slotSocketDisconnected(void);
    void slotReadSocket(void);
    void slotSocketError(QAbstractSocket::SocketError socketError);
    void slotConnect(void);
    void slotExportCloud(void);

    void slotWayPointInsert(QString, int, const WayPoint&);
    void slotWayPointDelete(QString, int);

    void slotSendData(const QByteArray &data);
    void slotGetStatus(void);

public:
    BaseStation(void);
    ~BaseStation();

//    const QVector3D& getCurrentVehiclePosition(void) const;
    const WayPoint getNextWayPoint(void) const;
};

#endif
