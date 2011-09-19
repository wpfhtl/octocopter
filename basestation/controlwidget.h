#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
//#include <Ogre.h>
#include "ui_controlwidget.h"
#include "basestation.h"
#include "waypoint.h"
#include "gpsstatusinformation.h"
#include "common.h" // for hash()

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class BaseStation;

class ControlWidget : public QDockWidget, public Ui::ControlWidget
{
Q_OBJECT
private:
    BaseStation *mBaseStation;
    QTimer mTimerRtkIndicator;
//    QList<WayPoint> mWayPoints;

    void initWayPointTable();

public:
    ControlWidget(BaseStation *baseStation);
    ~ControlWidget();

public slots:
    void slotUpdatePose(const Pose &pose);
    void slotUpdateMissionRunTime(const quint32& time);
    void slotUpdateBattery(const float& voltageCurrent);
    void slotUpdateWirelessRssi(const qint8& wirelessRssi);
    void slotUpdateBarometricHeight(const qint16& barometricHeight);
    void slotUpdateGpsStatus(const quint8& gnssMode, const quint8& integrationMode, const quint16& info, const quint8& error, const quint8& numSatellitesTracked, const quint8& lastPvtAge, const QString& status);
    void slotUpdateRtkStatus(bool working = false);
    void slotUpdateRoverConnection(bool connected);

    void slotSetWayPointCoordinateFields(Qt::MouseButton, QVector3D);

    // Called by FlightPlanner when it has changed its internal list.
    void slotWayPointInserted(const quint16& index, const WayPoint& waypoint);
    void slotWayPointDeleted(const quint16& index);
    void slotSetWayPoints(QList<WayPoint>);
    void slotWayPointsCleared();

private slots:
    void slotSimulationStarted();
    void slotSimulationPaused();

    // Called when buttons are pressed, simply emit signals to be processed by FlightPlanner
    void slotWayPointPrepend();
    void slotWayPointAppend();
    void slotWayPointDelete();
    void slotWayPointChange(int, int);
    void slotWayPointUp();
    void slotWayPointDown();

    void slotWayPointLoad();
    void slotWayPointSave();

    void slotSetScanVolume();

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

    void setScanVolume(QVector3D, QVector3D);

    void generateWaypoints();

    void wayPointInsert(const quint16& index, const WayPoint&);
    void wayPointDelete(const quint16& index);
    void wayPointSwap(const quint16& i, const quint16& j);
};

#endif // ControlWidget_H
