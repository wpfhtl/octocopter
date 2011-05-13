#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
//#include <Ogre.h>
#include "ui_controlwidget.h"
#include "basestation.h"
#include "waypoint.h"
#include "common.h" // for hash()

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class BaseStation;
//class DialogConfiguration;

class ControlWidget : public QDockWidget, public Ui::ControlWidget
{
Q_OBJECT
private:
    BaseStation *mBaseStation;
//    Battery* mBattery;
//    CoordinateConverter *mCoordinateConverter;
    QList<WayPoint> mWayPoints;

    void initWayPointTable();

public:
    ControlWidget(BaseStation *baseStation);
    ~ControlWidget();

    const WayPoint getNextWayPoint() const;

public slots:
    void slotUpdatePose(const Pose &pose);
//    void slotUpdateDynamics(QVector3D linearVelocity);
    void slotUpdateWayPoints(const QList<WayPoint>& waypoints);
    void slotUpdateMissionRunTime(const quint32& time);
    void slotUpdateBattery(const float& voltageCurrent);
    void slotUpdateWirelessRssi(const qint8& wirelessRssi);
    void slotUpdateBarometricHeight(const qint16& barometricHeight);
    // Called my FlightPlanner to add a new waypoint.
//    void slotNewWayPoint(const WayPoint&);
    void slotNewWayPoints(const QList<WayPoint>&);
    void slotUpdateGpsStatus(const quint8& mode, const quint8& info, const quint8& error, const quint8& numSatellitesTracked, const quint8& lastPvtAge, const QString& status);

    void slotRoverReachedNextWayPoint();

private slots:
    void slotSimulationStarted();
    void slotSimulationPaused();

    void slotWayPointPrepend();
    void slotWayPointAppend();
    void slotWayPointDelete();
    void slotWayPointChange(int row, int column);

    void slotWayPointUp();
    void slotWayPointDown();

    void slotSetScanVolume();

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

    void setScanVolume(QVector3D, QVector3D);

    void generateWaypoints();

    void wayPointInsert(QString, int index, const QList<WayPoint>&);
    void wayPointDelete(QString, int index);
};

#endif // ControlWidget_H
