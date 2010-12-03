#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
#include <Ogre.h>
#include "ui_controlwidget.h"
#include "triangulator.h"
#include "common.h" // for hash()

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class Triangulator;
//class DialogConfiguration;

class ControlWidget : public QDockWidget, public Ui::ControlWidget
{
Q_OBJECT
private:
    Triangulator *mTriangulator;
//    Battery* mBattery;
//    CoordinateConverter *mCoordinateConverter;
    QList<QVector3D> mWayPoints;

    void initWayPointTable();

public:
    ControlWidget(Triangulator *triangulator);
    ~ControlWidget();

    const QVector3D getNextWayPoint() const;

public slots:
    void slotUpdatePose(const QVector3D &position, const QQuaternion &rot);
    void slotUpdateDynamics(QVector3D linearVelocity);
    void slotUpdateWayPoints(QList<QVector3D> waypoints);

    // Called my FlightPlanner to add a new waypoint.
    void slotNewWayPoint(const QVector3D);

private slots:
    void slotUpdateBattery(const int chargeStateInPercent);
    void slotSimulationStarted();
    void slotSimulationPaused();

    void slotWayPointPrepend();
    void slotWayPointAppend();
    void slotWayPointDelete();
    void slotWayPointChange(int row, int column);

    void slotWayPointUp();
    void slotWayPointDown();

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

    void wayPointInsert(QString, int index, QVector3D);
    void wayPointDelete(QString, int index);
};

#endif // ControlWidget_H
