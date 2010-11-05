#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
#include <Ogre.h>
#include "ui_controlwidget.h"
#include "triangulator.h"

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

public:
    ControlWidget(Triangulator *triangulator);
    ~ControlWidget();

public slots:
    void slotUpdatePose(const QVector3D &position, const QQuaternion &rot);
    void slotUpdateDynamics(QVector3D linearVelocity);
    void slotUpdateWayPoints(QList<QVector3D> waypoints);

private slots:
    void slotUpdateBattery(const int chargeStateInPercent);
    void slotSimulationStarted();
    void slotSimulationPaused();

    void slotWayPointPrepend();
    void slotWayPointAppend();
    void slotWayPointDelete(int row, int column);

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

    void wayPointPrepend(QVector3D);
    void wayPointAppend(QVector3D);
    void wayPointDelete(int row, QVector3D);
};

#endif // ControlWidget_H
