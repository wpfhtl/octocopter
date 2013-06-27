#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
#include "ui_controlwidget.h"
#include <waypoint.h>
#include "waypointlist.h"
#include <gnssstatus.h>
#include <flightstate.h>
#include <vehiclestatus.h>
#include <pose.h>
#include <common.h> // for hash()

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class ControlWidget : public QDockWidget, public Ui::ControlWidget
{
Q_OBJECT
private:
    QString getBackgroundCss(const bool& error = true, const bool& dark = false);

    void updateWayPointTable();
    void initWayPointTable();
    QStyle* mStyle; // so we can delete it later, else valgrind starts bitching.

    // We don't use WayPointList, as that uses OpenGL stuff that we don't need here.
    QList<WayPoint> mWayPointList;

public:
    ControlWidget(QWidget *widget);
    ~ControlWidget();

public slots:
    void slotFlightStateChanged(const FlightState *const fs);
    void slotUpdatePose(const Pose *const pose);
    void slotUpdateVehicleStatus(const VehicleStatus* const vs);
    void slotUpdateGnssStatus(const GnssStatus* const gnssStatus);
    void slotUpdateConnectionDiffCorr(const bool working);
    void slotUpdateConnectionRover(const bool connected);

    // Called by FlightPlanner when it has changed its internal list.
    void slotSetWayPoints(const QList<WayPoint> *const, const WayPointListSource source);

private slots:
    void slotResizeToMinimum();

    // Called when buttons are pressed, simply emit signals to be processed by FlightPlanner
    void slotWayPointPrepend();
    void slotWayPointAppend();
    void slotWayPointDelete();
    void slotWayPointChanged(int, int);

    void slotWayPointLoad();
    void slotWayPointSave();

    void slotEmitWaypoints();

    void slotSetScanVolume();

signals:
    void setScanVolume(QVector3D, QVector3D);
    void showUserInterface();

    // These signals are emitted when the controlwidget wants waypoints to be changed
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);
};

#endif
