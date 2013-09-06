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

class FlightStateRestriction;

class ControlWidget : public QDockWidget, public Ui::ControlWidget
{
Q_OBJECT
private:
    QString getBackgroundCss(const bool& error = true, const bool& dark = false);

    void updateWayPointTable();
    void initWayPointTable();
    QStyle* mStyle; // so we can delete it later, else valgrind starts bitching.

    WayPointList mWayPointList;

public:
    ControlWidget(QWidget *widget);
    ~ControlWidget();

public slots:
    void slotSetFlightState(const FlightState *const fs);
    void slotSetFlightStateRestriction(const FlightStateRestriction *const fsr);
    void slotUpdatePose(const Pose *const pose);
    void slotUpdateVehicleStatus(const VehicleStatus* const vs);
    void slotUpdateInsStatus(const GnssStatus* const gnssStatus);
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
    void slotWayPointClear();
    void slotWayPointChanged(int, int);

    void slotWayPointLoad();
    void slotWayPointSave();

    void slotEmitWaypoints();

signals:
    void volumeGlobal(Box3D);
    void volumeLocal(Box3D);
    void wayPointSelected(int);
    void showUserInterface();
    void setScannerState(bool enabled);

    // These signals are emitted when the controlwidget wants waypoints to be changed
    void wayPoints(const QList<WayPoint>* const, const WayPointListSource);
};

#endif
