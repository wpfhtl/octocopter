#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

#include <QtGui>
#include "ui_controlwidget.h"
#include <waypoint.h>
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

    void initWayPointTable();
    QStyle* mStyle; // so we can delete it later, else valgrind starts bitching.

public:
    ControlWidget(QWidget *widget);
    ~ControlWidget();

//    QSize 	sizeHint() const {return minimumSize();}
//    QSize 	maximumSize() const {
//        qDebug() << "maximumSIze()" << minimumSize();
//        return minimumSize();}

public slots:
    void slotFlightStateChanged(const FlightState *const fs);
    void slotUpdatePose(const Pose *const pose);
    void slotUpdateVehicleStatus(const VehicleStatus* const vs);
    void slotUpdateGnssStatus(const GnssStatus* const gnssStatus);
    void slotUpdateConnectionRtk(const bool working);
    void slotUpdateConnectionRover(const bool connected);

//    void slotSetWayPointCoordinateFields(Qt::MouseButton, QVector3D);

    // Called by FlightPlanner when it has changed its internal list.
    void slotWayPointInserted(const quint16& index, const WayPoint& waypoint);
    void slotWayPointDeleted(const quint16& index);
    void slotSetWayPoints(const QList<WayPoint> *const);
    void slotWayPointsCleared();

private slots:
    void slotResizeToMinimum();

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

    void showUserInterface();

    // These signals are emitted when the controlwidget wants waypoints to be changed on the kopter
    void wayPointInsert(const quint16& index, const WayPoint&);
    void wayPointDelete(const quint16& index);
    void wayPointSwap(const quint16& i, const quint16& j);
};

#endif
