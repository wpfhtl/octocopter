#ifndef STATUSWIDGET_H
#define STATUSWIDGET_H

#include <QtGui>
//#include <QPlastiqueStyle>
#include "coordinateconverter.h"
#include "dialogconfiguration.h"
#include "ui_statuswidget.h"
#include "battery.h"
#include <pose.h>

#define QT_USE_FAST_CONCATENATION
#define QT_USE_FAST_OPERATOR_PLUS

class Battery;
class DialogConfiguration;

class StatusWidget : public QDockWidget, public Ui::DockWidget
{
Q_OBJECT
private:
    Simulator *mSimulator;
    Battery* mBattery;
    CoordinateConverter *mCoordinateConverter;

public:
    StatusWidget(Simulator *simulator);
    ~StatusWidget();
    double getTimeFactor() const;
    DialogConfiguration *mDialogConfiguration;

public slots:
    void slotSetButtonStartEnabled(bool);
    void slotSetButtonPauseEnabled(bool);
    void slotUpdatePose(const Pose *const pose);

private slots:
    void slotUpdateBattery(const int chargeStateInPercent);
    void slotUpdateVisualization(QSize windowSize, int triangles, float fps);
    void slotSimulationStarted();
    void slotSimulationPaused();
    void slotShowConfiguration();

signals:
    void windDetailChanged(bool,float);
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

};

#endif // STATUSWIDGET_H
