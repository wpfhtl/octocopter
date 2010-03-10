#ifndef SIMULATIONCONTROLWIDGET_H
#define SIMULATIONCONTROLWIDGET_H

#include <QDockWidget>
#include <QSettings>
#include <QDebug>
#include "simulator.h"
#include "laserscanner.h"
#include "ogrewidget.h"

#include "ui_simulationcontrolwidget.h"

class Simulator;

class SimulationControlWidget : public QDockWidget, public Ui::SimulationControlWidget
{
    Q_OBJECT

public:
    SimulationControlWidget(Simulator *simulator, OgreWidget* ogreWidget);
    ~SimulationControlWidget();

    double getTimeFactor();

private:
//    Ui::SimulationControlWidget *ui;
    Simulator *mSimulator;
    OgreWidget *mOgreWidget;
    QSettings mSettings;

private slots:
    // Called when the index-spinbox has a new value. Load that scanner's config into the form
    void slotScannerIndexChanged(void);

    // A form-element was changed, adjust the scanner's scenenode
    void slotScannerSettingsChanged(void);

    void slotScannerCreate(void);
    void slotScannerDelete(void);
    void slotScannerLoadConfiguration(void);
    void slotScannerSaveConfiguration(void);

    void slotSimulationPaused();
    void slotSimulationStarted();

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

};

#endif
