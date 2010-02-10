#ifndef SIMULATIONCONTROLWIDGET_H
#define SIMULATIONCONTROLWIDGET_H

#include <QDockWidget>
#include "simulator.h"
#include "laserscanner.h"
#include "ogrewidget.h"

#include "ui_simulationcontrolwidget.h"

//
//namespace Ui
//{
//    class SimulationControlWidget;
//}

class Simulator;

class SimulationControlWidget : public QDockWidget, public Ui::SimulationControlWidget
{
    Q_OBJECT

public:
    SimulationControlWidget(Simulator *simulator, OgreWidget* ogreWidget);
    ~SimulationControlWidget();

private:
//    Ui::SimulationControlWidget *ui;
    Simulator *mSimulator;
    OgreWidget *mOgreWidget;
    QSettings mSettings;

    void createScannersFromConfiguration(void);

private slots:
    // Called when the index-spinbox has a new value. Load that scanner's config into the form
    void slotScannerIndexChanged(void);

    // A form-element was changed, adjust the scanner's scenenode
    void slotScannerSettingsChanged(void);

    void slotScannerCreate(void);
    void slotScannerDelete(void);
    void slotScannerSaveConfiguration(void);

signals:
    void timeFactorChanged(double);
    void start();
    void pause();

};

#endif
