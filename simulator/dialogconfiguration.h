#ifndef DIALOGCONFIGURATION_H
#define DIALOGCONFIGURATION_H

#include <QDockWidget>
#include <QSettings>
#include <QDebug>
#include "simulator.h"
#include "laserscanner.h"
#include "camera.h"
#include "ogrewidget.h"

#include "ui_dialogconfiguration.h"

class Simulator;

class DialogConfiguration : public QDialog, public Ui::DialogConfiguration
{
    Q_OBJECT

public:
    DialogConfiguration(Simulator *simulator);
    ~DialogConfiguration();

private:
    Simulator *mSimulator;
    OgreWidget *mOgreWidget;
    QSettings mSettings;

public slots:

    void slotSaveConfiguration();
    void slotReadConfiguration();

private slots:

    void slotReadConfigurationLaserScanner();
    void slotSaveConfigurationLaserScanner();

    void slotReadConfigurationCamera();
    void slotSaveConfigurationCamera();

    void slotLaserScannerDetailChanged(QTableWidgetItem* item);
    void slotCameraDetailChanged(QTableWidgetItem* item);

    void slotLaserScannerAdd();
    void slotLaserScannerDel();

    void slotCameraAdd();
    void slotCameraDel();

signals:
    void timeFactorChanged(double);
    void simulationStart();
    void simulationPause();

};

#endif
