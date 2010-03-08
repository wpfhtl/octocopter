#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QMainWindow>
#include <QList>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>

#include "ogrewidget.h"
#include "battery.h"
#include "vehicle.h"
#include "laserscanner.h"
#include "statuswidget.h"
#include "simulationcontrolwidget.h"
#include "coordinateconverter.h"

class Vehicle;
class LaserScanner;
class SimulationControlWidget;

class Simulator : public QMainWindow
{
    Q_OBJECT

public:
    Simulator(void);
    float getTimeFactor(void) const;
    bool isPaused(void) const;
    int getSimulationTime(void) const; // returns milliseconds since start of simulation, scaled by timeFactor
    QList<LaserScanner*>* getLaserScannerList(void);

private:
    mutable QMutex mMutex;

    float mTimeFactor;
    QTime mTimeSimulationStart; // when simulation was started
    QTime mTimeSimulationPause; // when simulation was paused, invalid when it's not currently paused.
    OgreWidget* mOgreWidget;
    Battery* mBattery;
    StatusWidget* mStatusWidget;
    SimulationControlWidget* mSimulationControlWidget;
    CoordinateConverter mCoordinateConverter;
    Vehicle *mVehicle;

    QList<LaserScanner*> *mLaserScanners;

private slots:
    void slotAddVehicle(void);
    void slotSimulationStart(void);
    void slotSimulationPause(void);
    void slotSetTimeFactor(double);

public slots:
    void slotScanFinished(QList<CoordinateGps>);
};

#endif
