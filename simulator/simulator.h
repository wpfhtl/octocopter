#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QMainWindow>
#include <QList>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>

#include "ogrewidget.h"
#include "battery.h"
#include "physics.h"
#include "laserscanner.h"
#include "baseconnection.h"
#include "flightcontroller.h"
#include "camera.h"
#include "statuswidget.h"
#include "coordinateconverter.h"

#include "engine.h"

// probably not the right place to put this
#define deg2rad(x) (x)*M_PI/180.0
#define rad2deg(x) (x)*180.0/M_PI

class Physics;
class Camera;
class LaserScanner;
class StatusWidget;
class BaseConnection;

class Simulator : public QMainWindow
{
    Q_OBJECT

public:
    Simulator(void);
    ~Simulator(void);
    double getTimeFactor(void) const;
    bool isPaused(void) const;
    int getSimulationTime(void) const; // returns milliseconds since start of simulation, scaled by timeFactor
    QList<LaserScanner*>* getLaserScannerList(void);
//    CoordinateConverter* getCoordinateConverter(void);
    CoordinateConverter *mCoordinateConverter;
    Battery* mBattery;
    OgreWidget* mOgreWidget;
    Physics *mPhysics;
    BaseConnection* mBaseConnection;

    QList<LaserScanner*> *mLaserScanners;
    QList<Camera*> *mCameras;

    FlightController* mFlightController;

private:
    mutable QMutex mMutex;

    QTimer* mUpdateTimer;
    double mTimeFactor;
    QTime mTimeSimulationStart; // when simulation was started
    QTime mTimeSimulationPause; // when simulation was paused, invalid when it's not currently paused.
    StatusWidget* mStatusWidget;


private slots:
    void slotOgreInitialized(void);
    void slotSetTimeFactor(double);
    void slotNotifyDevicesOfNewTimeFactor();
    void slotUpdate();

public slots:
    void slotSimulationStart(void);
    void slotSimulationPause(void);

    void slotShowMessage(const QString);

signals:

};

#endif
