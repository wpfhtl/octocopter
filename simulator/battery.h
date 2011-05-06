#ifndef BATTERY_H
#define BATTERY_H

#include <QObject>
#include <QDebug>
#include <QTimer>
#include <QTime>

#include "simulator.h"

class Simulator;

class Battery : public QObject
{
Q_OBJECT

    // we use SI base units for all parameters: volt, ampere, seconds.

private:
    QTimer mUpdateTimer;
//    double mTimeFactor;
    double mCapacity; // in AH
    double mDischargeCurrent;  // in A, how much is drawn from the battery right now.
    double mEnergy;   // in AH, the energy left in the battery
    double mMaxVoltage;  // in Volt
    int mChargeStatusInPercent;
    Simulator* mSimulator;
    quint32 mSimulationTimeOfLastUpdate;

public:
    Battery(Simulator* simulator, const double &voltageMax = 12.0, const double &capacity = 4.0);
    void charge();

    double voltageCurrent(void) const;
    double voltageMax(void) const;

    double energy(void) const;
    double capacity(void) const;

public slots:
    void slotSetDischargeCurrent(const double &current);
//    void slotSetTimeFactor(const double &timeFactor);
    void slotStart();
    void slotPause();

signals:
    void chargeStatusChanged(int);

private slots:
    void slotUpdate();
};

#endif
