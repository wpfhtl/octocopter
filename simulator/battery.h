#ifndef BATTERY_H
#define BATTERY_H

#include <QObject>
#include <QDebug>
#include <QTimer>
#include <QTime>

class Battery : public QObject
{
Q_OBJECT

    // we use SI base units for all parameters: volt, ampere, seconds.

private:
    QTimer mUpdateTimer;
    double mTimeFactor;
    double mCapacity; // in AH
    double mDischargeCurrent;  // in A, how much is drawn from the battery right now.
    double mEnergy;   // in AH, the energy left in the battery
    double mVoltage;  // in Volt
    int mChargeStatusInPercent;

public:
    Battery(QObject *parent = 0, const double &voltage = 12.0, const double &capacity = 4.0);
    void setDischargeCurrent(const double &current);
    void charge();

    double currentVoltage(void) const;
    double voltage(void) const;

    double energy(void) const;
    double capacity(void) const;

signals:
    void chargeStatusChanged(int);

private slots:
    void slotUpdate();
    void slotSetTimeFactor(const double &timeFactor);

};

#endif
