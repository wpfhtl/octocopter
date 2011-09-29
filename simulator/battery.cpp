#include "battery.h"

Battery::Battery(Simulator* simulator, const double &voltage, const double &capacity) :
    mUpdateTimer(this),
    mMaxVoltage(voltage),
    mCapacity(capacity),
    mEnergy(capacity),
    mDischargeCurrent(0.0),
    mChargeStatusInPercent(100)
{
    mSimulator = simulator;
    mUpdateTimer.setInterval(500); // 2 times per second shall be enough to get a realistically discharging battery.

    connect(&mUpdateTimer, SIGNAL(timeout()), SLOT(slotUpdate()));
}

void Battery::slotSetDischargeCurrent(const double &current)
{
//    qDebug() << "Battery::slotSetDischargeCurrent(): discharging with" << current << "amps,"  << currentVoltage() * current << "watts.";
    mDischargeCurrent = current;
}

void Battery::slotUpdate(void)
{
    const int currentSimulationTime = mSimulator->getSimulationTime();
    float elapsedSimuationTime = ((float)(currentSimulationTime - mSimulationTimeOfLastUpdate))/1000.0;
    mEnergy -= elapsedSimuationTime * mDischargeCurrent / 3600.0;

    // if the percentage changed, emit a signal.
    const int currentChargeStatusInPercent = (int) (mEnergy / mCapacity * 100);
    if(mChargeStatusInPercent != currentChargeStatusInPercent)
    {
        emit chargeStatusChanged(currentChargeStatusInPercent);
    }
    mChargeStatusInPercent = currentChargeStatusInPercent;

    mSimulationTimeOfLastUpdate = currentSimulationTime;

//    qDebug() << "Battery::slotUpdate(): battery is now at" << mChargeStatusInPercent << "percent capacity, voltage is" << voltageCurrent() << "of" << mMaxVoltage;
}

double Battery::voltageCurrent(void) const
{
    // WARNING: find a better (nonlinear) discharge curve, empty Battery now has 0,8 * mVoltage
//    qDebug() << "Battery::voltageCurrent(): this" << this << "max" << mMaxVoltage << "energy" << mEnergy << "cap" << mCapacity;
    return (mMaxVoltage * 0.8) + (mMaxVoltage * 0.2 * (mEnergy / mCapacity));
}

double Battery::voltageMax(void) const
{
    return mMaxVoltage;
}

double Battery::energy(void) const
{
    return mEnergy;
}

double Battery::capacity(void) const
{
    return mCapacity;
}

void Battery::charge(void)
{
    mEnergy = mCapacity;
}

void Battery::slotStart()
{
    mUpdateTimer.start();
    mSimulationTimeOfLastUpdate = mSimulator->getSimulationTime();
}

void Battery::slotPause()
{
    mUpdateTimer.stop();
}

void Battery::slotSetCapacity(const double& capacity)
{
    Q_ASSERT(capacity > 0.5 && "Battery with less than 0.5 Ah? Crazy like a fool...");
    mCapacity = capacity;
    charge();
}

void Battery::slotSetVoltageMax(const double& voltage)
{
    mMaxVoltage = voltage;
    charge();
}
