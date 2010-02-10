#include "battery.h"

Battery::Battery(QObject *parent, const double &voltage, const double &capacity)
    : QObject(parent), mUpdateTimer(this), mVoltage(voltage), mCapacity(capacity), mEnergy(capacity), mChargeStatusInPercent(100)
{
    mUpdateTimer.setInterval(500); // /*20*/ 2 times per second shall be enough to get a realistically discharging battery.
    mUpdateTimer.start();
    connect(&mUpdateTimer, SIGNAL(timeout()), SLOT(slotUpdate()));
}

void Battery::setDischargeCurrent(const double &current)
{
    mDischargeCurrent = current;
}

void Battery::slotUpdate(void)
{
    mEnergy -= ((double)mUpdateTimer.interval()/1000) * mDischargeCurrent / 3600.0;

    // if the percentage changed, emit a signal.
    const int currentChargeStatusInPercent = (int) (mEnergy / mCapacity * 100);
    if(mChargeStatusInPercent != currentChargeStatusInPercent)
    {
        emit chargeStatusChanged(currentChargeStatusInPercent);
    }
    mChargeStatusInPercent = currentChargeStatusInPercent;

//    qDebug() << "Battery::slotUpdate(): battery is now at" << mChargeStatusInPercent << "percent capacity, voltage is" << currentVoltage() << "of" << mVoltage;
}

double Battery::currentVoltage(void) const
{
    // WARNING: find a better (nonlinear) discharge curve, empty Battery now has 0,8 * mVoltage
    return (mVoltage * 0.8) + (mVoltage * 0.2 * (mEnergy / mCapacity));
}

double Battery::voltage(void) const
{
    return mVoltage;
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
