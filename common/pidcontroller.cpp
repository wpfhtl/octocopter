#include "pidcontroller.h"

PidController::PidController(/*const QString& name, */const float p, const float i, const float d) :
//    mName(name),
    mP(p),
    mI(i),
    mD(d)
{
    reset();
}

void PidController::reset()
{
    mFirstControllerRun = true;
    mErrorIntegral = 0.0f;
    mLastTimeDiff = 0.0f;
    mLastError = 0.0f;
    mLastDerivative = 0.0f;
    mLastValue = 0.0f;
    mLastOutputP = 0.0f;
    mLastOutputI = 0.0f;
    mLastOutputD = 0.0f;
}

void PidController::setWeights(const float p, const float i, const float d)
{
    mP = p;
    mI = i;
    mD = d;
}

float PidController::computeOutput(const float input)
{
    mLastTimeDiff = qBound(
                0.015f,
                mTimeOfLastUpdate.msecsTo(QTime::currentTime()) / 1000.0f,
                0.2f);

    mLastValue = input;
    mLastError = mValueDesired - mLastValue;
    mErrorIntegral += mLastError * mLastTimeDiff;
    mLastDerivative = mFirstControllerRun ? 0.0f : (mLastError - mPreviousError) / mLastTimeDiff;
    mLastOutputP = mP * mLastError;
    mLastOutputI = mI * mErrorIntegral;
    mLastOutputD = mD * mLastDerivative;

    float output = mLastOutputP + mLastOutputI + mLastOutputD;

    // "amplify" smaller numbers to survive becoming integers :)
    output = output > 0.0f ? ceil(output) : floor(output);

//    qDebug() << toString();

    mPreviousError = mLastError;
    mFirstControllerRun = false;

    mTimeOfLastUpdate = QTime::currentTime();

    return output;
}

QString PidController::toString() const
{
    return QString ("p%1 i%2 d%3, firstrun: %4, time %5, value %6, should %7, prev-error %8, error %9, deriv %10, integ %11, output %12")
//            .arg(mName)
            .arg(mP, 3, 'f', 2).arg(mI, 3, 'f', 2).arg(mD, 3, 'f', 2)
            .arg(mFirstControllerRun)
            .arg(mLastTimeDiff, 3, 'f', 2)
            .arg(mLastValue, 3, 'f', 2)
            .arg(mValueDesired, 3, 'f', 2)
            .arg(mPreviousError, 3, 'f', 2)
            .arg(mLastError, 3, 'f', 2)
            .arg(mLastDerivative, 3, 'f', 2)
            .arg(mErrorIntegral, 3, 'f', 2)
            .arg(mLastOutputP + mLastOutputI + mLastOutputD, 3, 'f', 2);
}

QDebug operator<<(QDebug dbg, const PidController &pc)
{
    dbg << pc.toString();
    return dbg;
}

void PidController::setWeights(const QMap<QString,float>* const controllerWeights)
{
    mP = controllerWeights->value("p", 0.0f);
    mI = controllerWeights->value("i", 0.0f);
    mD = controllerWeights->value("d", 0.0f);
}

const bool PidController::hasSameWeights(const PidController* const p) const
{
    if(p->mP == mP && p->mI == mI && p->mD == mD)
        return true;
    else
        return false;
}
