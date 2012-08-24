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
    mLastOutput = 0.0f;
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
    mLastOutput = (mP * mLastError) + (mI * mErrorIntegral) + (mD * mLastDerivative);

    // "amplify" smaller numbers to survive becoming integers :)
    mLastOutput = mLastOutput > 0.0f ? ceil(mLastOutput) : floor(mLastOutput);

    qDebug() << toString();

    mPreviousError = mLastError;
    mFirstControllerRun = false;

    mTimeOfLastUpdate = QTime::currentTime();

    return mLastOutput;
}

QString PidController::toString() const
{
    return QString ("controller p%1 i%2 d%3, firstrun: %4, time %5, value %6, should %7, prev-error %8, error %9, deriv %10, integ %11, output %12")
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
            .arg(mLastOutput, 3, 'f', 2);
}

QDebug operator<<(QDebug dbg, const PidController &pc)
{
    dbg << pc.toString();
    return dbg;
}

void PidController::setWeights(QMap<QString,float> controllerWeights)
{
    mP = controllerWeights["p"];
    mI = controllerWeights["i"];
    mD = controllerWeights["d"];
}

const float PidController::getWeight(const QString& weight) const
{
    if(weight.toLower() == "p") return mP;
    if(weight.toLower() == "i") return mI;
    if(weight.toLower() == "d") return mD;

    return 0.0f;
}
