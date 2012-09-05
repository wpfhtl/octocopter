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
    mBeforeFirstIteration = true;
    mValueDesired = 0.0f;
    mIntegral = 0.0f;
    mTimeDiff = 0.0f;
    mErrorPrevious = 0.0f;
    mError = 0.0f;
    mDerivative = 0.0f;
    mValue = 0.0f;
    mOutputP = 0.0f;
    mOutputI = 0.0f;
    mOutputD = 0.0f;
}

void PidController::setWeights(const float p, const float i, const float d)
{
    qDebug() << "PidController::setWeights(): setting weights:" << p << i << d;
    mP = p;
    mI = i;
    mD = d;
}

float PidController::computeOutputFromValue(const float &input)
{
    mErrorPrevious = mError;

    mTimeDiff = qBound(
                0.015f,
                mTimeOfLastUpdate.msecsTo(QTime::currentTime()) / 1000.0f,
                0.2f);

    mValue = input;
    mError = mValueDesired - mValue;
    mIntegral += mError * mTimeDiff;
    mDerivative = mBeforeFirstIteration ? 0.0f : (mError - mErrorPrevious) / mTimeDiff;
    mOutputP = mP * mError;
    mOutputI = mI * mIntegral;
    mOutputD = mD * mDerivative;

    float output = mOutputP + mOutputI + mOutputD;

    // "amplify" smaller numbers to survive becoming integers :)
    output = output > 0.0f ? ceil(output) : floor(output);

//    qDebug() << toString();

    mBeforeFirstIteration = false;

    mTimeOfLastUpdate = QTime::currentTime();

    return output;
}

float PidController::computeOutputFromError(const float& error)
{
    mErrorPrevious = mError;

    mTimeDiff = qBound(
                0.015f,
                mTimeOfLastUpdate.msecsTo(QTime::currentTime()) / 1000.0f,
                0.2f);

    mValue = 0.0f; // set this to some absurd value.
    mError = error;
    mIntegral += mError * mTimeDiff;
    mDerivative = mBeforeFirstIteration ? 0.0f : (mError - mErrorPrevious) / mTimeDiff;
    mOutputP = mP * mError;
    mOutputI = mI * mIntegral;
    mOutputD = mD * mDerivative;

    float output = mOutputP + mOutputI + mOutputD;

    // "amplify" smaller numbers to survive becoming integers :)
    output = output > 0.0f ? ceil(output) : floor(output);

//    qDebug() << toString();

    mBeforeFirstIteration = false;

    mTimeOfLastUpdate = QTime::currentTime();

    return output;
}

QString PidController::toString() const
{
    return QString ("p%1 i%2 d%3, firstrun: %4, time %5, value %6, should %7, prev-error %8, error %9, deriv %10, integ %11, output %12")
//            .arg(mName)
            .arg(mP, 3, 'f', 2).arg(mI, 3, 'f', 2).arg(mD, 3, 'f', 2)
            .arg(mBeforeFirstIteration)
            .arg(mTimeDiff, 3, 'f', 2)
            .arg(mValue, 3, 'f', 2)
            .arg(mValueDesired, 3, 'f', 2)
            .arg(mErrorPrevious, 3, 'f', 2)
            .arg(mError, 3, 'f', 2)
            .arg(mDerivative, 3, 'f', 2)
            .arg(mIntegral, 3, 'f', 2)
            .arg(mOutputP + mOutputI + mOutputD, 3, 'f', 2);
}

QDebug operator<<(QDebug dbg, const PidController &pc)
{
    dbg << pc.toString();
    return dbg;
}

void PidController::setWeights(const QMap<QChar, float> *const controllerWeights)
{
    mP = controllerWeights->value('p', 0.0f);
    mI = controllerWeights->value('i', 0.0f);
    mD = controllerWeights->value('d', 0.0f);
}

const bool PidController::hasSameWeights(const PidController* const p) const
{
    if(p->mP == mP && p->mI == mI && p->mD == mD)
        return true;
    else
        return false;
}
