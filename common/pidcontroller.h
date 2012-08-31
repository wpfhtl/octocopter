#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>
#include <QDebug>
#include <QTime>

// http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!

class PidController
{

private:
//    QString mName;
    float mP, mI, mD;

    // In the first controller iteration, we don't want to build derivatives, they'd be waaayy off and destabilize the controller
    bool mBeforeFirstIteration;

    float mTimeDiff;
    float mValue;
    float mValueDesired;
    float mErrorPrevious;
    float mError;
    float mDerivative;
    float mIntegral;
    float mOutputP, mOutputI, mOutputD;

    QTime mTimeOfLastUpdate;


public:
    PidController(/*const QString& name = QString(),*/ const float p = 0.0f, const float i = 0.0f, const float d = 0.0f);

    const float getWeightP() const {return mP;}
    const float getWeightI() const {return mI;}
    const float getWeightD() const {return mD;}

    const float& getTimeDiff() const {return mTimeDiff;}
    const float& getValueDesired() const {return mValueDesired;}
    const float& getValue() const {return mValue;}
    const float& getError() const {return mError;}
    const float& getErrorPrevious() const {return mErrorPrevious;}
    const float& getDerivative() const {return mDerivative;}
    const float& getIntegral() const {return mIntegral;}

    const float& getOutputP() const {return mOutputP;}
    const float& getOutputI() const {return mOutputI;}
    const float& getOutputD() const {return mOutputD;}


//    void setName(const QString& name) {mName = name;}
    void setWeights(const float p, const float i, const float d);
    void setWeights(const QMap<QString, float> *const controllerWeights);
    void setDesiredValue(const float value) {mValueDesired = value;}

    float computeOutputFromValue(const float& input);
    float computeOutputFromError(const float& error);

    // returns true when the controller has been reset, but not yet used.
    const bool beforeFirstIteration() const {return mBeforeFirstIteration;}

    QString toString() const;

    void reset();

    const float getLastOutput() const {return mOutputP + mOutputI + mOutputD;}
    const float getLastOutputP() const {return mOutputP;}
    const float getLastOutputI() const {return mOutputI;}
    const float getLastOutputD() const {return mOutputD;}

    const bool hasSameWeights(const PidController* const p) const;

//    void slotClearErrorIntegral() {mErrorIntegral = 0.0f;}
    
    // for streaming
    friend QDataStream& operator<<(QDataStream &out, const PidController &pc)
    {
//        out << pc.mName;
        out << pc.mP << pc.mI << pc.mD;
        out << pc.mBeforeFirstIteration;
        out << pc.mTimeDiff;
        out << pc.mValue;
        out << pc.mValueDesired;
        out << pc.mErrorPrevious;
        out << pc.mError;
        out << pc.mDerivative;
        out << pc.mIntegral;
        out << pc.mOutputP;
        out << pc.mOutputI;
        out << pc.mOutputD;

        return out;
    }

    friend QDataStream& operator>>(QDataStream &in, PidController &pc)
    {
//        in >> pc.mName;
        in >> pc.mP;
        in >> pc.mI;
        in >> pc.mD;
        in >> pc.mBeforeFirstIteration;
        in >> pc.mTimeDiff;
        in >> pc.mValue;
        in >> pc.mValueDesired;
        in >> pc.mErrorPrevious;
        in >> pc.mError;
        in >> pc.mDerivative;
        in >> pc.mIntegral;
        in >> pc.mOutputP;
        in >> pc.mOutputI;
        in >> pc.mOutputD;

        return in;
    }
};

// for using qDebug();
QDebug operator<<(QDebug dbg, const PidController &mc);

#endif // PIDCONTROLLER_H
