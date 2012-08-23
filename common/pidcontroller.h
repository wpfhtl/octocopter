#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>
#include <QDebug>
#include <QTime>

// http://en.wikipedia.org/wiki/PID_controller, thanks Minorsky!

class PidController
{

private:
    QString mName;
    float mP, mI, mD;

    // In the first controller iteration, we don't want to build derivatives, they'd be waaayy off and destabilize the controller
    bool mFirstControllerRun;

    float mLastTimeDiff;
    float mLastValue;
    float mValueDesired;
    float mPreviousError;
    float mLastError;
    float mLastDerivative;
    float mErrorIntegral;
    float mLastOutput;

    QTime mTimeOfLastUpdate;

public:
    PidController(const QString& name = QString(), const float p = 0.0f, const float i = 0.0f, const float d = 0.0f);

    void setName(const QString& name) {mName = name;}
    void setWeights(const float p, const float i, const float d);
    void setDesiredValue(const float value) {mValueDesired = value;}

    float computeOutput(const float input);

    QString toString() const;

    const float& getLastError() {return mLastError;}
    
    void reset();
//    void slotClearErrorIntegral() {mErrorIntegral = 0.0f;}
    
    // for streaming
    friend QDataStream& operator<<(QDataStream &out, const PidController &pc)
    {
        out << pc.mName;
        out << pc.mP << pc.mI << pc.mD;
        out << pc.mFirstControllerRun;
        out << pc.mLastTimeDiff;
        out << pc.mLastValue;
        out << pc.mValueDesired;
        out << pc.mPreviousError;
        out << pc.mLastError;
        out << pc.mLastDerivative;
        out << pc.mErrorIntegral;
        out << pc.mLastOutput;

        return out;
    }

    friend QDataStream& operator>>(QDataStream &in, PidController &pc)
    {
        in >> pc.mName;
        in >> pc.mP;
        in >> pc.mI;
        in >> pc.mD;
        in >> pc.mFirstControllerRun;
        in >> pc.mLastTimeDiff;
        in >> pc.mLastValue;
        in >> pc.mValueDesired;
        in >> pc.mPreviousError;
        in >> pc.mLastError;
        in >> pc.mLastDerivative;
        in >> pc.mErrorIntegral;
        in >> pc.mLastOutput;

        return in;
    }
};

// for using qDebug();
QDebug operator<<(QDebug dbg, const PidController &mc);

#endif // PIDCONTROLLER_H
