#ifndef PROFILER_H
#define PROFILER_H

#include <QString>
#include <QDebug>
//#include <math.h>
#include <sys/time.h>

class Profiler
{
private:
    QString mName;
    qint32 mMinTimeForWarningUsec;
    /*struct */timeval mTimeStart/*, mTimeStop*/;

public:
    Profiler(const QString name, const qint32 minTimeForWarningUsec = 0);
    ~Profiler();
};

#endif
