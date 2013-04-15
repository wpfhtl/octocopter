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
    /*struct */timeval mTimeStart/*, mTimeStop*/;

public:
    Profiler(const QString name);
    ~Profiler();
};

#endif
