#include "profiler.h"

Profiler::Profiler(const QString name) : mName(name)
{
    if(gettimeofday(&mTimeStart, NULL))
      Q_ASSERT("Profiler::Profiler(): couldn't get time, aborting" && false);
}

Profiler::~Profiler()
{
    timeval mTimeStop;
    if(gettimeofday(&mTimeStop, NULL))
      Q_ASSERT("Profiler::~Profiler(): couldn't get time, aborting" && false);

    struct timeval difference;

    difference.tv_sec  = mTimeStop.tv_sec  - mTimeStart.tv_sec ;
    difference.tv_usec = mTimeStop.tv_usec - mTimeStart.tv_usec;

    while(difference.tv_usec < 0)
    {
        difference.tv_usec+=1000000;
        difference.tv_sec -=1;
    }

    const long long interval = 1000000LL * difference.tv_sec + difference.tv_usec;

    qDebug() << "Profiler::~Profiler(): action" << mName << "took" << interval << "usec /" << interval/1000 << "ms";
}
