#include "profiler.h"

Profiler::Profiler(const QString name, const qint32 minTimeForWarningUsec) : mName(name), mMinTimeForWarningUsec(minTimeForWarningUsec)
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

    const long long intervalUsec = 1000000LL * difference.tv_sec + difference.tv_usec;

    if(intervalUsec > mMinTimeForWarningUsec)
        qDebug() << "Profiler::~Profiler(): action" << mName << "took" << intervalUsec << "usec /" << intervalUsec/1000 << "ms";
}
