#include "profiler.h"

Profiler::Profiler(void)
{
}

void Profiler::start()
{
    if(gettimeofday(&timeStart, NULL))
      Q_ASSERT("Couldn't get time, aborting" && false);
}

long long Profiler::stop(const QString action)
{
    if(gettimeofday(&timeStop, NULL))
      Q_ASSERT("Couldn't get time, aborting" && false);

    struct timeval difference;

    difference.tv_sec  = timeStop.tv_sec  - timeStart.tv_sec ;
    difference.tv_usec = timeStop.tv_usec - timeStart.tv_usec;

    while(difference.tv_usec < 0)
    {
        difference.tv_usec+=1000000;
        difference.tv_sec -=1;
    }

    const long long interval = 1000000LL * difference.tv_sec + difference.tv_usec;

//    qDebug() << "Profiler::stop(): action" << action << "took" << interval << "usec";

    return interval;
}
