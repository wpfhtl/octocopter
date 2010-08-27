#ifndef PROFILER_H
#define PROFILER_H

#include <QObject>
#include <QSettings>
#include <Ogre.h>
#include <math.h>

#include "coordinategps.h"
//#include "coordinateogre.h"

class Profiler : public QObject
{

    Q_OBJECT
private:
    struct timeval timeStart, timeStop;

public:
    Profiler(void);
    void start();
    long long stop(const QString action = QString());
};

#endif
