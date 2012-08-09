#ifndef GNSSTIME_H
#define GNSSTIME_H

#include <QString>
#include <QDebug>
#include <QTime>
#include <QDateTime>

class GnssTime
{
public:
//    GnssTime();

    static QString currentTowString();
    static qint32 currentTow();
};

#endif // GNSSTIME_H
