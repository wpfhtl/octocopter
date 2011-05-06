#ifndef COMMON_H
#define COMMON_H

#include <QList>
#include <QVector3D>
#include <QString>
#include <QDateTime>
#include <QCryptographicHash>

#include <waypoint.h>

#define RAD2DEG(RAD) ((RAD)*180/M_PI)
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

QString hash(QList<WayPoint> list);

quint32 getCurrentGpsTowTime();

enum LogImportance
{
    Information,
    Warning,
    Error,
    Desaster
};

#endif // COMMON_H
