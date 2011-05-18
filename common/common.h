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

//bool isBitSet(quint8 number, quint8 bit);
bool testBit(quint16 number, quint8 bit);
bool testBitEqual(quint16 number1, quint16 number2, quint8 bit);

// fast exponentiation algorithm
inline int ipow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}


enum LogImportance
{
    Information,
    Warning,
    Error,
    Desaster
};

#endif // COMMON_H
