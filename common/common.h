#ifndef COMMON_H
#define COMMON_H

#include <QList>
#include <QVector3D>
#include <QString>
#include <QDateTime>
#include <QDebug>

#define RAD2DEG(RAD) ((RAD)*180/M_PI)
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

#define SQUARE(X) ((X)*(X))

// So I can do qDebug() << "string is" << Q(myStringObject) << "."; without having quotes around the string-value inthe output
#define Q(string) (string).toStdString().c_str()

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

QString t();

enum FlightState
{
    UserControl,
    ApproachWayPoint,
    Hover,
    Idle
};

QString getFlightStateString(const FlightState);

enum LogImportance
{
    Information,
    Warning,
    Error,
    Desaster
};

#endif // COMMON_H
