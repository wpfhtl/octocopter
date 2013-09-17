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

template <typename T>
T swap_endian(T u)
{
    union
    {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

struct PositionGeodetic
{
    // degrees, meters
    float longitude;
    float latitude;
    float elevation;
};

enum class WayPointListSource
{
    WayPointListSourceControlWidget,
    WayPointListSourceFlightPlanner,
    WayPointListSourceRover,
    WayPointListSourceUnknown
};

enum class OperatingMode
{
    OperatingOnline,    // connected to simulator or kopter
    OperatingOffline    // using LogPlayer
};

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

enum LogImportance
{
    Information,
    Warning,
    Error,
    Desaster
};


// compute the next higher power of 2 of 32-bit v
/*
quint32 nextHigherPowerOfTwo(quint32 v)
{
    // decrements, then sets all bits below its most significant bit to 1, then it increments
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}*/

struct Vector3i
{
    qint16 x,y,z;

    Vector3i() {x = y = z = 0;}
    Vector3i(qint16 x, qint16 y, qint16 z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

// for using qDebug()
QDebug operator<<(QDebug dbg, const Vector3i &v);

#endif // COMMON_H
