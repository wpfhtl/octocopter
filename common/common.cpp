#include "common.h"

bool testBit(quint16 number, quint8 bit)
{
    return ((number & ipow(2, bit)) != 0);
}

bool testBitEqual(quint16 number1, quint16 number2, quint8 bit)
{
    quint16 mask = ipow(2, bit);

    return (number1 & mask) == (number2 & mask);
}

QDebug operator<<(QDebug dbg, const Box3D &box)
{
    dbg << "Box3D:" << box.min << box.max;
    return dbg;
}

QDebug operator<<(QDebug dbg, const Vector3i &v)
{
    dbg.nospace() << "Vector3i(" << v.x << "/" << v.y << "/" << v.z << ")";
    return dbg;
}

QString t() { return QTime::currentTime().toString("HH:mm:ss:zzz"); }
