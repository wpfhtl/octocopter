#include "common.h"

QString hash(QList<WayPoint> list)
{
    QCryptographicHash hash(QCryptographicHash::Md4);
    foreach(const WayPoint v, list)
    {
        hash.addData(QByteArray::number(v.x()));
        hash.addData(QByteArray::number(v.y()));
        hash.addData(QByteArray::number(v.z()));
    }

    return hash.result();
}
