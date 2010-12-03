#include "common.h"

QString hash(QList<QVector3D> list)
{
    QCryptographicHash hash(QCryptographicHash::Md4);
    foreach(const QVector3D v, list)
    {
        hash.addData(QByteArray::number(v.x()));
        hash.addData(QByteArray::number(v.y()));
        hash.addData(QByteArray::number(v.z()));
    }

    return hash.result();
}

