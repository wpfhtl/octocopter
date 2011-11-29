#include "waypoint.h"

WayPoint::WayPoint() : QVector3D()
{
    purpose = SCAN;
}

WayPoint::WayPoint(const WayPoint& other) : QVector3D()
{
    purpose = other.purpose;

    setX(other.x());
    setY(other.y());
    setZ(other.z());
}

WayPoint::WayPoint(const QVector3D& vector, Purpose purpose) : QVector3D()
{
    this->purpose = purpose;
    setX(vector.x());
    setY(vector.y());
    setZ(vector.z());
}

WayPoint& WayPoint::operator=(const WayPoint& other)
{
    purpose = other.purpose;
    setX(other.x());
    setY(other.y());
    setZ(other.z());

    return *this;
}

QVector2D WayPoint::getPositionOnPlane() const
{
    return QVector2D(x(), z());
}

QString WayPoint::hash(QList<WayPoint> list)
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
