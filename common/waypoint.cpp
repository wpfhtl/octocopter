#include "waypoint.h"

WayPoint::WayPoint() : QVector3D()
{
    purpose = Purpose::SCAN;
    informationGain = 0.0f;
}

WayPoint::WayPoint(const WayPoint& other) : QVector3D()
{
    purpose = other.purpose;
    informationGain = other.informationGain;

    setX(other.x());
    setY(other.y());
    setZ(other.z());
}

WayPoint::WayPoint(const QVector3D& vector, const float informationGain, Purpose purpose) : QVector3D()
{
    this->purpose = purpose;
    this->informationGain = informationGain;
    setX(vector.x());
    setY(vector.y());
    setZ(vector.z());
}

WayPoint& WayPoint::operator=(const WayPoint& other)
{
    purpose = other.purpose;
    informationGain = other.informationGain;
    setX(other.x());
    setY(other.y());
    setZ(other.z());

    return *this;
}

QVector2D WayPoint::getPositionOnPlane() const
{
    return QVector2D(x(), z());
}

QString WayPoint::hash(const QList<WayPoint>* const list)
{
    QCryptographicHash hash(QCryptographicHash::Md4);
    foreach(const WayPoint v, *list)
    {
        hash.addData(QByteArray::number(v.x()));
        hash.addData(QByteArray::number(v.y()));
        hash.addData(QByteArray::number(v.z()));
        hash.addData(QByteArray::number(v.informationGain));
    }

    return hash.result();
}

QString WayPoint::toString() const
{
    return QString("wpt gain %1: %2 %3 %4").arg(informationGain, 1).arg(x(), 7).arg(y(), 7).arg(z(), 7);
}


QDataStream& operator<<(QDataStream &out, const WayPoint &wpt)
{
    out << wpt.x();
    out << wpt.y();
    out << wpt.z();
    out << wpt.informationGain;
    out << static_cast<quint8>(wpt.purpose);
    return out;
}

QDataStream& operator>>(QDataStream &in, WayPoint &wpt)
{
    float x,y,z;
    quint8 purpose;
    in >> x;
    in >> y;
    in >> z;
    in >> wpt.informationGain;
    in >> purpose;

    wpt.setX(x);
    wpt.setY(y);
    wpt.setZ(z);

    wpt.purpose = static_cast<WayPoint::Purpose>(purpose);

    return in;
}
