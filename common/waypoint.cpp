#include "waypoint.h"

WayPoint::WayPoint() : QVector3D()
{
    purpose = SCAN;
}

WayPoint::WayPoint(const WayPoint& other)
{
    purpose = other.purpose;

    setX(other.x());
    setY(other.y());
    setZ(other.z());
}

WayPoint::WayPoint(const QVector3D& vector, Purpose purpose)
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
