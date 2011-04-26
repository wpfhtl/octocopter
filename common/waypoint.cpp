#include "waypoint.h"

WayPoint::WayPoint() : QVector3D()
{
}

WayPoint::WayPoint(const WayPoint& other)
{
    setX(other.x());
    setY(other.y());
    setZ(other.z());
}

WayPoint::WayPoint(const QVector3D& vector)
{
    setX(vector.x());
    setY(vector.y());
    setZ(vector.z());
}

WayPoint& WayPoint::operator=(const WayPoint& other)
{
    setX(other.x());
    setY(other.y());
    setZ(other.z());

    return *this;
}
