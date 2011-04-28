#include "lidarpoint.h"

LidarPoint::LidarPoint() : QObject(), position(), direction(), distance(-1)
{

}

LidarPoint::LidarPoint(const QVector3D &position, const QVector3D &direction, const float &distance) :
        QObject(),
        position(position),
        direction(direction),
        distance(distance)
{
//#ifdef LIDARPOINT_KEEPS_PARENTNODE
        node = 0;
//#endif
}

LidarPoint::LidarPoint(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    distance = other.distance;

#ifdef LIDARPOINT_KEEPS_PARENTNODE
    node = other.node;
#endif
}

LidarPoint& LidarPoint::operator=(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    distance = other.distance;

#ifdef LIDARPOINT_KEEPS_PARENTNODE
    node = other.node;
#endif

    return *this;
}

bool LidarPoint::operator==(const LidarPoint &other) const
{
    // We ignore node* on purpose for now
    return
            this->position == other.position &&
            this->direction == other.direction &&
            this->distance == other.distance;
}


QDebug operator<<(QDebug dbg, const LidarPoint &lidarPoint)
{
    dbg.nospace() << "LidarPoint: Position:" << lidarPoint.position << "Direction:" << lidarPoint.direction << "squaredDistance:" << lidarPoint.distance;
    return dbg.maybeSpace();
}

QDataStream& operator<<(QDataStream &out, const LidarPoint &lidarPoint)
{
    out << lidarPoint.position << lidarPoint.direction << lidarPoint.distance;
    return out;
}

QDataStream& operator>>(QDataStream &in, LidarPoint &lidarPoint)
{
    in >> lidarPoint.position;
    in >> lidarPoint.direction;
    in >> lidarPoint.distance;
    return in;
}
