#include "lidarpoint.h"

LidarPoint::LidarPoint(const QVector3D &position, const QVector3D &direction, const float &squaredDistance) :
        QObject(),
        position(position),
        direction(direction),
        squaredDistance(squaredDistance)
{
#ifdef LIDARPOINT_KEEPS_PARENTNODE
        node = 0;
#endif
}

LidarPoint::LidarPoint(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    squaredDistance = other.squaredDistance;

#ifdef LIDARPOINT_KEEPS_PARENTNODE
    node = other.node;
#endif
}

LidarPoint& LidarPoint::operator=(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    squaredDistance = other.squaredDistance;

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
            this->squaredDistance == other.squaredDistance;
}


QDebug operator<<(QDebug dbg, const LidarPoint &lidarPoint)
{
    dbg.nospace() << "LidarPoint: Position:" << lidarPoint.position << "Direction:" << lidarPoint.direction << "squaredDistance:" << lidarPoint.squaredDistance;
    return dbg.maybeSpace();
}

QDataStream& operator<<(QDataStream &out, const LidarPoint &lidarPoint)
{
    out << lidarPoint.position << lidarPoint.direction << lidarPoint.squaredDistance;
    return out;
}

QDataStream& operator>>(QDataStream &in, LidarPoint &lidarPoint)
{
    in >> lidarPoint.position;
    in >> lidarPoint.direction;
    in >> lidarPoint.squaredDistance;
    return in;
}
