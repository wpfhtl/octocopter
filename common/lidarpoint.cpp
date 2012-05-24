#include "lidarpoint.h"

LidarPoint::LidarPoint() : position(), laserPos()
{

}

LidarPoint::LidarPoint(const QVector3D &position, const QVector3D &laserPos) : position(position), laserPos(laserPos)
{
        node = 0;
}

LidarPoint::LidarPoint(const LidarPoint &other)
{
    position = other.position;
    laserPos = other.laserPos;
    node = other.node;
}

LidarPoint& LidarPoint::operator=(const LidarPoint &other)
{
    position = other.position;
    laserPos = other.laserPos;
    node = other.node;

    return *this;
}

bool LidarPoint::operator==(const LidarPoint &other) const
{
    // We ignore node* on purpose for now
    return
            this->position == other.position &&
            this->laserPos == other.laserPos;
}

QDebug operator<<(QDebug dbg, const LidarPoint &lidarPoint)
{
    dbg.nospace() << "LidarPoint: PointAt:" << lidarPoint.position << "LaserAt:" << lidarPoint.laserPos;
    return dbg.maybeSpace();
}

QDataStream& operator<<(QDataStream &out, const LidarPoint &lidarPoint)
{
    out << lidarPoint.position << lidarPoint.laserPos;
    return out;
}

QDataStream& operator>>(QDataStream &in, LidarPoint &lidarPoint)
{
    in >> lidarPoint.position;
    in >> lidarPoint.laserPos;
    return in;
}
