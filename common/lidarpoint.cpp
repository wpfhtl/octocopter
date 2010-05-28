#include "lidarpoint.h"

LidarPoint::LidarPoint(const QVector3D &position, const QVector3D &direction, const float &squaredDistance) :
        QObject(),
        position(position),
        direction(direction),
        squaredDistance(squaredDistance)
{

}

LidarPoint::LidarPoint(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    squaredDistance = other.squaredDistance;
}

LidarPoint& LidarPoint::operator=(const LidarPoint &other)
{
    position = other.position;
    direction = other.direction;
    squaredDistance = other.squaredDistance;

    return *this;
}

bool LidarPoint::operator==(const LidarPoint &other) const
{
    return
            this->position == other.position &&
            this->direction == other.direction &&
            this->squaredDistance == other.squaredDistance;
}
