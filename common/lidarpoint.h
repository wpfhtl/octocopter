#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include <QVector3D>
#include <QDebug>

#ifdef LIDARPOINT_KEEPS_PARENTNODE
#include "node.h"
class Node;
#endif


class LidarPoint : public QObject
{
    Q_OBJECT

public:
    LidarPoint(const QVector3D &position, const QVector3D &direction, const float &squaredDistance);
    LidarPoint(const LidarPoint &other);

    LidarPoint& operator=(const LidarPoint &other);
    bool operator==(const LidarPoint &other) const;

#ifdef LIDARPOINT_KEEPS_PARENTNODE
    // The node that we live in. This pointer is set by the node when it swallows us.
    Node* node;
#endif

    // Position is position of detected object, direction is from
    // that object back to the laserscanner. The direction of the
    // ray might come in handy when we need to find better next-best-
    // shot poses.
    QVector3D position, direction;
//    Ray ray;

    // This tells us the distance between LaserScanner and detected
    // object. Probably helpful because the further an object is away,
    // the worse its position (not distance) accuracy will be due to
    // the platforms orientation-misalignment.
    float squaredDistance;

    // Distances to other QVector3Ds
    inline double distanceTo(const QVector3D &other) const { return (position-other).length(); }
    inline double squaredDistanceTo(const QVector3D &other) const { return (position-other).lengthSquared(); }

    // Distances to other LidarPoints
    inline double distanceTo(const LidarPoint &other) const { return (position-other.position).length(); }
    inline double squaredDistanceTo(const LidarPoint &other) const { return (position-other.position).lengthSquared(); }
};

// for using qDebug() << ldp;
QDebug operator<<(QDebug dbg, const LidarPoint &lidarPoint);

// for streaming
QDataStream& operator<<(QDataStream &out, const LidarPoint &lidarPoint);
QDataStream& operator>>(QDataStream &in, LidarPoint &lidarPoint);

#endif
