#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include <QVector3D>
#include <QDebug>

class Node;

class LidarPoint// : public QObject
{
    //Q_OBJECT

public:
    LidarPoint();
    LidarPoint(const QVector3D &position, const QVector3D &posLaser);
    LidarPoint(const LidarPoint &other);

    LidarPoint& operator=(const LidarPoint &other);
    bool operator==(const LidarPoint &other) const;

    // The node that we live in. This pointer is set by the node when it swallows us.
    //Node* node;

    QVector3D position, laserPos;

    // Distances to other QVector3Ds
    inline float distanceTo(const QVector3D &other) const { return (position-other).length(); }
    inline float squaredDistanceTo(const QVector3D &other) const { return (position-other).lengthSquared(); }

    // Distances to other LidarPoints
    inline float distanceTo(const LidarPoint &other) const { return (position-other.position).length(); }
    inline float squaredDistanceTo(const LidarPoint &other) const { return (position-other.position).lengthSquared(); }
};

// for using qDebug() << ldp;
QDebug operator<<(QDebug dbg, const LidarPoint &lidarPoint);

// for streaming
QDataStream& operator<<(QDataStream &out, const LidarPoint &lidarPoint);
QDataStream& operator>>(QDataStream &in, LidarPoint &lidarPoint);

#endif
