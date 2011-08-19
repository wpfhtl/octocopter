#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <QVector2D>
#include <QVector3D>

// For now, this class is just a QVector3D, but it might
// be extended to include orientation, timestamps etc.

class WayPoint : /*public QObject, */public QVector3D
{
//    Q_OBJECT
public:
    enum Purpose {SCAN, DETOUR};

    Purpose purpose;

    WayPoint();
    WayPoint(const WayPoint& other);
    WayPoint(const QVector3D& vector, Purpose = SCAN);

    WayPoint &operator=(const WayPoint& other);

    QVector2D getPositionOnPlane() const;

//signals:

//public slots:

};

#endif
