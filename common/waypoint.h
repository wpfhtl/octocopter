#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <QVector2D>
#include <QVector3D>
#include <QStringList>
#include <QCryptographicHash>

// For now, this class is just a QVector3D, but it might
// be extended to include orientation, timestamps etc.

class WayPoint : public QVector3D
{
//    Q_OBJECT
public:
    enum class Purpose {SCAN, DETOUR};
    float informationGain;

    WayPoint();
    WayPoint(const WayPoint& other);
    WayPoint(const QVector3D& vector, const float informationGain = 0);

    QString toString() const;

    WayPoint &operator=(const WayPoint& other);

    static QString hash(const QList<WayPoint> *const list);
    QVector2D getPositionOnPlane() const;
};


// for streaming
QDataStream& operator<<(QDataStream &out, const WayPoint &wpt);
QDataStream& operator>>(QDataStream &in, WayPoint &wpt);

#endif
