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

    Purpose purpose;
    quint8 informationGain;

    WayPoint();
    WayPoint(const WayPoint& other);
    WayPoint(const QVector3D& vector, const quint8 informationGain = 0, Purpose = Purpose::SCAN);
    WayPoint(const QString& string);

    QString toString() const;

    WayPoint &operator=(const WayPoint& other);

    static QString hash(const QList<WayPoint> *const list);
    QVector2D getPositionOnPlane() const;
};

#endif
