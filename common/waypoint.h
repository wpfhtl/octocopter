#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <QVector3D>

// For now, this class is just a QVector3D, but it might
// be extended to include orientation, timestamps etc.

class WayPoint : public QVector3D
{
    Q_OBJECT
public:
    WayPoint();

signals:

public slots:

};

#endif
