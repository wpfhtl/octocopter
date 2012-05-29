#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <QMap>
#include <QList>
#include <QColor>
#include <QVector>

#include "waypoint.h"

class WayPointList
{
private:
    QList<WayPoint> mWaypoints;
    quint32 mVbo;
    QColor mColor;

    void setVbo();

public:
    WayPointList();
    WayPointList(const QColor& color);
    ~WayPointList();

    float sphereSize;

    const QList<WayPoint>* list() const {return &mWaypoints;}
    const quint32 vbo() const {return mVbo;}
    const QColor color() const {return mColor;}

    void append(const WayPoint& wp);
    void append(const QList<WayPoint>& wps);
    void append(const WayPointList* wpl);
    void insert(const quint16& index, const WayPoint& wp);
    void remove(const quint16& index);
    WayPoint takeAt(const int index);
    int size() const {return mWaypoints.size();}
    void clear();
    void swap(const int& i, const int& j) {mWaypoints.swap(i,j);} // no need for setVbo(), rendering order doesn't matter

    void sortToShortestPath(const QVector3D &vehiclePosition);

    const WayPoint& at(const int index) const {return mWaypoints.at(index);}
    const WayPoint& first() const {return mWaypoints.first();}
};

#endif // WAYPOINTLIST_H
