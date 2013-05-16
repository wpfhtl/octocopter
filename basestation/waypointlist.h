#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <QMap>
#include <QList>
#include <QColor>
#include <QVector>
#include <QOpenGLFunctions_4_3_Core>

#include "waypoint.h"

class WayPointList : public QOpenGLFunctions_4_3_Core
{
private:
    QList<WayPoint> mWaypoints;
    quint32 mVbo;
    QColor mColor;
    float mSphereSize;

    void setVbo();

public:
    WayPointList();
    WayPointList(const QColor& color);
    ~WayPointList();

    void mergeCloseWaypoints(const float minimumDistance);

    float getSphereSize() const { return mSphereSize; }

    QList<WayPoint>* list() {return &mWaypoints;}
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
