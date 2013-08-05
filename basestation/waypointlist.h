#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <QMap>
#include <QList>
#include <QColor>
#include <QVector>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include "waypoint.h"

class WayPointList : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
private:
    QList<WayPoint> mWaypoints;
    quint32 mVbo;
    QColor mColor;
//    float mSphereSize;

    void updateVbo();

public:
    WayPointList();
    WayPointList(const QColor& color);
    WayPointList(const WayPointList& other);
    ~WayPointList();

    QString toString() const;

    bool isEmpty() const
    {
        return mWaypoints.isEmpty();
    }

    void mergeCloseWaypoints(const float minimumDistance);

    void setList(const QList<WayPoint>* const wayPointList);

//    float getSphereSize() const { return mSphereSize; }

    const QList<WayPoint>* list() const {return &mWaypoints;}
    const quint32 vbo() const {return mVbo;}
    const QColor color() const {return mColor;}

    void prepend(const WayPoint& wp);
    void append(const WayPoint& wp);
    void append(const QList<WayPoint>& wps);
    void append(const WayPointList* wpl);
    void insert(const quint16& index, const WayPoint& wp);
    void remove(const quint16& index);
    void removeLast() {mWaypoints.removeLast();}
    WayPoint takeAt(const int index);
    int size() const {return mWaypoints.size();}
    void clear();
    void swap(const int& i, const int& j) {mWaypoints.swap(i,j);} // no need for setVbo(), rendering order doesn't matter

    void sortToShortestPath(const QVector3D &vehiclePosition);

    void setWayPoint(const int index, const WayPoint& wpt)
    {
        mWaypoints[index] = wpt;
        updateVbo();
    }

    const WayPoint& at(const int index) const {return mWaypoints.at(index);}
    const WayPoint& first() const {return mWaypoints.first();}

    bool saveToFile(const QString& fileName) const;
    bool loadFromFile(const QString& fileName);
};

#endif // WAYPOINTLIST_H
