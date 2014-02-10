#ifndef BOX3D_H
#define BOX3D_H

#include <QPoint>
#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <common.h>

struct Box3D
{
    QVector3D min, max;

    // Required for shifting a smaller box in a larger box.
    Box3D *mBoundingBox;
    QVector<bool>* mVisitorGrid;
//    Vector3i mTravelDirection;

    Box3D();
    Box3D(const QVector3D& minBox, const QVector3D& maxBox);

    ~Box3D();

    QVector3D size() const;
    QVector3D center() const;
    bool containsAllOf(const Box3D* other) const;
    bool contains(const QVector3D* point) const;
    Box3D tryToKeepWithin(const Box3D& other) const;

    void setCenter(const QVector3D& center);

    Box3D moveToNextPosition(
            const QVector3D& vehiclePosition,
            const Box3D& boundingBox);
};

// for using qDebug()
QDebug operator<<(QDebug dbg, const Box3D &box);

#endif // BOX3D_H
