#ifndef OCTREE_H
#define OCTREE_H

#include <QtCore>

#include <QVector3D>
#include <QGLWidget>
#include <QList>

#include "node.h"
#include "lidarpoint.h"

class LidarPoint;
class Node;

class Octree
{
private:
    unsigned int mMaxItemsPerLeaf;
    QVector3D mMin, mMax;
    Node* mRootNode;

public:
    Octree(const QVector3D &min, const QVector3D &max, const unsigned int maxItemsPerLeaf);

//    QList<Node*> findOverlappingLeafs(const QVector3D &point, const double radius);

    QList<LidarPoint*> findNearestNeighbors(const QVector3D &point, const unsigned int count) const;
    QList<LidarPoint*> findNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    void sortPointList(const QVector3D &point, QList<LidarPoint*>* list) const;

    Node* insertPoint(LidarPoint* const point);
    Node* root();

    void drawGl(void) const;

    Node* getNodeForPoint(const QVector3D &point);
};

#endif
