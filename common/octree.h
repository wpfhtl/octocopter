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
    friend class Node;  // So that Node can access mPointHandler

private:
    unsigned int mMaxItemsPerLeaf;

    unsigned int mNumberOfItems;
    unsigned int mNumberOfNodes;

    // Pointers to the two last inserted LidarPoints. To be used for linear reduction.
    LidarPoint *mMri1, *mMri2;

    Node* mRootNode;

    // Its very likely that a point will be inserted into the same nod as its predecessor.
    // Thus, it makes sense to cache the node that received the last point.
    Node* mLastInsertionNode;

    // A function-pointer. Will be called to handle (=visualize?) points. When Octree::handlePoints()
    // is called, this function will be called for every point stored.
    void (*mPointHandler)(const QVector3D&);

public:
    Octree(const QVector3D &min, const QVector3D &max, const unsigned int maxItemsPerLeaf);


    // No two points closer than @distance will be inserted into this octree
    float mMinimumPointDistance;
    void setMinimumPointDistance(const float &distance);
    void setPointHandler(void (*pointHandler)(const QVector3D&));

    unsigned int getNumberOfItems(void) const;
    unsigned int getNumberOfNodes(void) const;

//    QList<Node*> findOverlappingLeafs(const QVector3D &point, const double radius);

    // Returns the N nearest neighbors of a given point in space. Result is not guaranteed to
    // be sorted by distance to @point.
    QList<LidarPoint*> findNearestNeighbors(const QVector3D &point, const unsigned int count) const;

    // Returns pointers to all LidarPoints in @radius of @point.
    QList<LidarPoint*> findNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Returns only the number of points in @radius of @point. More efficient than the methods above
    // if you're not interested in the points themselves
    uint32_t numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Yet more efficient: check whether there is AT LEAST ONE point within @radius of@point
    bool isNeighborWithinRadius(const QVector3D &point, const double radius) const;

    void sortPointList(const QVector3D &point, QList<LidarPoint*>* list) const;

    Node* insertPoint(LidarPoint* const point);
    Node* root();
    const Node* root() const;

    void handlePoints() const;
};

#endif
