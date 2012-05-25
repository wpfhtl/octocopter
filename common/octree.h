#ifndef OCTREE_H
#define OCTREE_H

#include <QVector>
#include <QVector3D>
#include <QList>
#include <QColor>

#include "node.h"
//#include "lidarpoint.h"

class LidarPoint;
class Node;

class Octree : public QObject
{
    Q_OBJECT

    friend class Node;  // So that Node can access mPointHandler and mInsertionHandler

private:
    quint32 mMaxItemsPerLeaf;

//    quint32 mNumberOfItems;
    quint32 mNumberOfNodes;

    // Indices/Offsets to the two last inserted LidarPoints. To be used for linear reduction.
    quint32 mMri1, mMri2;

    Node* mRootNode;

    // Leafs store indexes pointing into this central data store.
//    LidarPoint* mData;
    QVector<LidarPoint>* mData;

    // Its very likely that a point will be inserted into the same node as its predecessor.
    // Thus, it makes sense to cache the node that received the last point.
    Node* mLastInsertionNode;

    // This method is called by nodes when they inserted a point. It will simply emit pointInserted(),
    // so that someone else can just connect to the tree instead of connecting with every node.
    void pointInsertedByNode(const LidarPoint*);

public:
    Octree(const QVector3D &min, const QVector3D &max, const quint32 maxItemsPerLeaf, const quint32 expectedMaximumElementCount = 100 * 1000);
    ~Octree();

    // The octree is unlikely to grow bigger than this many elements.
    quint32 mExpectedMaximumElementCount;

    // Octree-Points are rendered using OpenGL 4 core profile. This means every octree saves
    // its points into a continuous memory region which can be mapped into opengl-memspace
    // to append points. Deletion of single points is currently not implemented (what to do
    // with memory of deleted points? Do memory management and re-set it later?)
    //
    // In order to render an octree's points, the GlWidget needs to know this octree's VBO id(s),
    // and the amount of points stored in each VBO (so when points are added, VBO can be updated)
    // We could use GLuint, but that would induce a GL dependency, so we use plain uints, same
    // thing anyway.
    // Mapping from multiple VBO-ids to their currently used ELEMENTS stored
    QMap<quint32, quint32> mVboIdsAndSizes;
    // Remember the number of elements stored in all used VBOs
    quint32 mElementsStoredInAllVbos;

    QColor mPointColor;

    // Give others access to our data
    const QVector<LidarPoint>* data() const {return mData;}

    // No two points closer than @distance will be inserted into this octree
    float mMinimumPointDistance;
    void setMinimumPointDistance(const float &distance);

    unsigned int getNumberOfItems(void) const;
    unsigned int getNumberOfNodes(void) const;

//    QList<Node*> findOverlappingLeafs(const QVector3D &point, const double radius);

    // Returns the N nearest neighbors of a given point in space. Result is not guaranteed to
    // be sorted by distance to @point.
    QList<const LidarPoint*> findNearestNeighbors(const QVector3D &point, const unsigned int count) const;

    // Returns pointers to all LidarPoints in @radius of @point.
    QList<const LidarPoint*> findNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Returns only the number of points in @radius of @point. More efficient than the methods above
    // if you're not interested in the points themselves
    quint32 numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Yet more efficient: check whether there is AT LEAST ONE point within @radius of@point
    bool isNeighborWithinRadius(const QVector3D &point, const double radius) const;

    // Sort @list oof points according to distance to @point
    void sortPointList(const QVector3D &point, QList<const LidarPoint*>* list) const;

    Node* insertPoint(LidarPoint* const point);
    Node* root();
    const Node* root() const;

public slots:
    void slotReset();

signals:
    void pointInserted(const LidarPoint*);
};

#endif
