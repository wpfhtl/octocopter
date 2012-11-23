#ifndef POINTCLOUDOCTREENODE_H
#define POINTCLOUDOCTREENODE_H

#include <QVector3D>
//#include <QGLWidget>
#include "lidarpoint.h"


#include <stdint.h> // for uint32_t

class PointCloudOctree;

class PointCloudOctreeNode
{
public:

    PointCloudOctree* mTree;

    // For leaf-nodes, a list of its data. Must be 0 for non-leaf-nodes
    //QList<LidarPoint*> data;

    // Instead of storing real points in the nodes, the nodes store
    // point-offsets into an octree-global data-buffer.
    QVector<quint32> pointIndices;

    // For non-leaf-nodes, a List of octants/children.
    QList<PointCloudOctreeNode*> children;

    // A Pointer to its parent, or 0 for the root-node.
    PointCloudOctreeNode* parent;

//    LidarPoint* getLidarPointFromIndex(const quint32 index);

    // This node's AABB
    QVector3D min, max;

    PointCloudOctreeNode(PointCloudOctree* tree, PointCloudOctreeNode* parent, const QVector3D &min, const QVector3D &max);
    ~PointCloudOctreeNode();

    PointCloudOctreeNode& operator=(const PointCloudOctreeNode &other);
    bool operator==(const PointCloudOctreeNode &other);

    QList<PointCloudOctreeNode*> getAllChildLeafs(void);
    const QList<const PointCloudOctreeNode*> getAllChildLeafs(void) const;

    bool overlapsSphere(const QVector3D &point, const double radius) const;
    bool includesPoint(const QVector3D &point) const;
    //bool includesData(const LidarPoint &lidarPoint);

    // On successful insertion, returns a pointer to the node which swallowed the point.
    // If the point is discarded, 0 is returned instead.
    // Warning: You pass ownership to this Node, and the point might get deleted if its found to be of low value.
    PointCloudOctreeNode* insertPoint(LidarPoint* lidarPoint);
    bool isLeaf(void) const;

    // If this method decides that this point is not worth its memory, it will delete the point and set @lidarPoint to 0
    bool insertAndReduce(LidarPoint* lidarPoint);

    // delete the point at this memory address
    //bool deletePoint(LidarPoint* const lidarPoint);

    // delete the point at this position
    bool deletePoint(const LidarPoint &lidarPoint);

    // delete all subnodes and all points
    void clearPoints();

    // Returns this Node's size
    QVector3D size(void) const;

    // Returns the @count nearest neighbors FROM THIS LEAF-NODE ONLY of the given point, sorted by ascending distance
    QList<const LidarPoint*> findNearestNeighbors(const QVector3D &point, const unsigned int count) const;

    // Returns all neighbors FROM THIS LEAF-NODE ONLY within @radius of the given @point
    QList<const LidarPoint*> findNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Returns neighbors FROM THIS LEAF-NODE ONLY within @radius
    quint32 numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Returns true when a neighbor is within @radius of POINT. Checks THIS LEAF-NODE ONLY
    bool neighborsWithinRadius(const QVector3D &point, const float radius) const;

    // Returns false if the given sphere leaks from this Node. If we're looking for neighbors
    // of the sphere's center, and the furthest neighbor was found on the sphere's surface,
    // a leaking sphere would mean that other nodes could contain closer neighbors.
    bool isSphereContained(const QVector3D point, const double radius);

    bool isBoxContained(const QVector3D &min, const QVector3D &max);

    // Creates a sub-partitioning for this octree-node.
    void partition();

    // Returns the correct octant for a position within this Node, 0 else.
    PointCloudOctreeNode* getLeaf(const QVector3D &point);

    // Returns this Node's center
    QVector3D center(void) const;
};

#endif
