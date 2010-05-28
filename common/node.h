#ifndef NODE_H
#define NODE_H

#include <QVector3D>
#include <QGLWidget>
#include "lidarpoint.h"
#include "octree.h"

class Octree;

class Node
{
public:

    static unsigned int mMaxItemsPerLeaf;
    static unsigned int mNumberOfItems;
    static unsigned int mNumberOfNodes;
    static Octree* mTree;

    // For leaf-nodes, a list of its data. Must be 0 for non-leaf-nodes
    QList<LidarPoint*> data;

    // For non-leaf-nodes, a List of octants/children.
    QList<Node*> children;

    // A Pointer to its parent, or 0 for the root-node.
    Node* parent;

    // This node's AABB
    QVector3D min, max;

    // Cached values.
//    QVector3D center;
//    double radius;

    void drawGl(void) const;

    Node(Node* parent, const QVector3D &min, const QVector3D &max);
    ~Node();

    Node& operator=(const Node &other);
    bool operator==(const Node &other);

    QList<Node*> getAllChildLeafs(void);

    bool overlapsSphere(const QVector3D &point, const double radius) const;
    bool includesPoint(const QVector3D &point) const;
    bool includesData(const LidarPoint &lidarPoint);
    Node* insertPoint(LidarPoint* const lidarPoint);
    bool isLeaf(void) const;

    // Returns this Node's size
    QVector3D size(void) const;

    QList<LidarPoint*> findNearestNeighbors(const QVector3D &point, const unsigned int count) const;
    QList<LidarPoint*> findNeighborsWithinRadius(const QVector3D &point, const double radius) const;

    // Returns false if the given sphere leaks from this Node. If we're looking for neighbors
    // of the sphere's center, and the furthest neighbor was found on the sphere's surface,
    // a leaking sphere would mean that other nodes could contain closer neighbors.
    bool isSphereContained(const QVector3D point, const double radius);

    bool isBoxContained(const QVector3D &min, const QVector3D &max);

    // Creates a sub-partitioning for this octree-node.
    void partition();

    // Returns the correct octant for a position within this Node, 0 else.
    Node* getLeaf(const QVector3D &point);

/*
    // Returns this Node's octants' extremes, in the same order as the octants
    QList<QVector3D> corners(void) const;

    // Returns this Nodes' planes as point/normal pairs
    QList<QVector3D> planes(void) const;
*/
    // Returns this Node's center
    QVector3D center(void) const;



};

#endif
