#include "octree.h"

Octree::Octree(const QVector3D &min, const QVector3D &max, const unsigned int maxItemsPerLeaf) :
        mMaxItemsPerLeaf(maxItemsPerLeaf)
{
    mRootNode = new Node(0, min, max);
    mRootNode->mTree = this;
}

Node* Octree::insertPoint(LidarPoint* const point)
{
    while(!mRootNode->includesPoint(point->position))
    {
        // We're supposed to save a point thats outside the bounds of our tree.
        // Create a supernode for the rootnode, then reparent.
        int positionOfOldRootInNewRoot;
        QVector3D newMin, newMax;

        // Its a cube, so all edges have same length.
        const double oldRootNodeEdgeLength = mRootNode->size().x();
        const QVector3D oldRootNodeCenter = mRootNode->center();

        // Where outside the rootnode is the point?
        if(point->position.x() >= oldRootNodeCenter.x())
        {
            // The old rootnode will be octant 0,3,4,7
            if(point->position.y() >= oldRootNodeCenter.y())
            {
                // The old rootnode will be octant 0,3
                if(point->position.z() >= oldRootNodeCenter.z())
                {
                    // The old rootnode will be octant 0
                    positionOfOldRootInNewRoot = 0;
                    newMin = mRootNode->min;

                    newMax = QVector3D(
                            mRootNode->max.x() + oldRootNodeEdgeLength,
                            mRootNode->max.y() + oldRootNodeEdgeLength,
                            mRootNode->max.z() + oldRootNodeEdgeLength
                            );
                }
                else
                {
                    // The old rootnode will be octant 3
                    positionOfOldRootInNewRoot = 3;
                    newMin = QVector3D(
                            mRootNode->min.x(),
                            mRootNode->min.y(),
                            mRootNode->min.z() - oldRootNodeEdgeLength
                            );

                    newMax = QVector3D(
                            mRootNode->max.x() + oldRootNodeEdgeLength,
                            mRootNode->max.y() + oldRootNodeEdgeLength,
                            mRootNode->max.z()
                            );
                }
            }
            else
            {
                // The old rootnode will be octant 4,7
                if(point->position.z() >= oldRootNodeCenter.z())
                {
                    // The old rootnode will be octant 4
                    positionOfOldRootInNewRoot = 4;
                    newMin = QVector3D(
                            mRootNode->min.x(),
                            mRootNode->min.y() - oldRootNodeEdgeLength,
                            mRootNode->min.z()
                            );

                    newMax = QVector3D(
                            mRootNode->max.x() + oldRootNodeEdgeLength,
                            mRootNode->max.y(),
                            mRootNode->max.z() + oldRootNodeEdgeLength
                            );
                }
                else
                {
                    // The old rootnode will be octant 7
                    positionOfOldRootInNewRoot = 7;
                    newMin = QVector3D(
                            mRootNode->min.x(),
                            mRootNode->min.y() - oldRootNodeEdgeLength,
                            mRootNode->min.z() - oldRootNodeEdgeLength
                            );

                    newMax = QVector3D(
                            mRootNode->max.x() + oldRootNodeEdgeLength,
                            mRootNode->max.y(),
                            mRootNode->max.z()
                            );
                }
            }
        }
        else
        {
            // The old rootnode will be octant 1,2,5,6
            if(point->position.y() >= oldRootNodeCenter.y())
            {
                // The old rootnode will be octant 1,2
                if(point->position.z() >= oldRootNodeCenter.z())
                {
                    //TODO: i swapped 1/2 to make it right(?). Think of and debug the other combinations
                    // The old rootnode will be octant 2
                    positionOfOldRootInNewRoot = 2;
                    newMin = QVector3D(
                            mRootNode->min.x() - oldRootNodeEdgeLength,
                            mRootNode->min.y(),
                            mRootNode->min.z() - oldRootNodeEdgeLength
                            );

                    newMax = QVector3D(
                            mRootNode->max.x(),
                            mRootNode->max.y() + oldRootNodeEdgeLength,
                            mRootNode->max.z()
                            );
                }
                else
                {
                    // The old rootnode will be octant 1
                    positionOfOldRootInNewRoot = 1;
                    newMin = QVector3D(
                            mRootNode->min.x() - oldRootNodeEdgeLength,
                            mRootNode->min.y(),
                            mRootNode->min.z()
                            );

                    newMax = QVector3D(
                            mRootNode->max.x(),
                            mRootNode->max.y() + oldRootNodeEdgeLength,
                            mRootNode->max.z() + oldRootNodeEdgeLength
                            );
                }
            }
            else
            {
                // The old rootnode will be octant 5,6
                if(point->position.z() >= oldRootNodeCenter.z())
                {
                    // The old rootnode will be octant 5
                    positionOfOldRootInNewRoot = 5;
                    newMin = QVector3D(
                            mRootNode->min.x() - oldRootNodeEdgeLength,
                            mRootNode->min.y() - oldRootNodeEdgeLength,
                            mRootNode->min.z()
                            );

                    newMax = QVector3D(
                            mRootNode->max.x(),
                            mRootNode->max.y(),
                            mRootNode->max.z() + oldRootNodeEdgeLength
                            );
                }
                else
                {
                    // The old rootnode will be octant 6
                    positionOfOldRootInNewRoot = 6;
                    newMin = QVector3D(
                            mRootNode->min.x() - oldRootNodeEdgeLength,
                            mRootNode->min.y() - oldRootNodeEdgeLength,
                            mRootNode->min.z() - oldRootNodeEdgeLength
                            );

                    newMax = mRootNode->max;
                }
            }
        }

        // Now we have a new root-node. Insert it and reparent the old one.
        qDebug() << "Octree::insertPoint(): mRootNode goes from" << mRootNode->min << "to" << mRootNode->max << "supernode swallows us at position" << positionOfOldRootInNewRoot;
        Node* const oldRootNode = mRootNode;
        mRootNode = new Node(0, newMin, newMax);
        mRootNode->partition();
        delete mRootNode->children.at(positionOfOldRootInNewRoot);
        mRootNode->children[positionOfOldRootInNewRoot] = oldRootNode;
        oldRootNode->parent = mRootNode;
    }
    return mRootNode->insertPoint(point);
}

Node* Octree::root()
{
    return mRootNode;
}

void Octree::drawGl(void) const
{
    // set transparent material?

    mRootNode->drawGl();
}

Node* Octree::getNodeForPoint(const QVector3D &point)
{
}

/* UNUSED
// Returns a list of leaf-nodes that overlap the given sphere.
QList<Node*> Octree::findOverlappingLeafs(const QVector3D &point, const double radius)
{
    // We use a top-down approach
    Node* currentNode;

    // Resultlist and list of Nodes to check
    QList<Node*> nodeList, nodeCandidates;

    // Initialize with the root-node.
    nodeCandidates << mRootNode;

    while(!nodeCandidates.isEmpty())
    {
        // Take a Node from the candidates to check
        currentNode = nodeCandidates.takeFirst();

        if(currentNode->isLeaf())
        {
            if(currentNode->overlapsSphere(point, radius))
            {
                nodeList << currentNode;
            }
        }
        else
        {
            // Check all this node's octants for overlapping
            foreach(const Node* n, (*currentNode)->children)
            {
                if(n->overlapsSphere(point, radius))
                    nodeCandidates << n;
            }
        }
    }

    return nodeList;
}
*/

void Octree::sortPointList(const QVector3D &point, QList<LidarPoint*>* list) const
{
    QMap<double, LidarPoint*> distanceMap;
    for(int i=0;i<list->size();i++)
        distanceMap.insert(list->at(i)->squaredDistanceTo(point), list->at(i));

    // Does NOT delete its data, as QList does not take ownership (of pointers).
    // Use qDeleteAll() if you also want to delete() the pointers.
    list->clear();

    QMap<double, LidarPoint*>::const_iterator i = distanceMap.constBegin();
    while(i != distanceMap.constEnd())
    {
        list->append(i.value());
        ++i;
    }
}

// Returns all neighbors of the given point within @radius
QList<LidarPoint*> Octree::findNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    QList<LidarPoint*> neighbors;
    Node* currentNode = mRootNode->getLeaf(point);

    // Go up the tree as long as our searchradius leaks from the node
    // If we reach the rootnode, use him.
    while(currentNode->parent && !currentNode->isSphereContained(point, radius))
        currentNode = currentNode->parent;

    foreach(Node* const n, currentNode->getAllChildLeafs())
    {
        if(n->overlapsSphere(point, radius))
            neighbors << n->findNeighborsWithinRadius(point, radius);
    }

    return neighbors;
}

bool Octree::foo(const QList<LidarPoint*> &list) const
{
//    qDebug() << "Now checking while condition, size" << list.size();
    return true;
}

// Returns AT LEAST the @count nearest neighbors of @point, sorted by ascending distance
QList<LidarPoint*> Octree::findNearestNeighbors(const QVector3D &point, const unsigned int count) const
{
    // Ask the leaf including this point for its neighbors
    Node* containingNode = mRootNode->getLeaf(point);
    QList<LidarPoint*> neighbors = containingNode->findNearestNeighbors(point, count);

    // sort the possibly empty list of neighbors
    sortPointList(point, &neighbors);

    // Keep a set of tested-leaf-ADDRESSES, so we don't test anyone twice.
    QSet<Node*> leafsTested;
    leafsTested << containingNode;

    // As long as we don't have enough neighbors OR there might be better neighbors in parentnodes, ...
    while(
            foo(neighbors)
            &&
            (
            neighbors.size() < count
            ||
            (
                    neighbors.size() >= count
                    &&
                    // ganze zeile falsch, iSC wahr
                    // there could be better neighbors in other nodes
                    !containingNode->isSphereContained(point, neighbors.at(count-1)->distanceTo(point))
            )
            )
    )
    {
        // ... we have to ask parent nodes for more neighbors.

        if(!containingNode->parent)
        {
            // We are at the root, there will be no more neighbors. I swear!
            return neighbors;
        }

        containingNode = containingNode->parent;

        QList<Node*> childLeafs = containingNode->getAllChildLeafs();
        for(int i=0;i<childLeafs.size();i++)
        {
            Node* ln = childLeafs.at(i);
            if(!leafsTested.contains(ln))
            {
                // Test this leafnode if
                // - it overlaps the sphere created by centerpoint @point and the furthest neighbor
                //  or
                // - if we haven't found any neighbors yet
                //  errata: that is, if we haven't found enough yet.

                qDebug() << "size of neighbors list:" << neighbors.size();
                //if(neighbors.empty() || ln->overlapsSphere(point, neighbors.at(count-1)->distanceTo(point)))
                if(neighbors.size() < count || ln->overlapsSphere(point, neighbors.at(count-1)->distanceTo(point)))
                {
                    leafsTested << ln;
                    neighbors << ln->findNearestNeighbors(point, count);
                    sortPointList(point, &neighbors);
                }
            }
        }

//        qDebug() << "Now checking whether we have enough points AND there cannot be any closer ones.";
        if(neighbors.size() >= count && containingNode->isSphereContained(point, neighbors.at(count-1)->distanceTo(point)))
        {
            // We have enough nodes and there cannot be closer neighbors in parent-nodes, so our search has ended.
            return neighbors;
        }
        else
        {
            if(neighbors.size() < count)
                qDebug() << "NOT returning, count is too small" << neighbors.size();

            if(!containingNode->isSphereContained(point, neighbors.at(count-1)->distanceTo(point)))
                qDebug() << "NOT returning, sphere around furthest neighbor is not contained";
        }
    }

//    qDebug() << "neighborsize:" << neighbors.size();

    // We should never end up here. If there's not enough neighbors, we advance to the top of the tree and
    // then return the most recent resultset. If we have enough neighbors, and there cannot be closer ones,
    // we return above.
//    Q_ASSERT(false);

    // The comment above is NOT true if the very first call to findNearestNeighbors() yields the required
    // neighbors. The fix is easy, just return the list.
    return neighbors;
}
