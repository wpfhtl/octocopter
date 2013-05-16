#include "pointcloudoctree.h"

PointCloudOctree::PointCloudOctree(const QVector3D &min, const QVector3D &max, quint32 maxItemsPerLeaf, const quint32 expectedMaximumElementCount) :
    mMaxItemsPerLeaf(maxItemsPerLeaf),
    mExpectedMaximumElementCount(expectedMaximumElementCount),
    mElementsStoredInAllVbos(0),
    mNumberOfNodes(0),
    mLastInsertionNode(0),
    mMinimumPointDistance(-1.0f) // disable check by default
{
    mRootNode = new PointCloudOctreeNode(this, 0, min, max);

    mPointColor = QColor(255,255,255,64);

    // Try to allocate our data storage
    //mData = (LidarPoint*)malloc(sizeof(LidarPoint) * mExpectedMaximumElementCount);

    // Using a QVector means that all elements will be initialized with the default c'tor :|
    mData = new QVector<LidarPoint>;
    mData->reserve(mExpectedMaximumElementCount);
}

PointCloudOctree::~PointCloudOctree()
{
    delete mRootNode;

    mData->clear();
    delete mData;
}

void PointCloudOctree::setMinimumPointDistance(const float &distance)
{
    mMinimumPointDistance = distance;
}

void PointCloudOctree::slotReset()
{
    mRootNode->clearPoints();
    mNumberOfNodes = 0;
    mLastInsertionNode = 0;
}

PointCloudOctreeNode* PointCloudOctree::insertPoint(LidarPoint* const point)
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
        qDebug() << "PointCloudOctree::insertPoint(): mRootNode goes from" << mRootNode->min << "to" << mRootNode->max << "supernode swallows us at position" << positionOfOldRootInNewRoot;
        PointCloudOctreeNode* const oldRootNode = mRootNode;
        mRootNode = new PointCloudOctreeNode(this, 0, newMin, newMax);
        mRootNode->partition();
        delete mRootNode->children.at(positionOfOldRootInNewRoot);
        mRootNode->children[positionOfOldRootInNewRoot] = oldRootNode;
        oldRootNode->parent = mRootNode;
    }

    // Insert the LidarPoint into the right Node. First, try to insert into the mLastInsertion-Node. If
    // that doesn't work, go the old route of traversing down from the root-node.
    if(mLastInsertionNode == 0 || ! mLastInsertionNode->includesPoint(point->position))
    {
        // If the point does NOT get saved, 0 is returned. In that case, don't update the node-pointer.
        PointCloudOctreeNode* insertionNode = mRootNode->insertPoint(point);
        if(insertionNode != 0) mLastInsertionNode = insertionNode;
    }
    else
    {
//        qDebug() << "inserting point" << point << "into old node.";
        mLastInsertionNode->insertPoint(point);
    }



    return mLastInsertionNode;
}

PointCloudOctreeNode* PointCloudOctree::root()
{
    return mRootNode;
}

const PointCloudOctreeNode* PointCloudOctree::root() const
{
    return mRootNode;
}

void PointCloudOctree::sortPointList(const QVector3D &point, QList<const LidarPoint*>* list) const
{
    QMap<float, const LidarPoint*> distanceMap;
    for(int i=0;i<list->size();i++)
        distanceMap.insert(list->at(i)->squaredDistanceTo(point), list->at(i));

    // Does NOT delete its data, as QList does not take ownership (of pointers).
    // Use qDeleteAll() if you also want to delete() the pointers.
    list->clear();

    QMap<float, const LidarPoint*>::const_iterator i = distanceMap.constBegin();
    while(i != distanceMap.constEnd())
    {
        list->append(i.value());
        ++i;
    }
}

// FindNeighborsWithinRadius is used very often, without actually using the nodes returned, just checking their number
// Creating, passing and destroying QLists might not be the cheapest thing to do, so this could be more performant
quint32 PointCloudOctree::numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    uint32_t numberOfPointsFound = 0;
    PointCloudOctreeNode* currentNode = mRootNode->getLeaf(point);

    // Go up the tree as long as our searchradius leaks from the node
    // If we reach the rootnode, use him.
    while(currentNode->parent && !currentNode->isSphereContained(point, radius))
        currentNode = currentNode->parent;

    QList<PointCloudOctreeNode*> nodeList = currentNode->getAllChildLeafs();
    for(int i=0;i<nodeList.size();i++)
    {
        PointCloudOctreeNode* currentNode = nodeList.at(i);
        if(currentNode->overlapsSphere(point, radius))
        {
            //neighbors << currentNode->findNeighborsWithinRadius(point, radius);
            numberOfPointsFound += currentNode->numberOfNeighborsWithinRadius(point, radius);
        }
    }

    return numberOfPointsFound;
}

bool PointCloudOctree::isNeighborWithinRadius(const QVector3D &point, const double radius) const
{
    PointCloudOctreeNode* currentNode = mRootNode->getLeaf(point);

    // For performance reasons, check the same leaf first. This should work in 99% of cases.
    if(currentNode->neighborsWithinRadius(point, radius) > 0)
        return true;

    // Go up the tree as long as our searchradius leaks from the node
    // If we reach the rootnode, use him.
    while(currentNode->parent && !currentNode->isSphereContained(point, radius))
        currentNode = currentNode->parent;

    QList<PointCloudOctreeNode*> nodeList = currentNode->getAllChildLeafs();
    for(int i=0;i<nodeList.size();i++)
    {
        PointCloudOctreeNode* currentNode = nodeList.at(i);
        if(currentNode->overlapsSphere(point, radius))
        {
            if(currentNode->numberOfNeighborsWithinRadius(point, radius) > 0)
                return true;
        }
    }

    return false;
}

// Returns all neighbors of the given point within @radius
QList<const LidarPoint*> PointCloudOctree::findNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    QList<const LidarPoint*> neighbors;
    PointCloudOctreeNode* currentNode = mRootNode->getLeaf(point);

    // Go up the tree as long as our searchradius leaks from the node
    // If we reach the rootnode, use him.
    while(currentNode->parent && !currentNode->isSphereContained(point, radius))
        currentNode = currentNode->parent;

    foreach(const PointCloudOctreeNode* const n, currentNode->getAllChildLeafs())
    {
        if(n->overlapsSphere(point, radius))
            neighbors << n->findNeighborsWithinRadius(point, radius);
    }

    return neighbors;
}

// Returns AT LEAST the @count nearest neighbors of @point, sorted by ascending distance
QList<const LidarPoint*> PointCloudOctree::findNearestNeighbors(const QVector3D &point, const unsigned int count) const
{
    // Ask the leaf including this point for its neighbors
    PointCloudOctreeNode* containingNode = mRootNode->getLeaf(point);
    QList<const LidarPoint*> neighbors = containingNode->findNearestNeighbors(point, count);

    // sort the possibly empty list of neighbors
    sortPointList(point, &neighbors);

    // Keep a set of tested-leaf-ADDRESSES, so we don't test anyone twice.
    QSet<PointCloudOctreeNode*> leafsTested;
    leafsTested << containingNode;

    // As long as we don't have enough neighbors OR there might be better neighbors in parentnodes, ...
    while(
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

        QList<PointCloudOctreeNode*> childLeafs = containingNode->getAllChildLeafs();
        for(int i=0;i<childLeafs.size();i++)
        {
            PointCloudOctreeNode* ln = childLeafs.at(i);
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

void PointCloudOctree::pointInsertedByNode(const LidarPoint* lp)
{
    emit pointInserted(lp);
}

unsigned int PointCloudOctree::getNumberOfItems(void) const
{
    return mData->size();
    //return mNumberOfItems;
}

unsigned int PointCloudOctree::getNumberOfNodes(void) const
{
    return mNumberOfNodes;
}

void PointCloudOctree::updateVbo()
{
    // Check whether this octree has more points stored than the VBO
    quint32 numberOfPointsToStoreInAllVbos = getNumberOfItems() - mElementsStoredInAllVbos;

    while(numberOfPointsToStoreInAllVbos)
    {
        // Insert the lidarpoints into any VBO that can accomodate them
        // This maps from VBO-ID to numberOfElements (not bytes)
        QMapIterator<quint32, quint32> it(mVboIdsAndSizes);
        while(it.hasNext() && numberOfPointsToStoreInAllVbos)
        {
            it.next();

            // Only fill this VBO if it has enough free space for at least one point
            // An Octree can grow bigger than octree::mExpectedMaximumElementCount, but the VBOs are created for these many elements
            const unsigned int numberOfPointsToStoreInThisVbo = std::min(numberOfPointsToStoreInAllVbos, mExpectedMaximumElementCount - it.value());

            if(numberOfPointsToStoreInThisVbo)
            {
                glBindBuffer(GL_ARRAY_BUFFER, it.key());

                quint32 byteOffset = it.value() * sizeof(LidarPoint);

                // For updates < 32kb, glBufferSubData is supposed to be better than glMapBuffer
                glBufferSubData(
                            GL_ARRAY_BUFFER,
                            byteOffset, // offset in the VBO
                            numberOfPointsToStoreInThisVbo * sizeof(LidarPoint), // how many bytes to store?
                            (void*)(mData->constData() + it.value()) // data to store
                            );

                glBindBuffer(GL_ARRAY_BUFFER, 0);

                numberOfPointsToStoreInAllVbos -= numberOfPointsToStoreInThisVbo;

                // Update the number of bytes used
                mVboIdsAndSizes.insert(it.key(), it.value() + numberOfPointsToStoreInThisVbo);
                mElementsStoredInAllVbos += numberOfPointsToStoreInThisVbo;
            }
        }

        // We filled the existing VBOs with points above. But if we still
        // have a numberOfPointsToStore, we need to create a new VBO
        if(numberOfPointsToStoreInAllVbos)
        {
            // call glGetError() to clear eventually present errors
            glGetError();

            // Initialize the pointcloud-VBO
            const quint32 vboNewByteSize = mExpectedMaximumElementCount * sizeof(LidarPoint);
            GLuint vboNew;
            glGenBuffers(1, &vboNew);
            glBindBuffer(GL_ARRAY_BUFFER, vboNew);
            glBufferData(GL_ARRAY_BUFFER, vboNewByteSize, NULL, GL_DYNAMIC_DRAW);

            if(glGetError() == GL_NO_ERROR)
            {
                qDebug() << "PointCloudOctree::updateVbo(): Created new VBO" << vboNew << "containing" << vboNewByteSize << "bytes";
                mVboIdsAndSizes.insert(vboNew, 0);
            }
            else
            {
                qDebug() << "PointCloudOctree::updateVbo(): Couldn't create VBO containing" << vboNewByteSize << "bytes!";
            }

            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }
}
