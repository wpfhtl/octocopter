#include "octree.h"
//octree.h includes node.h! #include "node.h"

#define SQR(x) (x)*(x)

Node::Node(Octree* tree, Node* parent, const QVector3D &min, const QVector3D &max) :
    mTree(tree),
    parent(parent),
    min(min),
    max(max)
{
    mTree->mNumberOfNodes++;

    // I *think* this is necessary for Node::isSpereContained()
    //    Q_ASSERT(min.x() < max.x());
    //    Q_ASSERT(min.y() < max.y());
    //    Q_ASSERT(min.z() < max.z());
    //    qDebug() << "Node::Node(): creating Node" << mNumberOfNodes << this << "from" << min << "to" << max;
}

Node::~Node()
{
    // We clear our structures, but we do NOT clear the data.
    pointIndices.clear();

    // Make all subnodes do the same.
    qDeleteAll(children);
    children.clear();

    mTree->mNumberOfNodes--;
}

// This method should destruct the container, but keep the data
void Node::clearPoints()
{
    Q_ASSERT("This is a mess, we can only delete the whole thing!");
    if(isLeaf())
    {
//        mTree->mNumberOfItems -= data.size();
//        qDeleteAll(data);
//        data.clear();
    }
    else
    {
        foreach(Node* const currentNode, children)
            currentNode->clearPoints(); // first make it clear() its @data without deleteing the points...

        qDeleteAll(children);
        children.clear();
    }
}

Node& Node::operator=(const Node &other)
{
    // We create a shallow copy.
    mTree = other.mTree;
    parent = other.parent;
    min = other.min;
    max = other.max;
    pointIndices = other.pointIndices;
    children = other.children;

    return *this;
}

bool Node::operator==(const Node &other)
{
    return
            mTree == other.mTree &&
            parent == other.parent &&
            min == other.min &&
            max == other.max &&
            pointIndices == other.pointIndices &&
            children == other.children;
}


inline bool Node::isLeaf(void) const
{
    return children.size() == 0;
}

inline bool Node::includesPoint(const QVector3D &point) const
{
    return
            min.x() <= point.x() &&
            min.y() <= point.y() &&
            min.z() <= point.z() &&

            point.x() <= max.x() &&
            point.y() <= max.y() &&
            point.z() <= max.z();
}



/*
bool Node::includesData(const LidarPoint &lidarPoint)
{
    if(isLeaf())
    {
        // the lidarPoint must be in data, else its not in here.
        if(includesPoint(lidarPoint.position))
        {
            //foreach(const LidarPoint* p, data)
            //{
            //    if(*p == lidarPoint)
            //        return true;
            //}
            for(int i=0;i<pointIndices.size();i++)
            {
                LidarPoint* p = &mTree->mData[pointIndices.at(i)];
                if(*p == lidarPoint)
                    return true;
            }
            return false;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // check the correct subNode
        getLeaf(lidarPoint.position)->includesData(lidarPoint);
    }
}*/

bool Node::insertAndReduce(LidarPoint* const lidarPoint)
{
    // Linear reduction:
    // If there have been two points inserted before, check whether @lidarPoint is close to where
    // a line from mMri2 to mMri1, extended by the distance from mMri2 to @lidarPoint, would end:
    // (mMri1 is mMostRecentlyInserted1)
    //
    // mMri2 --------------------> mMri1 ----------------------> @lidarPoint
    //
    // If it is, delete mMri1, insert lidarPoint and let mMri1 point to lidarPoint.
    // If not, just insert lidarPoint normally.

    /* the long version:
        const QVector3D origin = mMri2->position;
        const QVector3D direction = (mMri1->position - mMri2->position).normalized();
        const float distanceMri1ToLidarPoint = mMri1->position.distanceToLine(lidarPoint->position, QVector3D());
        const QVector3D lidarPointAnticipated = origin + (distanceMri1ToLidarPoint * direction.normalized());
        if((lidarPointAnticipated - lidarPoint->position).lengthSquared() < 0.25)
            ....
    */
/*
    if(
            false && // FIXME, disabled for testing
            mTree->mMri1
            &&
            mTree->mMri2
            &&
            (mTree->mMri2->position + (mTree->mMri1->position.distanceToLine(lidarPoint->position, QVector3D()) * (mTree->mMri1->position - mTree->mMri2->position).normalized()) - lidarPoint->position).lengthSquared() < 3.0
            )
    {
        Q_ASSERT("port to use indices!");
        qDebug() << "Node::insertAndReduce(): sweet, reducing a point." << pointIndices.size();
        // delete mMri1, its on the ray between mMri2 and lidarPoint
        mTree->mMri1->node->deletePoint(mTree->mMri1);
        mTree->mMri1 = lidarPoint;

        //data.append(lidarPoint);
        lidarPoint->node = this;
        mTree->mNumberOfItems++;
        return true;
    }
    else*/
    {
        //qDebug() << "Node::insertAndReduce(): aww, reduction failed, will insert.";

        // Probably stupid, but lets start easy: Do not insert if it has N close-by neighbors
//        if(!mTree->isNeighborWithinRadius(lidarPoint->position, mTree->mMinimumPointDistance))
        if(!neighborsWithinRadius(lidarPoint->position, mTree->mMinimumPointDistance))
        {
            const quint32 masterStorageSize = mTree->mData->size();
            mTree->mData->append(*lidarPoint);

            // Add the index of this new point in the master storage to our index-vector
            pointIndices.append(masterStorageSize);

            // update the offsets for the next iteration
//            mTree->mMri2 = mTree->mMri1;
//            mTree->mMri1 = masterStorageSize;

            // old, use master vector size
            //mTree->mNumberOfItems++;
            return true;
        }
        else
        {
            // We delete the point, as it does not contain much information. Haha.
            delete lidarPoint;
            return false;
        }
    }

}

Node* Node::insertPoint(LidarPoint* const lidarPoint) // a const pointer to a non-const LidarPoint
{
    // Insert a lidarPoint. If we are a leaf and we contain the point, go ahead
    //    qDebug() << "Node::insertPoint(): inserting point to node" << this << "at" << lidarPoint->position;
    if(isLeaf())
    {
        // Now insert the lidarPoint. If its found to be of low informational
        // value, it'll be discarded. In that case, just return.
        if(!insertAndReduce(lidarPoint)) return 0;

        mTree->pointInsertedByNode(lidarPoint);

        // TODO: don't just append, merge with neighbors and weigh the LPs distances/normals

        // if this leaf is full, create children and partition all our guests into new nodes
        if(pointIndices.size() > mTree->mMaxItemsPerLeaf)
        {
            // create childnodes
            partition();

            // take every point we curently host...
            for(int i=0;i<pointIndices.size();i++)
            {
                //LidarPoint* const lp = data.takeFirst();
                //const quint32 pointIndex = pointIndices.takeFirst();
                const LidarPoint lp = mTree->mData->at(pointIndices.at(i));
                // ... and put it into the correct child. We *could* use that leaf's insertPoint()
                // here, but that does some unneccessary checks (isLeaf(), includesPoint(), size
                // check), so we rather move the payload directly.
                //getLeaf(lp->position)->insertPoint(lp);

                // All of the mMaxItemsPerLeaf+1 lidarPoints in this node might have moved down to
                // the same octant, so that octant might now contain the mMaxItemsPerLeaf+1 nodes.
                // But thats an unlikely case and will be fixed on its next insertPoint().
                getLeaf(lp.position)->pointIndices.append(pointIndices.at(i));
            }
            pointIndices.clear();
        }

        return this;
        //            qDebug() << "Node::insertPoint(): I'm a leaf, but I don't include the point" << lidarPoint->position;
        //            Q_ASSERT(false);

    }
    else
    {
        // This is not a leaf. Into which subNode should we insert lidarPoint?
        //        qDebug() << "Node::insertPoint(): I'm not leaf, forwarding insertion request to correct octant";
        Node* subNode = getLeaf(lidarPoint->position);
        return subNode->insertPoint(lidarPoint);
    }
}

bool Node::overlapsSphere(const QVector3D &point, const double radius) const
{
    // http://tog.acm.org/resources/GraphicsGems/gems/BoxSphere.c

    float  dmin = 0;
    double  radiusSquared = SQR(radius);

    if(point.x() < min.x())
        dmin += SQR(point.x() - min.x());
    else if(point.x() > max.x())
        dmin += SQR(point.x() - max.x());

    if(point.y() < min.y())
        dmin += SQR(point.y() - min.y());
    else if(point.y() > max.y())
        dmin += SQR(point.y() - max.y());

    if(point.z() < min.z())
        dmin += SQR(point.z() - min.z());
    else if(point.z() > max.z())
        dmin += SQR(point.z() - max.z());

    if(dmin <= radiusSquared)
        return true;
    else
        return false;
}

QList<const LidarPoint*> Node::findNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    const double radiusSquared = SQR(radius);

    QList<const LidarPoint*> result;

    /*
    foreach(LidarPoint* const p, data)
        if(p->squaredDistanceTo(point) <= radiusSquared)
            result << p;
            */
    for(int i=0;i<pointIndices.size();i++)
    {
        if(mTree->mData->at(i).squaredDistanceTo(point) <= radiusSquared)
            result << &mTree->mData->at(i);
    }

    return result;
}

quint32 Node::numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    const double radiusSquared = SQR(radius);

    quint32 number = 0;

    for(int i=0;i<pointIndices.size();i++)
    {
        if(mTree->mData->at(i).squaredDistanceTo(point) <= radiusSquared)
            number++;
    }

    return number;
}

QList<const LidarPoint*> Node::findNearestNeighbors(const QVector3D &point, const unsigned int count) const
{
    // This node includes the given point.
    if(isLeaf())
    {
        // This node is a leaf, go and find the @count nearest neighbors.
        // map from distance => pointIndex
        QMap<float, quint32> distanceMap;

        //        foreach(LidarPoint* const p, data)
        //            distanceMap.insert(p->squaredDistanceTo(point), p);
        for(int i=0;i<pointIndices.size();i++)
            distanceMap.insert(mTree->mData->at(i).squaredDistanceTo(point), i);

        QList<const LidarPoint*> result;

        QMap<float, quint32>::const_iterator i = distanceMap.constBegin();
        while(i != distanceMap.constEnd() && result.size() < count)
        {
            result << &mTree->mData->at(i.value());
            ++i;
        }

        return result;
    }

    Q_ASSERT(false);
}

bool Node::neighborsWithinRadius(const QVector3D &point, const float radius) const
{
    const float radiusSquared = SQR(radius);
    qDebug() << "Node::neighborsWithinRadius(): checking" << pointIndices.size() << "points for neighborhood closer than" << radius << "to" << point;

    for(int i=0;i<pointIndices.size();i++)
    {
        const float distanceToPointSquared = mTree->mData->at(i).squaredDistanceTo(point);
        if(distanceToPointSquared <= radiusSquared)
            return true;
    }

    return false;
}

// Returns true if the given sphere fits completely inside this Node.
bool Node::isSphereContained(const QVector3D point, const double radius)
{
    if(isBoxContained(
                QVector3D(point.x()-radius, point.y()-radius, point.z()-radius),
                QVector3D(point.x()+radius, point.y()+radius, point.z()+radius)
                ))
    {
        //        qDebug() << "Node::isSphereContained() point" << point << "radius" << radius << "IS contained";
        return true;
    }
    else
    {
        //        qDebug() << "Node::isSphereContained() point" << point << "radius" << radius << "IS NOT contained";
        return false;
    }
}

inline bool Node::isBoxContained(const QVector3D &min, const QVector3D &max)
{
    return includesPoint(min) && includesPoint(max);
}

// Creates a sub-partitioning for this octree-node.
void Node::partition()
{
    const QVector3D ctr = center();

    // create octant 0
    children.append(
                new Node
                (
                    mTree,
                    this,
                    min,
                    ctr
                    )
                );

    // create octant 1
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        ctr.x(),
                        min.y(),
                        min.z()
                        ),
                    QVector3D
                    (
                        max.x(),
                        ctr.y(),
                        ctr.z()
                        )
                    )
                );

    // create octant 2
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        ctr.x(),
                        min.y(),
                        ctr.z()
                        ),
                    QVector3D
                    (
                        max.x(),
                        ctr.y(),
                        max.z()
                        )
                    )
                );

    // create octant 3
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        min.x(),
                        min.y(),
                        ctr.z()
                        ),
                    QVector3D
                    (
                        ctr.x(),
                        ctr.y(),
                        max.z()
                        )
                    )
                );

    // create octant 4
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        min.x(),
                        ctr.y(),
                        min.z()
                        ),
                    QVector3D
                    (
                        ctr.x(),
                        max.y(),
                        ctr.z()
                        )
                    )
                );

    // create octant 5
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        ctr.x(),
                        ctr.y(),
                        min.z()
                        ),
                    QVector3D
                    (
                        max.x(),
                        max.y(),
                        ctr.z()
                        )
                    )
                );

    // create octant 6
    children.append(
                new Node
                (
                    mTree,
                    this,
                    ctr,
                    max
                    )
                );

    // create octant 7
    children.append(
                new Node
                (
                    mTree,
                    this,
                    QVector3D
                    (
                        min.x(),
                        ctr.y(),
                        ctr.z()
                        ),
                    QVector3D
                    (
                        ctr.x(),
                        max.y(),
                        max.z()
                        )
                    )
                );
}

// Returns the correct leaf-node for a position within this Node.
Node* Node::getLeaf(const QVector3D &point)
{
    //    qDebug() << "Node::getLeaf(): Node" << this << ", from" << min << "to" << max << "point" << point;
    if(isLeaf())
    {
        // Leaves have no children
        if(includesPoint(point))
        {
            //            qDebug() << "Node::getLeaf(): Node" << this << "contains point" << point;
            return this;
        }
        else
        {
            // We are a leaf, but we don't contain the point. Thats wrong.
            Q_ASSERT(false);
        }
    }
    else
    {
        // We have octants. Find the right one and let him do the work.
        //        Q_ASSERT(children);

        // TODO: optimize by not looping, but thinking.
        foreach(Node* const currentNode, children)
        {
            if(currentNode->includesPoint(point))
                return currentNode->getLeaf(point);
        }

        // At this point, it seems none of our octants include the point. Thats wrong.
        Q_ASSERT(false);
    }
}

QList<Node*> Node::getAllChildLeafs(void)
{
    QList<Node*> result;

    if(isLeaf())
    {
        result << this;
        return result;
    }

    foreach(Node* const currentNode, children)
    {
        result << currentNode->getAllChildLeafs();
    }

    return result;
}

const QList<const Node*> Node::getAllChildLeafs(void) const
{
    QList<const Node*> result;

    if(isLeaf())
    {
        result << this;
        return result;
    }

    foreach(const Node* const currentNode, children)
    {
        result << currentNode->getAllChildLeafs();
    }

    return result;
}

/* Returns this Node's octants' extremes, in the same order as the octants
QList<QVector3D> Node::corners(void) const
{
    // TODO: precalculate this on construction
    QList<QVector3D> corners;

    corners << min; // 0
    corners << QVector3D(max.x(), min.y(), min.z()); // 1
    corners << QVector3D(max.x(), min.y(), max.z()); // 2
    corners << QVector3D(min.x(), min.y(), max.z()); // 3
    corners << QVector3D(min.x(), max.y(), min.z()); // 4
    corners << QVector3D(max.x(), max.y(), min.z()); // 5
    corners << max; // 6
    corners << QVector3D(min.x(), max.y(), max.z()); // 7

    return corners;
}*/

QVector3D Node::size() const
{
    return max - min;
}

// Returns this Node's center
inline QVector3D Node::center(void) const
{
    return (min + max) * 0.5f;
}

/*
bool Node::deletePoint(LidarPoint* const lidarPoint)
{
    for(int i=0;i<pointIndices.size();i++)
    {
        if(data.at(i) == lidarPoint)
        {
            mTree->mData->remove(data.takeAt(i));
            return true;
        }
    }
    return false;
}*/

// delete the point at this position
bool Node::deletePoint(const LidarPoint &lidarPoint)
{
    for(int i=0;i<pointIndices.size();i++)
    {
        if(mTree->mData->at(pointIndices.at(i)).position == lidarPoint.position)
        {
            // This is SLOOOOOOOW!
            mTree->mData->remove(pointIndices.at(i));
            pointIndices.remove(i);
            return true;
        }
    }
    return false;
}

/*LidarPoint* Node::getLidarPointFromIndex(const quint32 index)
{
    return &mTree->mData[index];
}*/
