#include "node.h"

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
    mTree->mNumberOfItems -= data.size();
    mTree->mNumberOfNodes--;

    qDeleteAll(data);
    qDeleteAll(children);
}

Node& Node::operator=(const Node &other)
{
    // We create a shallow copy.
    mTree = other.mTree;
    parent = other.parent;
    min = other.min;
    max = other.max;
    data = other.data;
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
            data == other.data &&
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

bool Node::includesData(const LidarPoint &lidarPoint)
{
    if(isLeaf())
    {
        // the lidarPoint must be in data, else its not in here.
        if(includesPoint(lidarPoint.position))
        {
            foreach(const LidarPoint* p, data)
            {
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
}

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

    if(
            false && // FIXME, disabled for testing
            mTree->mMri1
            &&
            mTree->mMri2
            &&
            (mTree->mMri2->position + (mTree->mMri1->position.distanceToLine(lidarPoint->position, QVector3D()) * (mTree->mMri1->position - mTree->mMri2->position).normalized()) - lidarPoint->position).lengthSquared() < 3.0
            )
    {
        qDebug() << "Node::insertAndReduce(): sweet, reducing a point." << data.size();
        // delete mMri1, its on the ray between mMri2 and lidarPoint
        mTree->mMri1->node->deletePoint(mTree->mMri1);
        mTree->mMri1 = lidarPoint;

        data.append(lidarPoint);
        lidarPoint->node = this;
        mTree->mNumberOfItems++;
        return true;
    }
    else
    {
//        qDebug() << "Node::insertAndReduce(): aww, reduction failed, will insert." << data.size();

        // Probably stupid, but lets start easy: Do not insert if it has N close-by neighbors
        if(! mTree->isNeighborWithinRadius(lidarPoint->position, mTree->mMinimumPointDistance))
        {
            data.append(lidarPoint);
            lidarPoint->node = this;
            mTree->mNumberOfItems++;

            // update the pointers for the next iteration
            mTree->mMri2 = mTree->mMri1;
            mTree->mMri1 = lidarPoint;
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
//        if(includesPoint(lidarPoint->position))
//        {
            // Now insert the lidarPoint. If its found to be of low informational
            // value, it'll be discarded. In that case, just return.
            if(!insertAndReduce(lidarPoint)) return 0;

//            qDebug() << "Node::insertPoint(): point" << lidarPoint->position << "saved, total nodes" << mNumberOfNodes << "items" << mNumberOfItems;

            // TODO: don't just append, merge with neighbors and weigh the LPs distances/normals

            // if this leaf is full, create children and partition all our guests into new nodes
            if(data.size() > mTree->mMaxItemsPerLeaf)
            {
                // create childnodes
                qDebug() << "more than mMaxItemsPerLeaf in node, partitioning...";
                partition();

                // take every point we curently host...
                while(data.size())
                {
                    LidarPoint* const lp = data.takeFirst();
                    // ... and put it into the correct child. We *could* use that leaf's insertPoint()
                    // here, but that does some unneccessary checks (isLeaf(), includesPoint(), size
                    // check), so we rather move the payload directly.
                    //getLeaf(lp->position)->insertPoint(lp);

                    // All of the mMaxItemsPerLeaf+1 lidarPoints in this node might have moved down to
                    // the same octant, so that octant might now contain the mMaxItemsPerLeaf+1 nodes.
                    // But thats an unlikely case and will be fixed on its next insertPoint().
                    getLeaf(lp->position)->data.append(lp);
                }
            }

            return this;
//        }
//        else
//        {
            // We're supposed to save a lidarPoint that this node cannot contain.
            // Thats a pretty grave error.
//            qDebug() << "Node::insertPoint(): I'm a leaf, but I don't include the point" << lidarPoint->position;
//            Q_ASSERT(false);
//        }
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

QList<LidarPoint*> Node::findNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    const double radiusSquared = SQR(radius);

    QList<LidarPoint*> result;

    foreach(LidarPoint* const p, data)
        if(p->squaredDistanceTo(point) <= radiusSquared)
            result << p;

    return result;
}

uint32_t Node::numberOfNeighborsWithinRadius(const QVector3D &point, const double radius) const
{
    const double radiusSquared = SQR(radius);

    uint32_t number = 0;

    foreach(LidarPoint* const p, data)
        if(p->squaredDistanceTo(point) <= radiusSquared)
            number++;

    return number;
}

QList<LidarPoint*> Node::findNearestNeighbors(const QVector3D &point, const unsigned int count) const
{
    // This node includes the given point.
    if(isLeaf())
    {
        // This node is a leaf, go and find the @count nearest neighbors.
        QMap<double, LidarPoint*> distanceMap;
        foreach(LidarPoint* const p, data)
            distanceMap.insert(p->squaredDistanceTo(point), p);

        QList<LidarPoint*> result;

        QMap<double, LidarPoint*>::const_iterator i = distanceMap.constBegin();
        while(i != distanceMap.constEnd() && result.size() < count)
        {
            result << i.value();
            ++i;
        }

        return result;

    }

    Q_ASSERT(false);
}

bool Node::neighborsWithinRadius(const QVector3D &point, const double radius) const
{
    const double radiusSquared = SQR(radius);

    foreach(LidarPoint* const p, data)
        if(p->squaredDistanceTo(point) <= radiusSquared)
            return true;

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

    /*
    QList<QVector3D> planes = this->planes();
    for(int i=0; i < planes.size(); i+=2)
    {
        if(radius > point.distanceToPlane(planes.at(i), planes.at(i+1)))
        {
            // Sphere leaks this node.
            return false;
        }
    }

    // The given sphere does not leak this node.
    return true;
    */
}

inline bool Node::isBoxContained(const QVector3D &min, const QVector3D &max)
{
    return includesPoint(min) && includesPoint(max);
}

// Creates a sub-partitioning for this octree-node.
void Node::partition()
{
//    qDebug() << "Node::partition(): Seems I'm full, partitioning...";
//    children = new QList<Node>;

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

/* my attempt at a general partitioning method
void Node::partition()
{
    // Partition in how many segments per axis?
    const int divisions = 2;

    children = new QList<Node>;

    QList xValues<double>;
    for(int i=0;i<=divisions;i++) xValues << min.x() + i*(max.x()-min.x());

    QList yValues<double>;
    for(int i=0;i<=divisions;i++) yValues << min.y() + i*(max.y()-min.y());

    QList zValues<double>;
    for(int i=0;i<=divisions;i++) zValues << min.z() + i*(max.z()-min.z());

    int listPositions[divisions + 1];

    for(int i=0;i<pow(3, divisions);i++)
    {
        for(int j=0;j<=divisions;j++)
            listPositions[]


        Node* newNode(

                );
        children.append(newNode);
    }
}
*/

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
/*
// Returns this Node's octants' extremes, in the same order as the octants
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
}
*/

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
// Returns this Nodes' planes as point/normal pairs
QList<QVector3D> Node::planes(void) const
{
    // TODO: precalculate all of this on construction
    const double centerX = min.x()+(max.x()-min.x())/2;
    const double centerY = min.y()+(max.y()-min.y())/2;
    const double centerZ = min.z()+(max.z()-min.z())/2;

    QList<QVector3D> result;

    result << QVector3D(centerX, centerY, min.z()) << QVector3D(0, 0, -1);
    result << QVector3D(max.x(), centerY, centerZ) << QVector3D(1, 0, 0);
    result << QVector3D(centerX, centerY, max.z()) << QVector3D(0, 0, 1);
    result << QVector3D(min.x(), centerY, centerZ) << QVector3D(-1, 0, 0);
    result << QVector3D(centerX, max.y(), centerZ) << QVector3D(0, 1, 0);
    result << QVector3D(centerX, min.y(), centerZ) << QVector3D(0, -1, 0);

    return result;
}
*/

void Node::handlePoints() const
{
    foreach(const LidarPoint* const p, data)
        mTree->mPointHandler(p->position);

    foreach(const Node* const n, children)
        n->handlePoints();
}

//void Node::drawGl(void) const
//{
//    glDisable(GL_LIGHTING);
//    GLint r = (*(int*)(this+4));// % 255;
//    GLint g = (*(int*)(this+2));// % 255;
//    GLint b = (*(int*)(this+3));// % 255;
//    glColor4i(r,g,b, 1147483648);

//    glColor4f(1,1,1, 0.03);
//    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
//    glLineWidth(1);

/*
    glBegin(GL_LINE_LOOP);
    glVertex3f(min.x(), min.y(), min.z()); // 0
    glVertex3f(max.x(), min.y(), min.z()); // 1
    glVertex3f(max.x(), min.y(), max.z()); // 2
    glVertex3f(min.x(), min.y(), max.z()); // 3
    glVertex3f(min.x(), min.y(), min.z()); // 0
    glVertex3f(min.x(), max.y(), min.z()); // 4
    glVertex3f(max.x(), max.y(), min.z()); // 5
    glVertex3f(max.x(), max.y(), max.z()); // 6
    glVertex3f(min.x(), max.y(), max.z()); // 7
    glVertex3f(min.x(), max.y(), min.z()); // 4
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(max.x(), min.y(), min.z()); // 1
    glVertex3f(max.x(), max.y(), min.z()); // 5
    glVertex3f(max.x(), max.y(), max.z()); // 6
    glVertex3f(max.x(), min.y(), max.z()); // 2
    glVertex3f(min.x(), min.y(), max.z()); // 3
    glVertex3f(min.x(), max.y(), max.z()); // 7
    glVertex3f(max.x(), max.y(), max.z()); // 6
    glVertex3f(max.x(), max.y(), min.z()); // 5
    glEnd();
    */

//    glLineWidth(1);
//    glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
//    glBegin(GL_POINTS);
//    foreach(const LidarPoint* const p, data)
//        glVertex3f(p->position.x(), p->position.y(), p->position.z());
//    glEnd();

    /*
    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLE_FAN);
    foreach(const LidarPoint* const p, data)
    {
        QList<LidarPoint*> neighbors = mTree->findNearestNeighbors(p->position, 3);
        if(neighbors.size() < 3) continue;

        glVertex3f(p->position.x(), p->position.y(), p->position.z());

        foreach(const LidarPoint* const q, neighbors)
            glVertex3f(q->position.x(), q->position.y(), q->position.z());

        glVertex3f(neighbors.first()->position.x(), neighbors.first()->position.y(), neighbors.first()->position.z());
    }
    glEnd();
    */

//    glEnable(GL_LIGHTING);

//    foreach(const Node* const n, children)
//        n->drawGl();
//}

bool Node::deletePoint(LidarPoint* const lidarPoint)
{
    for(int i=0;i<data.size();i++)
    {
        if(data.at(i) == lidarPoint)
        {
            delete data.takeAt(i);
            mTree->mNumberOfItems--;
            return true;
        }
    }
    return false;
}

// delete the point at this position
bool Node::deletePoint(const LidarPoint &lidarPoint)
{
    for(int i=0;i<data.size();i++)
    {
        if(data.at(i)->position == lidarPoint.position)
        {
            delete data.takeAt(i);
            mTree->mNumberOfItems--;
            return true;
        }
    }
    return false;
}
