#include "box3d.h"
#include "common.h"

Box3D::Box3D()
{
    min = QVector3D(-32, -32, -32);
    max = QVector3D(32, 32, 32);
    mVisitorGrid = nullptr;
}

Box3D::Box3D(const QVector3D& minBox, const QVector3D& maxBox)
{
    min = minBox;
    max = maxBox;
    mVisitorGrid = nullptr;
}

Box3D::~Box3D()
{
    delete mVisitorGrid;
}

QVector3D Box3D::size() const
{
    return max - min;
}

QVector3D Box3D::center() const
{
    return min + (max - min)/2.0f;
}

bool Box3D::containsAllOf(const Box3D* other) const
{
    return
            min.x() <= other->min.x() &&
            min.y() <= other->min.y() &&
            min.z() <= other->min.z() &&
            max.x() >= other->max.x() &&
            max.y() >= other->max.y() &&
            max.z() >= other->max.z();
}

bool Box3D::contains(const QVector3D* point) const
{
    return
            min.x() <= point->x() &&
            min.y() <= point->y() &&
            min.z() <= point->z() &&
            max.x() >= point->x() &&
            max.y() >= point->y() &&
            max.z() >= point->z();
}

Box3D Box3D::tryToKeepWithin(const Box3D& other) const
{
    const QVector3D otherCenter = other.center();
    const QVector3D otherSize = other.size();
    Box3D result;

    if(size().x() <= otherSize.x())
    {
        // fitting is possible - move within bounds if necessary!
        if(min.x() < other.min.x())
        {
            result.min.setX(other.min.x());
            result.max.setX(result.min.x() + size().x());
        }
        else if(max.x() > other.max.x())
        {
            result.min.setX(other.max.x() - size().x());
            result.max.setX(other.max.x());
        }
        else
        {
            result.min.setX(min.x());
            result.max.setX(max.x());
        }
    }
    else
    {
        // box is too large in this dimension, so place it in center of other
        result.min.setX(otherCenter.x() - size().x()/2.0f);
        result.max.setX(otherCenter.x() + size().x()/2.0f);
    }

    if(size().y() <= otherSize.y())
    {
        // fitting is possible - move within bounds if necessary!
        if(min.y() < other.min.y())
        {
            result.min.setY(other.min.y());
            result.max.setY(result.min.y() + size().y());
        }
        else if(max.y() > other.max.y())
        {
            result.min.setY(other.max.y() - size().y());
            result.max.setY(other.max.y());
        }
        else
        {
            result.min.setY(min.y());
            result.max.setY(max.y());
        }
    }
    else
    {
        // box is too large in this dimension, so place it in center of other
        result.min.setY(otherCenter.y() - size().y()/2.0f);
        result.max.setY(otherCenter.y() + size().y()/2.0f);
    }


    if(size().z() <= otherSize.z())
    {
        // fitting is possible - move within bounds if necessary!
        if(min.z() < other.min.z())
        {
            result.min.setZ(other.min.z());
            result.max.setZ(result.min.z() + size().z());
        }
        else if(max.z() > other.max.z())
        {
            result.min.setZ(other.max.z() - size().z());
            result.max.setZ(other.max.z());
        }
        else
        {
            result.min.setZ(min.z());
            result.max.setZ(max.z());
        }
    }
    else
    {
        // box is too large in this dimension, so place it in center of other
        result.min.setZ(otherCenter.z() - size().z()/2.0f);
        result.max.setZ(otherCenter.z() + size().z()/2.0f);
    }

    return result;
}

void Box3D::setCenter(const QVector3D& center)
{
    const QVector3D sz = size();
    min = center - sz/2.0f;
    max = center + sz/2.0f;
}

Box3D Box3D::moveToNextPosition(const QVector3D& vehiclePosition, const Box3D& boundingBox)
{
    Q_ASSERT(contains(&vehiclePosition));
/*
    const Vector3i gridCells = Vector3i(
                ceil(mBoundingBox->size().x() / size().x()),
                ceil(mBoundingBox->size().y() / size().y()),
                ceil(mBoundingBox->size().z() / size().z())
                );

    // If we haven't so far, create a grid so we can remember which places in the boundingbox our small box has visited.
    if(mVisitorGrid == nullptr)
    {
        mBoundingBox = new Box3D(boundingBox);

        mVisitorGrid = new QVector<bool>(gridCells.x * gridCells.z, false);
        mTravelDirection = Vector3i(-1, 0, 0); // travel left (-x)

        Box3D result;
        result.setCenter(boundingBox.center());
        return result;
    }

    // The box has been positioned previously.
    Q_ASSERT(mBoundingBox == boundingBox);

    const Vector3i currentCell = Vector3i(
                floor(mBoundingBox->size().x() / size().x()),
                floor(mBoundingBox->size().y() / size().y()),
                floor(mBoundingBox->size().z() / size().z())
                );
*/
    Q_ASSERT(false);
    return Box3D();
}


QDebug operator<<(QDebug dbg, const Box3D &box)
{
    dbg << "Box3D:" << box.min << box.max;
    return dbg;
}
