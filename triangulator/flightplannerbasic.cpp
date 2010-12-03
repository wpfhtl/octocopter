#include "flightplannerbasic.h"

FlightPlannerBasic::FlightPlannerBasic(const QVector3D * const position, const QQuaternion * const orientation, Octree* pointCloud) : FlightPlanner(position, orientation, pointCloud)
{
    // The octree is initialized on arrival of the first point, with this point at its center.
    // We do this so we can drop spheres only within the octree's XZ plane.
    mOctree = 0;
}

FlightPlannerBasic::~FlightPlannerBasic()
{
}

void FlightPlannerBasic::slotWayPointReached(const QVector3D /*wayPoint*/)
{
    // gut!
}


Node* FlightPlannerBasic::insertPoint(LidarPoint* const point)
{
    if(mOctree == 0)
    {
        // Create the octree around the first arriving point.
        mOctree = new Octree(
                point->position - QVector3D(10, 10, 10), // min
                point->position + QVector3D(10, 10, 10),  // max
                10);

        mOctree->setMinimumPointDistance(20);

        mOctree->setPointHandler(GlWidget::drawSphere);

        connect(mOctree, SIGNAL(pointInserted(const LidarPoint*)), SLOT(slotPointInserted(const LidarPoint*)));
    }

    Node* insertionNode = mOctree->insertPoint(point);

    return insertionNode;
}

void FlightPlannerBasic::slotPointInserted(const LidarPoint* lp)
{
    LidarPoint wayPoint(*lp);

    wayPoint.position.setY(wayPoint.position.y()+15.0);

    emit newWayPoint(wayPoint.position);
}

void FlightPlannerBasic::visualize() const
{
    if(mOctree) mOctree->handlePoints();
}
