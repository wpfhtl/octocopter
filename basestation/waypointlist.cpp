#include "waypointlist.h"

WayPointList::WayPointList()
{
    mVbo = 0;
    mSphereSize = 1.0f;
}

WayPointList::WayPointList(const QColor& color)
{
    mVbo = 0;
    mColor = color;
    mWaypoints.clear();
    mSphereSize = 1.0f;
}

WayPointList::~WayPointList()
{
    if(mVbo) glDeleteBuffers(1, &mVbo);
}

void WayPointList::append(const WayPoint& wp) {mWaypoints.append(wp); setVbo();}
void WayPointList::append(const QList<WayPoint>& wps) {mWaypoints.append(wps); setVbo();}
void WayPointList::append(const WayPointList* wpl) {mWaypoints.append(wpl->mWaypoints); setVbo();}
void WayPointList::remove(const quint16& index) {mWaypoints.removeAt(index); setVbo();}
void WayPointList::clear() {mWaypoints.clear(); setVbo();}
void WayPointList::insert(const quint16& index, const WayPoint& wp) {mWaypoints.insert(index, wp); setVbo();}

WayPoint WayPointList::takeAt(const int index)
{
    const WayPoint wpt = mWaypoints.takeAt(index);
    setVbo();
    return wpt;
}

void WayPointList::mergeCloseWaypoints(const float minimumDistance)
{
    for(int i=0;i<mWaypoints.size();i++)
    {
        const WayPoint& w1 = mWaypoints.at(i);

        // Do not check the current waypoint (i) against previous waypoints (j <= i),
        // because those previous waypoints (j) have already been checked against the current (i).
        // That is, only check following WPs, not preceding ones.
        for(int j=i+1;j<mWaypoints.size();j++)
        {
            // On the last i-element, there is no following WP
            if(mWaypoints.size() > j)
            {
                const WayPoint& w2 = mWaypoints.at(j);
                if(w1.distanceToLine(w2, QVector3D()) < minimumDistance)
                {
                    mWaypoints.removeAt(j);
                    j--;
                }
            }
        }
    }
}

void WayPointList::sortToShortestPath(const QVector3D &vehiclePosition)
{
    //    qDebug() << "FlightPlannerInterface::sortToShortestPath(): vehicle is at" << currentVehiclePosition;

    float distanceBefore = 0;

//    for(int i=1;i<mWaypoints.size();i++) distanceBefore += mWaypoints.at(i-1).distanceToLine(mWaypoints.at(i), QVector3D());
//    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << mWaypoints.size() << "points before:" << distanceBefore;

    QList<WayPoint> wps(mWaypoints);
//    float distanceBeforewps = 0;
//    for(int i=1;i<wps.size();i++) distanceBeforewps += wps.at(i-1).distanceToLine(wps.at(i), QVector3D());
//    qDebug() << "FlightPlannerInterface::sortToShortestPath(): wps total distance between" << wps.size() << "points before:" << distanceBeforewps;

    mWaypoints.clear();
    mWaypoints.append(vehiclePosition);

    while(wps.size())
    {
        float closestNeighborDistance = 9999999999999999.0f;
        int indexOfClosestNeighbor = -1;

        for(int i=0;i<wps.size();i++)
        {
            const float currentDistance = mWaypoints.last().distanceToLine(wps.at(i), QVector3D());
            if(currentDistance < closestNeighborDistance)
            {
                closestNeighborDistance = currentDistance;
                indexOfClosestNeighbor = i;
            }
        }

        mWaypoints.append(wps.at(indexOfClosestNeighbor));
        wps.removeAt(indexOfClosestNeighbor);
    }

    mWaypoints.takeFirst();

    setVbo();

//    float distanceAfter = 0;
//    for(int i=1;i<mWaypoints.size();i++) distanceAfter += mWaypoints.at(i-1).distanceToLine(mWaypoints.at(i), QVector3D());
    //    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points after:" << distanceAfter;
}

// Copy all waypoints into our VBO
void WayPointList::setVbo()
{
    if(!mVbo)
    {
        initializeOpenGLFunctions();
        glGenBuffers(1, &mVbo);
    }

    QVector<float> vertices;
    foreach (const WayPoint& wpt, mWaypoints)
        vertices << wpt.x() << wpt.y() << wpt.z();

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, mWaypoints.size() * sizeof(QVector3D), (void*)vertices.constData(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
