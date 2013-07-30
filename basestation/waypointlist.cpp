#include "waypointlist.h"
#include <QFile>
#include <QDebug>
#include <QString>
#include <QTextStream>

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

void WayPointList::prepend(const WayPoint& wp) {mWaypoints.prepend(wp); updateVbo();}
void WayPointList::append(const WayPoint& wp) {mWaypoints.append(wp); updateVbo();}
void WayPointList::append(const QList<WayPoint>& wps) {mWaypoints.append(wps); updateVbo();}
void WayPointList::append(const WayPointList* wpl) {mWaypoints.append(wpl->mWaypoints); updateVbo();}
void WayPointList::remove(const quint16& index) {mWaypoints.removeAt(index); updateVbo();}
void WayPointList::clear() {mWaypoints.clear(); updateVbo();}
void WayPointList::insert(const quint16& index, const WayPoint& wp) {mWaypoints.insert(index, wp); updateVbo();}

WayPoint WayPointList::takeAt(const int index)
{
    const WayPoint wpt = mWaypoints.takeAt(index);
    updateVbo();
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
                    // We have a collision, need to merge w1 and w2.
                    if(w1.informationGain > w2.informationGain)
                    {
                        mWaypoints.removeAt(j);
                        j--;
                    }
                    else
                    {
                        mWaypoints.removeAt(i);
                        i--;
                        break; // out of the inner for loop, so we can check the same i again.
                    }
                }
            }
        }
    }

    updateVbo();
}

QString WayPointList::toString() const
{
    QString result = QString("WayPointList with %1 elements:\n").arg(mWaypoints.size());

    for(int i=0;i<mWaypoints.size();i++)
        result.append(QString("%1: %2\n").arg(i, 2).arg(mWaypoints.at(i).toString()));

    return result;
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

    updateVbo();

//    float distanceAfter = 0;
//    for(int i=1;i<mWaypoints.size();i++) distanceAfter += mWaypoints.at(i-1).distanceToLine(mWaypoints.at(i), QVector3D());
    //    qDebug() << "FlightPlannerInterface::sortToShortestPath(): total distance between" << wayPoints.size() << "points after:" << distanceAfter;
}

void WayPointList::setList(const QList<WayPoint>* const wayPointList)
{
    mWaypoints = *wayPointList;
    updateVbo();
}

// Copy all waypoints into our VBO
void WayPointList::updateVbo()
{
    if(!mVbo)
    {
        initializeOpenGLFunctions();
        glGenBuffers(1, &mVbo);
    }

    QVector<float> vertices;
    // We reserve space for x,y,z,w (where w is always 1.0) and information gain. Points with informatio gain > 0.0 are
    // for SCANning, others must be DETOUR
    vertices.reserve(mWaypoints.size() * 5);
    foreach(const WayPoint& wpt, mWaypoints)
    {
        //vertices << wpt.x() << wpt.y() << wpt.z() << (wpt.purpose == WayPoint::Purpose::SCAN ? 1.0f : 0.5f);
        vertices << wpt.x() << wpt.y() << wpt.z() << 1.0f << wpt.informationGain;
    }

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, mWaypoints.size() * 5 * sizeof(float), (void*)vertices.constData(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool WayPointList::loadFromFile(const QString& fileName)
{
    if(fileName.isNull())
        return false;

    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    while(!file.atEnd())
    {
        const QString line(file.readLine());
        const QStringList values = line.split(";", QString::SkipEmptyParts);

        if(values.size() != 3)
        {
            qDebug() << __PRETTY_FUNCTION__ << "File Format Error, not three numbers in line!";
            return false;
        }

        const WayPoint wpt(
                    QVector3D(
                        values.at(0).toFloat(),
                        values.at(1).toFloat(),
                        values.at(2).toFloat()
                        )
                    );

        mWaypoints.append(wpt);
    }

    file.close();
    updateVbo();
    return true;
}

bool WayPointList::saveToFile(const QString& fileName) const
{
    if(fileName.isNull())
        return false;

    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;

    QTextStream out(&file);

    for(int i=0;i<mWaypoints.size();i++)
    {
        out << mWaypoints.at(i).x() << ";";
        out << mWaypoints.at(i).y() << ";";
        out << mWaypoints.at(i).z();
        out << "\n";
    }

    file.close();
    return true;
}
