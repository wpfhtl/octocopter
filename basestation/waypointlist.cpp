#include "waypointlist.h"
#include <QFile>
#include <QDebug>
#include <QString>
#include <QTime>
#include <QTextStream>

#include <tsp/graph.h>

WayPointList::WayPointList() : QObject(), OPENGL_FUNCTIONS_CLASS()
{
    mVbo = 0;
    // We set this to true by default, so that on the first call to vbo() (which is
    // done in GlScene with an empty list), a VBO is created. This is required for
    // correct initialization of the VAO.
    mVboDirty = true;
}

WayPointList::WayPointList(const QColor& color) : QObject(), OPENGL_FUNCTIONS_CLASS()
{
    mVbo = 0;
    mVboDirty = true;
    mColor = color;
    mWaypoints.clear();
}

WayPointList::WayPointList(const WayPointList& other) : QObject(), OPENGL_FUNCTIONS_CLASS()
{
    mVbo = 0; // don't clone someone else's vbo, create your own!
    mVboDirty = true;
    mColor = other.mColor;
    mWaypoints = other.mWaypoints;
}

WayPointList::~WayPointList()
{
    if(mVbo) glDeleteBuffers(1, &mVbo);
}

void WayPointList::prepend(const WayPoint& wp) {mWaypoints.prepend(wp); mVboDirty = true;}
void WayPointList::append(const WayPoint& wp) {mWaypoints.append(wp); mVboDirty = true;}
void WayPointList::append(const QList<WayPoint>& wps) {mWaypoints.append(wps); mVboDirty = true;}
void WayPointList::append(const WayPointList* wpl) {mWaypoints.append(wpl->mWaypoints); mVboDirty = true;}
void WayPointList::remove(const quint16& index) {mWaypoints.removeAt(index); mVboDirty = true;}
void WayPointList::clear() {mWaypoints.clear(); mVboDirty = true;}
void WayPointList::insert(const quint16& index, const WayPoint& wp) {mWaypoints.insert(index, wp); mVboDirty = true;}

WayPoint WayPointList::takeAt(const int index)
{
    const WayPoint wpt = mWaypoints.takeAt(index);
    mVboDirty = true;
    return wpt;
}

void WayPointList::setHeight(const float height)
{
    for(int i=0;i<mWaypoints.size();i++)
    {
        mWaypoints[i].setY(height);
    }
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

    mVboDirty = true;
}

QString WayPointList::toString() const
{
    QString result = QString("WayPointList with %1 elements:\n").arg(mWaypoints.size());

    for(int i=0;i<mWaypoints.size();i++)
        result.append(QString("wpt %1, gain %2, %3 %4 %5\n").arg(i, 2).arg(mWaypoints[i].informationGain, 2).arg(mWaypoints[i].x(), 7).arg(mWaypoints[i].y(), 7).arg(mWaypoints[i].z(), 7));

    return result;
}

// Finds the shortest Hamiltonian cycle starting and ending at vehiclePosition.
// INstead, we want:
// Finds the shortest Hamiltonian cycle starting /*and ending*/ at vehiclePosition.
void WayPointList::sortToShortestPath(const QVector3D &vehiclePosition)
{
    qDebug() << "WayPointList::sortToShortestPath(): vehicle is at" << vehiclePosition << "waypointlist contains" << mWaypoints.size() << "waypoints";

    if(mWaypoints.size() == 0)
    {
        qDebug() << "WayPointList::sortToShortestPath(): 0 waypoints, returning.";
        return;
    }

    QTime t;t.start();

    Graph g;

    QList<WayPoint> wpl(mWaypoints);
    wpl.prepend(vehiclePosition);
    g.setWayPointList(&wpl);

    Path p = g.optTSP();

    mWaypoints.clear();

    // Do not use the first point, it's at the vehicle's position!
    qDebug() << "WayPointList::sortToShortestPath(): ignoring first point:" << wpl.at(p.vertices[0]);
    for(int i=1;i<p.vertices.size();i++)
        mWaypoints.append(wpl.at(p.vertices[i]));

    qDebug() << "WayPointList::sortToShortestPath(): TSP done after" << t.elapsed() << "ms - waypointlist contains" << mWaypoints.size() << "waypoints:\n" << toString();

    // The code above gives us the shortest hamiltonian cycle starting and ending at vehiclePosition. We ignore the last
    // waypoint (which isn't given back above anyway) as we don't want to come back to the vehicle's starting position
    // after traversing all waypoints. Imagine waypoints being laid out in a U-shape, with the vehicle-poition being the
    // right dot of an Ü. Since the code above computes a cycle through all waypoints, it doesn't matter if the vehicle
    // drives down the left or right wing of the U. Because we don't use the whole cycle, but only parts of it, it's
    // important that we fly down the right wing of the U, leading to a much shorter path.
    // In short, check to see whether it's better to go through the path backwards!
    const float distForward = getDistance(&mWaypoints, vehiclePosition, WayPointList::TravelDirection::TravelDirectionForward);
    const float distBackward = getDistance(&mWaypoints, vehiclePosition, WayPointList::TravelDirection::TravelDirectionBackward);
    if(distBackward < distForward)
    {
        reverseWayPoints();
        qDebug() << "after reversion, wpl looks like this:\n" << toString();
    }

    QList<WayPoint> wplLastBecomesFirst = mWaypoints;
    wplLastBecomesFirst.prepend(wplLastBecomesFirst.takeLast());
    const float distNormal = getDistance(&mWaypoints, vehiclePosition, WayPointList::TravelDirection::TravelDirectionForward);
    const float distLastBecomesFirst = getDistance(&wplLastBecomesFirst, vehiclePosition, WayPointList::TravelDirection::TravelDirectionForward);
    if(distLastBecomesFirst < distNormal)
    {
        qDebug() << "putting last wpt first saves" << (distNormal - distLastBecomesFirst);
        mWaypoints = wplLastBecomesFirst;
    }
    else
    {
        qDebug() << "putting last wpt first adds length:" << -(distNormal - distLastBecomesFirst);
    }

    mVboDirty = true;
}

void WayPointList::setList(const QList<WayPoint>* const wayPointList)
{
    mWaypoints = *wayPointList;
    mVboDirty = true;
}

void WayPointList::reverseWayPoints()
{
    qDebug() << __PRETTY_FUNCTION__;

    for(int k=0; k<(mWaypoints.size()/2); k++)
        mWaypoints.swap(k,mWaypoints.size()-(1+k));

    mVboDirty = true;
}

float WayPointList::getDistance(const QList<WayPoint>* const wpl, const QVector3D &startingFrom, const WayPointList::TravelDirection& direction)
{
    QString debugString("distance of going %1 through %2...(%3)...%4 is %5");
    debugString = debugString.arg(direction == WayPointList::TravelDirection::TravelDirectionForward ? "forward" : "backward");

    float distance = 0.0f;

    if(direction == WayPointList::TravelDirection::TravelDirectionForward)
    {
        if(wpl->size())
        {
            debugString = debugString.arg(wpl->first().toString());
            debugString = debugString.arg(wpl->size()-2);
            debugString = debugString.arg(wpl->last().toString());
        }

        for(int i=0;i<wpl->size();i++)
        {
            // On the first iteration, add distance from @startingFrom to first waypoint
            if(i == 0) distance += startingFrom.distanceToLine(wpl->at(i), QVector3D());

            // If there is a next waypoint, add the distance from current to next
            if(i+1 < wpl->size()) distance += wpl->at(i).distanceToLine(wpl->at(i+1), QVector3D());
        }
    }
    else if(direction == WayPointList::TravelDirection::TravelDirectionBackward)
    {
        if(wpl->size())
        {
            debugString = debugString.arg(wpl->last().toString());
            debugString = debugString.arg(wpl->size()-2);
            debugString = debugString.arg(wpl->first().toString());
        }

        for(int i=wpl->size()-1;i>0;i--)
        {
            // On the first iteration, add distance from @startingFrom to first waypoint
            if(i == wpl->size()-1) distance += startingFrom.distanceToLine(wpl->at(i), QVector3D());

            // If there is a next waypoint, add the distance from current to next
            if(i-1 >= 0) distance += wpl->at(i).distanceToLine(wpl->at(i-1), QVector3D());
        }
    }

    qDebug() << "WayPointList::getDistance()" << debugString.arg(distance);
    return distance;
}

// Copy all waypoints into our VBO
void WayPointList::updateVbo()
{
    if(!mVbo)
    {
        if(!initializeOpenGLFunctions()) qFatal("couldn't init opengl functions!");
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

    mVboDirty = false;
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
    mVboDirty = true;
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

const quint32 WayPointList::vbo()
{
    if(mVboDirty)
        updateVbo();

    return mVbo;
}
