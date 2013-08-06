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
    mVboDirty = false;
}

WayPointList::WayPointList(const QColor& color) : QObject(), OPENGL_FUNCTIONS_CLASS()
{
    mVbo = 0;
    mVboDirty = false;
    mColor = color;
    mWaypoints.clear();
}

WayPointList::WayPointList(const WayPointList& other) : QObject(), OPENGL_FUNCTIONS_CLASS()
{
    mVbo = 0; // don't clone someone else's vbo, create your own!
    mVboDirty = false;
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

void WayPointList::sortToShortestPath(const QVector3D &vehiclePosition)
{
    qDebug() << "FlightPlannerInterface::sortToShortestPath(): vehicle is at" << vehiclePosition << "waypointlist contains" << mWaypoints.size() << "waypoints";

    QTime t;t.start();

    Graph g;

    QList<WayPoint> wpl(mWaypoints);
    wpl.prepend(vehiclePosition);
    g.setWayPointList(&wpl);

    Path p = g.optTSP();

    mWaypoints.clear();

    // Do not use the first and the last point, they're both at the vehicle's position!
    for(int i=1;i<p.vertices.size();i++)
        mWaypoints.append(wpl.at(p.vertices[i]));

    mVboDirty = true;

    qDebug() << "FlightPlannerInterface::sortToShortestPath(): done after" << t.elapsed() << "ms - waypointlist contains" << mWaypoints.size() << "waypoints";
}

void WayPointList::setList(const QList<WayPoint>* const wayPointList)
{
    mWaypoints = *wayPointList;
    mVboDirty = true;
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
