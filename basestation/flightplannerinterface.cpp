#include <GL/glew.h>
#include "glwidget.h"
#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(QWidget* widget, Octree* pointCloud) : QObject()
{
    mOctree = pointCloud;
    mGlWidget = 0;
    mParentWidget = widget;
    mShaderProgramDefault = mShaderProgramSpheres = 0;
    mBoundingBoxVbo = 0;

    mVehiclePoses.reserve(25 * 60 * 20); // enough poses for 20 minutes

    mWaypointListMap.insert("ahead", new WayPointList(QColor(255,0,0,200)));
    mWaypointListMap.insert("passed", new WayPointList(QColor(255,255,0,200)));

    qDebug() << "FlightPlannerInterface c'tor.";
}

FlightPlannerInterface::~FlightPlannerInterface()
{
    qDeleteAll(mWaypointListMap);
}

void FlightPlannerInterface::slotSetScanVolume(const QVector3D minBox, const QVector3D maxBox)
{
    mScanVolumeMin = minBox;
    mScanVolumeMax = maxBox;

    setVboBoundingBox();
}

bool FlightPlannerInterface::insertPointsFromNode(const Node* node)
{
    if(node->isLeaf())
    {
        for(int i=0;i<node->pointIndices.size();i++)
            insertPoint(new LidarPoint(node->mTree->data()->at(node->pointIndices.at(i))));
    }
    else
    {
        // invoke recursively for childnodes/leafs
        const QList<const Node*> childNodes = node->getAllChildLeafs();
        foreach(const Node* childNode, childNodes)
            if(!insertPointsFromNode(childNode))
                return false;
    }

    return true;
}

void FlightPlannerInterface::slotClearVehicleTrajectory()
{
    mVehiclePoses.clear();
    if(mGlWidget) mGlWidget->slotClearVehicleTrajectory();

    emit suggestVisualization();
}

void FlightPlannerInterface::slotCheckWayPointsHashFromRover(const QString &hash)
{
    if(WayPoint::hash(getWayPoints()) != hash)
    {
        emit message(
                    Warning,
                    QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                    QString("Waypoints hash from rover does not match our hash, resending list"));

        emit wayPointsSetOnRover(getWayPoints());
    }
}


const Pose FlightPlannerInterface::getLastKnownVehiclePose(void) const
{
    if(mVehiclePoses.size())
        return mVehiclePoses.last();
    else
        return Pose();
}

void FlightPlannerInterface::slotVehiclePoseChanged(const Pose* const pose)
{
    mVehiclePoses.append(*pose);
    //    if(mVehiclePoses.size() > 2) mVehiclePoses.takeFirst();
}

const QVector3D FlightPlannerInterface::getCurrentVehicleVelocity() const
{
    if(mVehiclePoses.size() < 2) return QVector3D();

    const Pose& last = mVehiclePoses.last();
    const Pose& secondLast = mVehiclePoses.at(mVehiclePoses.size() - 2);

    const quint32 timeDiffMs = last.timestamp - secondLast.timestamp;

    if(timeDiffMs == 0)
        return QVector3D();
    else
        return (last.getPosition() - secondLast.getPosition()) * (1000 / timeDiffMs);
}

void FlightPlannerInterface::slotWayPointDelete(const quint16& index)
{
    if(mWaypointListMap.value("ahead")->size() <= index)
    {
        qWarning() << "FlightPlannerInterface::slotWayPointDelete(): cannot delete waypoint at index" << index << ", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }

    mWaypointListMap["ahead"]->remove(index);
    emit wayPointDeleted(index);
    emit wayPointDeleteOnRover(index);
    emit suggestVisualization();
}

// Called when the UI inserted a WPT. Tell the rover!
void FlightPlannerInterface::slotWayPointInsert(const quint16& index, const WayPoint& wpt)
{
    if(index > mWaypointListMap.value("ahead")->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointInsert(): cannot delete waypoint at index" << index << ", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }

    mWaypointListMap["ahead"]->insert(index, wpt);
    emit wayPointInserted(index, wpt);
    emit wayPointInsertOnRover(index, wpt);
    emit suggestVisualization();
}

// Called when rover inserted a wpt. DO NOT TELL ROVER to insert that same wpt again!
void FlightPlannerInterface::slotWayPointInsertedByRover(const quint16& index, const WayPoint& wpt)
{
    if(index > mWaypointListMap.value("ahead")->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointInsertedByRover(): cannot delete waypoint at index" << index << ", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Waypoint appended by rover: %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));

    mWaypointListMap["ahead"]->insert(index, wpt);
    emit wayPointInserted(index, wpt);
    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointSwap(const quint16& i, const quint16& j)
{
    if(mWaypointListMap.value("ahead")->size() <= i || mWaypointListMap.value("ahead")->size() <= j)
    {
        qWarning() << "FlightPlannerInterface::slotWayPointSwap(): cannot swap waypoints at index" << i << "and" << j <<", size is only" << mWaypointListMap.value("ahead")->size();
        return;
    }
    qDebug() << "FlightPlannerInterface::slotWayPointSwap(): swapping waypoints at index" << i << "and" << j <<", size is" << mWaypointListMap.value("ahead")->size();

    mWaypointListMap["ahead"]->swap(i,j);

    emit wayPointDeleted(j);
    emit wayPointDeleteOnRover(j);
    emit wayPointInserted(j, mWaypointListMap["ahead"]->at(j));
    emit wayPointInsertOnRover(j, mWaypointListMap["ahead"]->at(j));
    emit wayPointDeleted(i);
    emit wayPointDeleteOnRover(i);
    emit wayPointInserted(i, mWaypointListMap["ahead"]->at(i));
    emit wayPointInsertOnRover(i, mWaypointListMap["ahead"]->at(i));

    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointsClear()
{
    mWaypointListMap["ahead"]->clear();
    emit wayPointsSetOnRover(mWaypointListMap["ahead"]->list());
    emit wayPoints(mWaypointListMap["ahead"]->list());
    emit suggestVisualization();
}

void FlightPlannerInterface::slotWayPointReached(const WayPoint& wpt)
{
    qDebug() << "FlightPlannerInterface::slotWayPointReached(): rover->baseconnection->flightplanner waypoint reached, so appending first element of mWayPointsAhead to mWayPointsPassed";

    if(!mWaypointListMap.value("ahead")->size())
    {
        qWarning() << "FlightPlannerInterface::slotWayPointReached(): mWayPointsAhead is empty, how can you reach a waypoint?";
        return;
    }

    emit message(
                Information,
                QString("%1::%2(): ").arg(metaObject()->className()).arg(__FUNCTION__),
                QString("Reached waypoint %1 %2 %3").arg(wpt.x()).arg(wpt.y()).arg(wpt.z()));

    mWaypointListMap["passed"]->append(mWaypointListMap["ahead"]->takeAt(0));
    emit wayPointDeleted(0);
    qDebug() << "FlightPlannerInterface::slotWayPointReached(): rover->baseconnection->flightplanner waypoint reached, emitted wayPointDeleted(0)";
    emit suggestVisualization();
}

const QList<WayPoint>* const FlightPlannerInterface::getWayPoints()
{
    return mWaypointListMap.value("ahead")->list();
    //return *mWayPointsAhead;
}

void FlightPlannerInterface::getScanVolume(QVector3D& min, QVector3D& max)
{
    min = mScanVolumeMin;
    max = mScanVolumeMax;
}

void FlightPlannerInterface::setVboBoundingBox()
{
    if(mBoundingBoxVbo == 0)
    {
        qDebug() << "FlightPlannerInterface::setVbo(): VBO for bounding box still undefined, returning.";
        return;
    }

    mBoundingBoxVertices.clear();

    // Fill the vertices buffer with vertices for quads and lines
    mBoundingBoxVertices
            // 1 back
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f

            // 2 front
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f

            // 3 left
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f

            // 4 right
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f

            // 6 top
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMax.y() << mScanVolumeMax.z() << 1.0f

            // 5 bottom
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMax.z() << 1.0f
            << mScanVolumeMax.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f
            << mScanVolumeMin.x() << mScanVolumeMin.y() << mScanVolumeMin.z() << 1.0f;

    glBindBuffer(GL_ARRAY_BUFFER, mBoundingBoxVbo);

//    qDebug() << "FlightPlannerInterface::setVboBoundingBox(): reserving" << sizeof(float) * mBoundingBoxVertices.size() << "bytes in VBO...";
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * mBoundingBoxVertices.size(), NULL, GL_STATIC_DRAW);

//    qDebug() << "FlightPlannerInterface::setVboBoundingBox(): copying" << mBoundingBoxVertices.size() * sizeof(float) << "bytes of vertices into VBO...";
    glBufferSubData(
                GL_ARRAY_BUFFER,
                0, // offset in the VBO
                mBoundingBoxVertices.size() * sizeof(float), // how many bytes to store?
                (void*)(mBoundingBoxVertices.constData()) // data to store
                );

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void FlightPlannerInterface::slotVisualize()
{
    // Bounding Box
    // Initialize shaders and VBO if necessary
    if(mShaderProgramDefault == 0 && mGlWidget != 0)
    {
        mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
        glGenBuffers(1, &mBoundingBoxVbo);
        setVboBoundingBox();
    }

    if(mShaderProgramDefault != 0)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            glBindBuffer(GL_ARRAY_BUFFER, mBoundingBoxVbo);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0); // position

            // draw the lines around the box
            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(0.2f, 0.2f, 1.0f, 0.8f));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawArrays(GL_LINE_LOOP, 4, 4);
            glDrawArrays(GL_LINE_LOOP, 8, 4);
            glDrawArrays(GL_LINE_LOOP, 12, 4);
            glDrawArrays(GL_LINE_LOOP, 16, 4);
            glDrawArrays(GL_LINE_LOOP, 20, 4);

            // draw a half-transparent box
//            mShaderProgramDefault->setUniformValue("fixedColor", QVector4D(1.0f, 1.0f, 1.0f, 0.015f));
//            glDrawArrays(GL_QUADS, 0, 24);

            glDisableVertexAttribArray(0);
        }
        glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

    // Waypoint Lists
    // Initialize shaders and VBO if necessary
    if(mShaderProgramSpheres == 0 && mGlWidget != 0)
    {
        mShaderProgramSpheres = new ShaderProgram(this, "shader-particles-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");
    }

    if(mShaderProgramSpheres != 0)
    {
        mShaderProgramSpheres->bind();
        mShaderProgramSpheres->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            QMapIterator<QString, WayPointList*> i(mWaypointListMap);
            while (i.hasNext()) {
                i.next();
                WayPointList *wpl = i.value();

                mShaderProgramSpheres->setUniformValue("fixedColor",
                                                       QVector4D(
                                                           wpl->color().redF(),
                                                           wpl->color().greenF(),
                                                           wpl->color().blueF(),
                                                           wpl->color().alphaF()
                                                           )
                                                       );

                // Set particleRadius variable in the shader program
                Q_ASSERT(glGetUniformLocation(mShaderProgramSpheres->programId(), "particleRadius") != -1);
                glUniform1f(glGetUniformLocation(mShaderProgramSpheres->programId(), "particleRadius"), wpl->sphereSize);

                glBindBuffer(GL_ARRAY_BUFFER, wpl->vbo());
                // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
                Q_ASSERT(glGetAttribLocation(mShaderProgramSpheres->programId(), "in_position") != -1);
                glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramSpheres->programId(), "in_position"));
                glVertexAttribPointer(glGetAttribLocation(mShaderProgramSpheres->programId(), "in_position"), 3, GL_FLOAT, GL_FALSE, 0, 0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                // Draw using shaders
                glDrawArrays(GL_POINTS, 0, wpl->list()->size());

                glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramSpheres->programId(), "in_position"));
            }
        }
        glDisable(GL_BLEND);
        mShaderProgramSpheres->release();
    }

    /* port to opengl4 core: draw line between future waypoints
    glLineWidth(1);
    glColor4f(1.0f, 1.0f, 0.0f, 0.8f);
    glBegin(GL_LINE_STRIP);

    if(mVehiclePoses.size())
    {
        const QVector3D p = mVehiclePoses.last().getPosition();
        glVertex3f(p.x(), p.y(), p.z());
    }

    foreach(const WayPoint& wpt, *mWayPointsAhead)
        glVertex3f(wpt.x(), wpt.y(), wpt.z());
    glEnd();
    */
}


void FlightPlannerInterface::slotDeleteGeneratedWayPoints()
{
    mWaypointListMap["generated"]->clear();
    emit suggestVisualization();
}

void FlightPlannerInterface::slotSubmitGeneratedWayPoints()
{
    mWaypointListMap["ahead"]->append(mWaypointListMap.value("generated"));
    mWaypointListMap["generated"]->clear();
    mWaypointListMap["ahead"]->sortToShortestPath(mVehiclePoses.last().getPosition());
    emit wayPointsSetOnRover(mWaypointListMap.value("ahead")->list());
    emit wayPoints(mWaypointListMap.value("ahead")->list());

    emit suggestVisualization();
}
