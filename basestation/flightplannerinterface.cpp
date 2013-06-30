#include "glwindow.h"
#include "flightplannerinterface.h"

FlightPlannerInterface::FlightPlannerInterface(QWidget* widget, GlWindow* glWidget, PointCloud *pointcloud) : QObject()
{
    mGlWidget = glWidget;
    mParentWidget = widget;
    mShaderProgramDefault = mShaderProgramSpheres = 0;
    mVboBoundingBox = 0;
    mVboWayPointConnections = 0;
    mShowWayPoints = true;
    mShowBoundingBox = true;

    mVehiclePoses.reserve(25 * 60 * 20); // enough poses for 20 minutes with 25Hz

    mWaypointListMap.insert("ahead", new WayPointList(QColor(255,0,0,200)));
    mWaypointListMap.insert("passed", new WayPointList(QColor(255,255,0,200)));

    mPointCloudDense = pointcloud;

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

    OpenGlUtilities::setVboToBoundingBox(mVboBoundingBox, mScanVolumeMin, mScanVolumeMax);
}


void FlightPlannerInterface::slotClearVehicleTrajectory()
{
    mVehiclePoses.clear();
    if(mGlWidget) mGlWidget->slotClearVehicleTrajectory();

    emit suggestVisualization();
}

const Pose FlightPlannerInterface::getLastKnownVehiclePose(void) const
{
    if(mVehiclePoses.size())
        return mVehiclePoses.last();
    else
        return Pose();
}

void FlightPlannerInterface::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_B && !(event->modifiers() & Qt::ShiftModifier))
    {
        qDebug() << "FlightPlannerInterface::keyPressEvent(): b, toggling bounding box visualization";
        mShowBoundingBox = !mShowBoundingBox;
    }
}

void FlightPlannerInterface::slotVehiclePoseChanged(const Pose* const pose)
{
    mVehiclePoses.append(*pose);
    //    if(mVehiclePoses.size() > 2) mVehiclePoses.takeFirst();
}
/*
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
}*/

/*
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
}*/

void FlightPlannerInterface::slotSetWayPoints(const QList<WayPoint>* const wayPointList, const WayPointListSource source)
{
    WayPointList* wpl = mWaypointListMap["ahead"];
    wpl->setList(wayPointList);

    // Tell others about our new waypoints!
    emit wayPoints(mWaypointListMap["ahead"]->list(), source);
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
    emit wayPoints(mWaypointListMap["ahead"]->list(), WayPointListSource::WayPointListSourceFlightPlanner);
    qDebug() << "FlightPlannerInterface::slotWayPointReached(): rover->baseconnection->flightplanner waypoint reached, emitted wayPointDeleted(0)";
    emit suggestVisualization();
}

const QList<WayPoint>* const FlightPlannerInterface::getWayPoints()
{
    return mWaypointListMap.value("ahead")->list();
}

//void FlightPlannerInterface::getScanVolume(QVector3D& min, QVector3D& max)
//{
//    min = mScanVolumeMin;
//    max = mScanVolumeMax;
//}

void FlightPlannerInterface::slotVisualize()
{
    // Bounding Box
    // Initialize shaders and VBO if necessary
    if(mShaderProgramDefault == 0 && mGlWidget != 0)
    {
        qDebug() << __PRETTY_FUNCTION__ << "initializing opengl...";
        initializeOpenGLFunctions();
        mShaderProgramDefault = new ShaderProgram(this, "shader-default-vertex.c", "", "shader-default-fragment.c");
    }

    if(mVboBoundingBox == 0)
    {
        glGenBuffers(1, &mVboBoundingBox);
        OpenGlUtilities::setVboToBoundingBox(mVboBoundingBox, mScanVolumeMin, mScanVolumeMax);
    }

    if(mShowBoundingBox && mShaderProgramDefault != 0)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            glBindBuffer(GL_ARRAY_BUFFER, mVboBoundingBox);
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


    // Waypoints

    // Initialize shaders and VBO if necessary
    /*
    if(mShaderProgramSpheres == 0 && mGlWidget != 0)
    {
        mShaderProgramSpheres = new ShaderProgram(this, "shader-particles-vertex.c", "shader-particles-geometry.c", "shader-particles-fragment.c");
    }

    if(mShowWayPoints && mShaderProgramSpheres != 0)
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
                glUniform1f(glGetUniformLocation(mShaderProgramSpheres->programId(), "particleRadius"), wpl->getSphereSize());

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
    }*/

    if(mShowWayPoints && mShaderProgramDefault != 0)
    {
        mShaderProgramDefault->bind();
        mShaderProgramDefault->setUniformValue("useFixedColor", true);

        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Beau.Ti.Ful!
        {
            QMapIterator<QString, WayPointList*> i(mWaypointListMap);
            while (i.hasNext()) {
                i.next();
                WayPointList *wpl = i.value();

                if(wpl->list()->size() == 0)
                {
                    //qDebug() << __PRETTY_FUNCTION__ << "waypointlist" << i.key() << "is empty, not drawing...";
                    continue;
                }

                mShaderProgramDefault->setUniformValue("fixedColor",
                                                       QVector4D(
                                                           wpl->color().redF(),
                                                           wpl->color().greenF(),
                                                           wpl->color().blueF(),
                                                           wpl->color().alphaF()
                                                           )
                                                       );

                glBindBuffer(GL_ARRAY_BUFFER, wpl->vbo());
                // Make the contents of this array available at layout position vertexShaderVertexIndex in the vertex shader
                Q_ASSERT(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position") != -1);
                glEnableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
                glVertexAttribPointer(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"), 3, GL_FLOAT, GL_FALSE, 0, 0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                // Draw using shaders
                glDrawArrays(GL_LINE_STRIP, 0, wpl->list()->size());

                glDisableVertexAttribArray(glGetAttribLocation(mShaderProgramDefault->programId(), "in_position"));
            }
        }
        glDisable(GL_BLEND);
        mShaderProgramDefault->release();
    }

}

