#ifndef GLWIDGET_H
#define GLWIDGET_H


#include <QtGui>
#include <QColor>
#include <QGLWidget>
#include <QVector>
#include <QVector3D>

#include "model.h"

#include <cuda_gl_interop.h>

#include "flightplannerinterface.h"
#include "openglutilities.h"
#include "shaderprogram.h"
#include "pose.h"

class Octree;

class FlightPlannerInterface;


class GlWidget : public QGLWidget
{
    Q_OBJECT

    QList<Octree*> mOctrees;
    FlightPlannerInterface *mFlightPlanner;

    QVector3D mCamLookAtOffset;

    Model *mModelVehicle;

    Pose mLastKnownVehiclePose;

    // Timer
    int mTimerIdZoom, mTimerIdRotate;
    QDateTime mTimeOfLastExternalUpdate;

    // Mouse Rotations
    QPoint      mLastMousePosition;
    QVector3D   mCameraPosition;
    GLfloat     rotX, rotY, rotZ;

    GLuint mVertexArrayObject;

    unsigned int mVboVehicle, mVboAxes;

    unsigned int mVboVehiclePathElementSize;
    unsigned int mVboVehiclePathBytesMaximum;
    unsigned int mVboVehiclePathBytesCurrent;
    GLuint mVboVehiclePath;

    GLuint mUboId;
    unsigned int mUboSize;

    ShaderProgram *mShaderProgramDefault;
    ShaderProgram *mShaderProgramParticles; // for testing billboarding of the octree

    // Wheel Zooming. For smooth zooming, mZoomFactorCurrent converges toward mZoomFactorTarget
    GLdouble    mZoomFactorTarget, mZoomFactorCurrent;

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void zoom(double zoomFactor);
    QVector3D convertMouseToWorldPosition(const QPoint&);

public:
    GlWidget(QWidget* parent, FlightPlannerInterface* flightPlanner);
    void moveCamera(const QVector3D &pos);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void timerEvent ( QTimerEvent * event );

signals:
    void initializingInGlContext();
    void visualizeNow();
    void mouseClickedAtWorldPos(Qt::MouseButton, QVector3D);

public slots:
    // When this is called, we take note of the time of last external update. Because when zooming/rotating
    // is also active, the redraws caused by those actions overlap with the external redraws, causing
    // superfluous and slow redrawing. So, by only redrawing for rotation/zoom when there hasn't been an
    // external redraw in a while, we can save CPU cycles.
    void slotUpdateView();

    // GlWidget renders points from all known octrees. These methods (de)register octrees for rendering.
    // Ownership remains with the caller, meaning they MUST be deregistered before deletion
    void slotOctreeRegister(Octree* o);
    void slotOctreeUnregister(Octree* o);

    void slotNewVehiclePose(Pose);

    void slotEnableTimerRotation(const bool& enable);
    void slotViewFromTop();
    void slotViewFromSide();
    void slotSaveImage();
};

#endif
