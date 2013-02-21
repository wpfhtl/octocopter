#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QColor>
#include <QGLWidget>
#include <QVector>
#include <QVector3D>

#include <cuda_gl_interop.h>

#include "model.h"
#include "openglutilities.h"
#include "flightcontrollervalues.h"
#include "shaderprogram.h"

class PointCloud;

class FlightPlannerInterface;


class GlWidget : public QGLWidget
{
    Q_OBJECT

    QList<PointCloud*> mPointCloudsToRender;

    QVector3D mCamLookAtOffset;

    Model *mModelVehicle, *mModelThrust, *mModelConeYaw, *mModelConePitch, *mModelConeRoll, *mModelTarget;
    Model *mModelControllerP, *mModelControllerI, *mModelControllerD;

    const Pose* mLastKnownVehiclePose;


    // Set by slotSetFlightControllerValues(), then visualized for FligthController debugging
    const FlightControllerValues* mLastFlightControllerValues;

    // Timer
    QTime mTimeOfLastRender;
    QTimer* mTimerUpdate;

    float mRotationPerFrame;
    bool mViewRotating;
    bool mViewZooming;

    // Mouse Rotations
    QPoint      mLastMousePosition;
    QVector3D   mCameraPosition;

    // The components of this vector store the rotation (in degrees) of the camera around the origin
    QVector2D mCameraRotation;

    GLuint mVertexArrayObject;

    unsigned int mVboVehicle, mVboAxes;

    unsigned int mVboVehiclePathElementSize;
    unsigned int mVboVehiclePathBytesMaximum;
    unsigned int mVboVehiclePathBytesCurrent;
    GLuint mVboVehiclePath;

    GLuint mUboId;
    unsigned int mUboSize;

    quint32 mFramesRenderedThisSecond;

    ShaderProgram *mShaderProgramDefault;
    ShaderProgram *mShaderProgramPointCloud;
    ShaderProgram *mShaderProgramParticles; // for testing billboarding of the PointCloud

    // Wheel Zooming. For smooth zooming, mZoomFactorCurrent converges toward mZoomFactorTarget
    GLdouble    mZoomFactorTarget, mZoomFactorCurrent;

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void zoom(double zoomFactor);

    void renderController(const QMatrix4x4 &transform, const PidController* const controller);

public:
    GlWidget(QWidget* parent/*, FlightPlannerInterface* flightPlanner*/);
    void moveCamera(const QVector3D &pos);
    void keyPressEvent(QKeyEvent *event);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

signals:
    void initializingInGlContext();
    void visualizeNow();
    void rotating(bool);

public slots:
    // When this is called, we take note of the time of last external update. Because when zooming/rotating
    // is also active, the redraws caused by those actions overlap with the external redraws, causing
    // superfluous and slow redrawing. So, by only redrawing for rotation/zoom when there hasn't been an
    // external redraw in a while, we can save CPU cycles.
    void slotUpdateView();

    // GlWidget renders points from all known PointClouds. These methods (de)register PointClouds for rendering.
    // Ownership remains with the caller, meaning they MUST be deregistered before deletion
    void slotPointCloudRegister(PointCloud* p);
    void slotPointCloudUnregister(PointCloud* p);

    // LogPlayer and RoverConnection set values used/computed by the flightcontroller.
    // These shall be visualized here in GlWidget for debugging.
    void slotSetFlightControllerValues(const FlightControllerValues *const fcv);

    void slotNewVehiclePose(const Pose *const);
    void slotClearVehicleTrajectory();

    void slotEnableTimerRotation(const bool& enable);
    void slotViewFromTop();
    void slotViewFromSide();
    void slotSaveImage();
};

#endif
