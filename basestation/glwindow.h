#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QColor>
#include <QWindow>
#include <QVector>
#include <QVector3D>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include <cuda_gl_interop.h>

#include "model.h"
#include "rawscan.h"
#include "openglutilities.h"
#include "flightcontrollervalues.h"
#include "shaderprogram.h"

class PointCloud;

class FlightPlannerInterface;


class GlWindow : public QWindow, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT

    QOpenGLContext* mOpenGlContext;

    QList<PointCloud*> mPointCloudsToRender;

    QVector3D mCamLookAtOffset;

    Model *mModelVehicle, *mModelThrust, *mModelConeYaw, *mModelConePitch, *mModelConeRoll, *mModelHoverPosition, *mModelTrajectoryStart, *mModelTrajectoryGoal, *mModelVelocityArrow;
    Model *mModelControllerP, *mModelControllerI, *mModelControllerD;

    const Pose* mLastKnownVehiclePose;

    // Set by slotSetFlightControllerValues(), then visualized for FligthController debugging
    const FlightControllerValues* mLastFlightControllerValues;

    // Timer
    QTime mTimeOfLastRender;
    QTimer* mTimerUpdate;

    float mRotationPerFrame;
    bool mViewZooming;

    // Mouse Rotations
    QPoint      mLastMousePosition;
    QVector3D   mCameraPosition;

    // The components of this vector store the rotation (in degrees) of the camera around the origin
    QVector2D mCameraRotation;

    GLuint mVertexArrayObject;

    unsigned int mVboVehicle, mVboAxes, mVboRawScanRays;

    unsigned int mVboVehiclePathElementSize;
    unsigned int mVboVehiclePathBytesMaximum;
    unsigned int mVboVehiclePathBytesCurrent;
    GLuint mVboVehiclePath;

    QMatrix4x4 mRayVisualizationRelativeScannerMatrix;
    quint16 mRayVisualizationUsableDistanceIndexFirst, mRayVisualizationUsableDistanceIndexLast;

    GLuint mUboId;
    unsigned int mUboSize;

    quint32 mFramesRenderedThisSecond;

    ShaderProgram *mShaderProgramDefault;
    ShaderProgram *mShaderProgramPointCloud;
    ShaderProgram *mShaderProgramRawScanRays;

    // Wheel Zooming. For smooth zooming, mZoomFactorCurrent converges toward mZoomFactorTarget
    GLdouble    mZoomFactorTarget, mZoomFactorCurrent;

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void zoom(double zoomFactor);

    void renderController(const QMatrix4x4 &transform, const PidController* const controller);

    void resize();
    void exposeEvent(QExposeEvent *event);
    void resizeEvent(QResizeEvent *event);

public:
    GlWindow(QWindow *parent = 0/*, FlightPlannerInterface* flightPlanner*/);
    void moveCamera(const QVector3D &pos);
    //void keyPressEvent(QKeyEvent *event);
    bool isPointCloudRegistered(PointCloud* p);
    void reloadShaders();


    // Sometimes, mapping an opengl buffer to CUDA memory fails. The only reason I can think of is that the buffer
    // is currently used for rendering. Rendering should never be interrupted (allowing CUDA stuff to happen) by either
    // threading or because I screwed up using QCoreApplication::processEvents(). To check this isn't the case, this
    // member indicates whether we're currently rendering. In other parts of the code, we can then ASSERT on this. Dirrrty!
    bool mIsCurrentlyRendering;


    // Shouldn't be public, I know....
    bool mRenderAxisBase, mRenderAxisVehicle, mRenderTrajectory, mRenderVehicle, mRenderRawScanRays;
    // points scanned from further distance than this shouldn't be rendered by the shader!
    float mMaxPointVisualizationDistance;
    float mBackgroundBrightness;
    float mPointCloudPointSize;
    float mPointCloudPointAlpha;
    float mPointCloudColorLow, mPointCloudColorHigh;

signals:
    void initializingInGlContext();
    void visualizeNow();
    void showOnlyPointsScannedFromLessThan(float);
    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    // This will init opengl and emit initializingInGlContext()
    void slotInitialize();

    void slotSetCameraRotation(const float rotation);

    // When this is called, we take note of the time of last external update. Because when zooming/rotating
    // is also active, the redraws caused by those actions overlap with the external redraws, causing
    // superfluous and slow redrawing. So, by only redrawing for rotation/zoom when there hasn't been an
    // external redraw in a while, we can save CPU cycles.
    void slotRenderLater();
    void slotRenderNow();

    // GlWidget renders points from all known PointClouds. These methods (de)register PointClouds for rendering.
    // Ownership remains with the caller, meaning they MUST be deregistered before deletion
    void slotPointCloudRegister(PointCloud* p);
    void slotPointCloudUnregister(PointCloud* p);

    // LogPlayer and RoverConnection set values used/computed by the flightcontroller.
    // These shall be visualized here in GlWidget for debugging.
    void slotSetFlightControllerValues(const FlightControllerValues *const fcv);

    // Called from logplayer, for debugging unfused laser data. Copy it to OpenGL-Buffer if you need it.
    void slotNewRawScan(const RawScan * const rawScan);

    void slotNewVehiclePose(const Pose *const);
    void slotClearVehicleTrajectory();

    void slotViewFromTop();
    void slotViewFromSide();
};

#endif
