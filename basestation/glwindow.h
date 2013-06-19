#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QColor>
#include <QWindow>
#include <QVector>
#include <QVector3D>
#include <QOpenGLFunctions_4_3_Core>

#include <cuda_gl_interop.h>

#include "model.h"
#include "openglutilities.h"
#include "flightcontrollervalues.h"
#include "shaderprogram.h"

class PointCloud;

class FlightPlannerInterface;

class GlWindow : public QWindow, protected QOpenGLFunctions_4_3_Core
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

    // points scanned from further distance than this shouldn't be rendered by the shader!
    float mMaxPointVisualizationDistance;
    bool mBackgroundDarkOrBright;

    float mRotationPerFrame;
    bool mViewRotating;
    bool mViewZooming;
    bool mRenderAxisBase, mRenderAxisVehicle, mRenderTrajectory, mRenderVehicle, mRenderRawScanRays;

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
    void keyPressEvent(QKeyEvent *event);
    bool isPointCloudRegistered(PointCloud* p);

signals:
    void initializingInGlContext();
    void visualizeNow();
    void rotating(bool);
    void showOnlyPointsScannedFromLessThan(float);
    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    // This will init opengl and emit initializingInGlContext()
    void slotInitialize();

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
    void slotNewRayData(const Pose* const relativeScannerPose, const qint32& timestampScanScanner, std::vector<quint16> * const distances);

    void slotNewVehiclePose(const Pose *const);
    void slotClearVehicleTrajectory();

    void slotEnableTimerRotation(const bool& enable);
    void slotViewFromTop();
    void slotViewFromSide();
};

#endif
