#ifndef GLSCENE_H
#define GLSCENE_H

#include <QObject>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

#include "pointcloud.h"
#include "waypointlist.h"
#include "model.h"
#include "rawscan.h"
#include "flightcontrollervalues.h"
#include "openglutilities.h"
#include "shaderprogram.h"
#include <common.h>
#include <gnssstatus.h>

class ProcessingState;
class FlightPlannerParticles;

class GlScene : public QObject, protected OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT
public:
    GlScene();
    ~GlScene();

    void render();

    bool isPointCloudRegistered(PointCloudCuda *p);
    void reloadShaders();

    void initialize();

    // Skip the setters and make this public, there's no harm
    bool
    mRenderAxisBase,
    mRenderAxisVehicle,
    mRenderTrajectory,
    mRenderVehicle,
    mRenderRawScanRays,
    mRenderParticles,
    mRenderInformationGain,
    mRenderBoundingBoxGlobal,
    mRenderBoundingBoxLocal,
    mRenderOccupancyGrid,
    mRenderPathPlannerGrid,
    mRenderWayPointsAhead,
    mRenderWayPointsPassed,
    mRenderSatelliteSignals;

    // points scanned from further distance than this shouldn't be rendered by the shader!
    float mMaxPointVisualizationDistance;

    float mParticleRadius, mParticleOpacity;

    float mPointCloudPointSize;
    float mPointCloudPointAlpha;
    float mPointCloudColorLow, mPointCloudColorHigh;

    // This allows FlightPlanner to tell GlScene in what state it currently is.
    // GlScene will adjust render-settings (Show Particles/Grids? Rotate? ...) accordingly.
    enum class FlightPlannerProcessingState
    {
        Idle,
        ReducingPointCloud,
        ParticleSimulation,
        WayPointComputation,
        WayPointChecking
    };

public slots:
    void slotSetFlightPlannerProcessingState(const FlightPlannerProcessingState& state)
    {
        qDebug() << __PRETTY_FUNCTION__ << "setting processing state to" << static_cast<quint8>(state);
        mFlightPlannerProcessingState = state;

        emit suggestVisualization();
    }

    void slotSetVolumeGlobal(const Box3D* volume);
    void slotSetVolumeLocal(const Box3D* volume);

    void slotSetWayPointListAhead(WayPointList* wpl);
    void slotSetWayPointListPassed(WayPointList* wpl);

    // GlWidget renders points from all known PointClouds. These methods (de)register PointClouds for rendering.
    // Ownership remains with the caller, meaning they MUST be deregistered before deletion
    void slotPointCloudRegister(PointCloudCuda *p);
    void slotPointCloudUnregister(PointCloudCuda *p);

    // LogPlayer and RoverConnection set values used/computed by the flightcontroller.
    // These shall be visualized here in GlWidget for debugging.
    void slotSetFlightControllerValues(const FlightControllerValues *const fcv);

    // Called from logplayer, for debugging unfused laser data. Copy it to OpenGL-Buffer if you need it.
    void slotNewRawScan(const RawScan * const rawScan);

    void slotNewVehiclePose(const Pose *const);
    void slotClearVehicleTrajectory();

    void slotSetActiveWayPoint(qint32 index) {mActiveWayPointVisualizationIndex = index; suggestVisualization();}
    void slotSetParticleRadius(float r) { mParticleRadius = r; }

    void slotSetVboInfoGridInformationGain(const quint32 vboPressure, const Box3D& boundingBoxGrid, const Vector3<quint16>& gridCells);
    void slotSetVboInfoGridOccupancy(const quint32 vbo, const Box3D& gridBoundingBox, const Vector3<quint16>& gridCells);
    void slotSetVboInfoGridPathPlanner(const quint32 vbo, const Box3D &gridBoundingBox, const Vector3<quint16> &gridCells);
    void slotSetVboInfoParticles(const quint32 vboPositions, const quint32 count, const float particleRadius, const Box3D particleSystemBoundingBox);

    void slotSetInsStatus(const GnssStatus* const g);

    void slotUpdateMatrixCameraToClip(const quint32 windowWidth, const quint32 windowHeight);
    void slotUpdateMatrixModelToCamera();

    void setCameraZoom(const float zoom) {mCameraZoom = zoom;}
    void setCameraRotation(const float yaw, const float pitch, const float roll);
    void rotateCamera(const float yaw, const float pitch, const float roll);
    void moveCameraTarget(const float x, const float y, const float z);

private:
    FlightPlannerProcessingState mFlightPlannerProcessingState;

    struct RawScanRayVisualization
    {
        QOpenGLVertexArrayObject* vao;
        GLuint vbo;
        QMatrix4x4 relativeScannerPose;
        quint32 distanceIndexFirst;
        quint32 distanceIndexLast;

        RawScanRayVisualization()
        {
            vao = nullptr;
            vbo = 0;
        }
    };
    QVector<RawScanRayVisualization*> mRawScanVisualizations;

    void renderController(const QMatrix4x4 &transform, const PidController* const controller);

    QMatrix4x4 mMatrixCameraToClip, mMatrixModelToCamera;

    QVector3D mCameraPosition;
    QVector3D mCamLookAtOffset;
    // The components of this vector store the rotation (in degrees) of the camera around the origin
    QVector3D mCameraRotation;
    float mCameraZoom;

    bool mIsInitialized;

    WayPointList *mWayPointsAhead, *mWayPointsPassed;
    QList<PointCloudCuda*> mPointCloudsToRender;

    Model *mModelVehicle, *mModelThrust, *mModelConeYaw, *mModelConePitch, *mModelConeRoll, *mModelHoverPosition, *mModelTrajectoryStart, *mModelTrajectoryGoal, *mModelVelocityArrow;
    Model *mModelControllerP, *mModelControllerI, *mModelControllerD;

    const Pose* mLastKnownVehiclePose;

    // Set by slotSetFlightControllerValues(), then visualized for FligthController debugging
    const FlightControllerValues* mLastFlightControllerValues;

    // We use VAOs, as mandated by OpenGL Core Profile 4. The idea: Instead of doing vertex specification
    // - bindBuffer(vbo), enableVertexAttributes(X), vertexAttribPointer(X,...), glDraw*, disableVertexAttribPointer(X), bindBuffer(0) on every call
    // we do this once when the VAO is bound. Then, when rendering, we just bind the VAO, render and unbind it.! This is supposedly faster
    // A VAO does NOT save the bound buffers (VBOs), but it doesn't need to: the vertexAttribPointer saves the buffer, and the VAO saves the vertexattribpointer.
    QOpenGLVertexArrayObject
    *mVaoAxes,
    *mVaoPointCloud,
    *mVaoTrajectory,
    *mVaoBoundingBoxGlobal,
    *mVaoBoundingBoxLocal,
    *mVaoParticles,
    *mVaoGridMapOfInformationGain,
    *mVaoGridMapOfOccupancy,
    *mVaoGridMapOfPathPlanner,
    *mVaoWayPointsAhead,
    *mVaoWayPointsPassed,
    *mVaoSatelliteSignals;

    quint32 mNumberOfSatellitesGps, mNumberOfSatellitesGlonass, mNumberOfSatelliteSignals;

    QMatrix4x4 mRayVisualizationRelativeScannerMatrix;
    quint16 mRayVisualizationUsableDistanceIndexFirst, mRayVisualizationUsableDistanceIndexLast;

    GLuint mUboId;
    unsigned int mUboSize;

    ShaderProgram *mShaderProgramDefault;
    ShaderProgram *mShaderProgramPointCloud;
    ShaderProgram *mShaderProgramRawScanRays;
    ShaderProgram *mShaderProgramWaypoint;
    ShaderProgram *mShaderProgramParticles;
    ShaderProgram *mShaderProgramGrid;
    ShaderProgram *mShaderProgramSatelliteSignals;

    const Box3D *mVolumeGlobal, *mVolumeLocal;

    qint32 mActiveWayPointVisualizationIndex;

    quint32 mNumberOfParticles;

    Box3D mBoundingBoxGridInformationGain;
    Vector3<quint16> mGridInformationGainCellCount;

    Box3D mBoundingBoxGridOccupancy;
    Vector3<quint16> mGridOccupancyCellCount;

    Box3D mBoundingBoxGridPathFinder;
    Vector3<quint16> mGridPathFinderCellCount;


    GLuint mVboAxes;
    GLuint mVboVehiclePathElementSize;
    GLuint mVboVehiclePathBytesMaximum;
    GLuint mVboVehiclePathBytesCurrent;
    GLuint mVboVehiclePath;
    GLuint mVboGridMapOfOccupancy;
    GLuint mVboGridMapOfInformationGain;
    GLuint mVboGridMapOfPathPlanner;
    GLuint mVboParticlePositions;
    GLuint mVboBoundingBoxVolumeLocal;
    GLuint mVboBoundingBoxVolumeGlobal;
    GLuint mVboSatelliteSignals;

signals:
    void suggestVisualization();
};

#endif
