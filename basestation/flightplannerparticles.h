#ifndef FLIGHTPLANNERPARTICLES_H
#define FLIGHTPLANNERPARTICLES_H

#include "flightplannerinterface.h"
#include "particlerenderer.h"
#include "particlesystem.h"
#include "lidarpoint.h"
#include "glwidget.h"
#include <waypoint.h>
#include "openglutilities.h"

class FlightPlannerParticlesDialog;

class PointCloudCuda;

class FlightPlannerParticles : public FlightPlannerInterface
{
    Q_OBJECT
public:
    FlightPlannerParticles(QWidget* parentWidget, GlWidget* glWidget, PointCloud* pointcloud);
    ~FlightPlannerParticles();

    void keyPressEvent(QKeyEvent *event);

private:
    FlightPlannerParticlesDialog* mDialog;

    PointCloudCuda* mPointCloudColliders;

    QVector3D mLastParticleSystemPositionToFollowVehicle;

    QList<WayPoint> mWayPointsGenerated, mWayPointsDetour;

    ParticleSystem* mParticleSystem;
    ParticleRenderer* mParticleRenderer;

    cudaError_t mCudaError;

    ShaderProgram* mShaderProgramGridLines; // for drawing the grid

    // To re-fill our datastructure when the boundingbox has changed.
    bool insertPointsFromNode(const Node* node);

    void slotInitializeWaypointPressureGrid();
    void showWaypointPressure();

    bool getRankedWaypoints(QVector4D* const waypoints, const quint16 numberOfWaypointsRequested);

    bool mProcessPhysics;

    SimulationParameters mSimulationParameters;


    // There is a gridmap of waypoint pressure on the GPU. To be useful for processing on the host, we first create a
    // list of QVector4D that corresponds to the cell's positions. Then, we sort the latter list using thrust::sort_by_key
    // (the key being the waypoint pressure) and receive a ranking of QVector4Ds. Hah!
    float*  mDeviceGridMapWaypointPressureCellWorldPositions;

    // We copy the waypoint pressure of the grid cells from the mVboGridMapOfWayPointPressure VBO to this pointer, then use
    //
    // thrust::sort_by_key(
    //                      mDeviceGridMapWayPointPressureSorted.begin(),
    //                      mDeviceGridMapWayPointPressureSorted.end(),
    //                      mDeviceGridMapCellWorldPositions.begin(),
    //                      operator>()
    // );
    quint8* mDeviceGridMapWayPointPressureSorted;
    // A gridmap (same grid as always) containing values from 0 to 255. 0 means no waypoint candidates within, 255 means maximum waypoint pressure.
    unsigned int   mVboGridMapOfWayPointPressure;
    // Waypoint-pressure-values are stored in a VBO as 8bit-unsigned-ints
    struct cudaGraphicsResource *mCudaVboResourceGridMapOfWayPointPressure; // handles OpenGL-CUDA exchange

signals:
    void vboInfoGridWaypointPressure(quint32 vboPressure, QVector3D gridBoundingBoxMin, QVector3D gridBoundingBoxMax, Vector3i grid);

private slots:
    void slotShowUserInterface();
    void slotGenerateWaypoints();
    void slotProcessPhysics(bool value) { mProcessPhysics = value; }

public slots:
    void slotSetScanVolume(const QVector3D min, const QVector3D max);

    void slotNewScanData(const QVector<QVector3D>* const pointList, const QVector3D* const scannerPosition);

    // Inserts detour-waypoints between vehicle position and next waypoint if necessary.
    // Returns true if path was found, else false.
    void slotCreateSafePathToNextWayPoint();

    void slotClearGridWayPointPressure();

    void slotInitialize();

    // Overridden from base, create safe path to next waypoint whenever current one was reached.
    void slotWayPointReached(const WayPoint);

    // Overridden from base to move vehicle physics object and check for collisions
    void slotVehiclePoseChanged(const Pose *const pose);

    void slotVisualize();

};

#endif // FLIGHTPLANNERPARTICLES_H
