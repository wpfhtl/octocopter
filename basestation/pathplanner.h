#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QObject>

#include "waypoint.h"
#include "grid.cuh"
#include "waypointlist.h"
#include "pointcloudcuda.h"
#include "pathplanner.cuh"
#include "parameterspathplanner.cuh"

class PathPlanner : public QObject
{
    Q_OBJECT

private:
    static const int mMaxWaypoints = 100; // should be more than enough
    ParametersPathPlanner mParametersPathPlanner;
    PointCloudCuda* mPointCloudColliders;
    bool mRepopulateOccupanccyGrid;
    cudaStream_t mCudaStream;
    bool mIsInitialized;

    // This points to float4 elements in GPU memory with waypoints as computed on the GPU.
    // It  is used when computing subpaths and when checking waypoint safety.
    float* mDeviceWaypoints;

    GLuint mVboGridOccupancy;
    GLuint mVboGridPathFinder;
    struct cudaGraphicsResource *mCudaVboResourceGridOccupancy; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceGridPathFinder; // handles OpenGL-CUDA exchange

    void populateOccupancyGrid(quint8 *gridOccupancy = nullptr);
    void alignPathPlannerGridToColliderCloud();

public:
    explicit PathPlanner(PointCloudCuda * const pointCloudColliders, QObject *parent = 0);
    ~PathPlanner();

    // Must be called:
    // - before any work is done
    // - whenever the number of grid cells change
    // Requires:
    // - active OpenGL context (because of cudaGraphicsGLRegisterBuffer)
    // - defined number of gridcells (grid-cells are allocated in VBO)
    void initialize();

    void checkWayPointSafety(const QVector3D &vehiclePosition, const WayPointList* const wayPointsAhead);
    void moveWayPointsToSafety(WayPointList* wayPointList);

signals:
    // To allow others to render our occupancy grid.
    void vboInfoGridOccupancy(quint32 vboPressure, Box3D gridBoundingBox, Vector3<quint16> grid);
    void vboInfoGridPathPlanner(quint32 vboPressure, Box3D gridBoundingBox, Vector3<quint16> grid);

    // This is ALWAYS emitted after a call to slotRequestPath(). If the list is empty, no path was found.
    // If it is not empty, the first and last elements MUST be start and goal, respectively.
    //
    // Can also be emitted after checkWayPointSafety() found an unsafe waypoint and the path was re-planned.
    void path(const QList<WayPoint>* const, WayPointListSource);

    // Is emitted when checkWayPointSafety() finds out that the path must be deleted completely.
    void generateNewWayPoints();

    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    void slotColliderCloudInsertedPoints();
    void slotComputePath(const QVector3D& vehiclePosition, const WayPointList &wayPointList);
};

#endif // PATHPLANNER_H
