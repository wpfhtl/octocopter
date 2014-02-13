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
    static const int mMaxWaypoints = 1000; // should be more than enough
    ParametersPathPlanner mParametersPathPlanner;
    PointCloudCuda* mPointCloudColliders;
    bool mRepopulateOccupanccyGrid;
    cudaStream_t mCudaStream;
    bool mIsInitialized;

    // This points to float4 elements in GPU memory with waypoints as computed on the GPU.
    // It  is used when computing subpaths and when checking waypoint safety.
    float* mDeviceWaypoints;

    // A pointer to the populated and dilated occupancy grid. Either nullptr or mapped.
    // The pathplanner copies it and uses that for path planning.
    quint8* mGridOccupancyTemplate;
    // A pointer to the grid used by the path planner
    quint8* mGridOccupancyPathPanner;

    GLuint mVboGridOccupancy;
    GLuint mVboGridPathPlanner;
    struct cudaGraphicsResource *mCudaVboResourceGridOccupancyTemplate; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceGridPathPlanner; // handles OpenGL-CUDA exchange

    void populateOccupancyGrid();
    void alignPathPlannerGridToColliderCloud();

    bool checkAndMapGridOccupancy(cudaGraphicsResource *resource);
    bool checkAndUnmapGridOccupancy(cudaGraphicsResource *resource);

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

    // Checks whether all waypoints are free w.r.t. to the dilated occupancy grid. When a waypoint is in
    // an occupied cell, it will be removed from the list.
    // Returns true if everything was fine , false if collisions occurred and waypoints were removed.
    bool checkWayPointSafety(const QVector3D &vehiclePosition, const WayPointList * const wayPointsAhead);

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
//    void generateNewWayPoints();

    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    void slotColliderCloudInsertedPoints();
    void slotComputePath(const QVector3D& vehiclePosition, const WayPointList &wayPointList);
};

#endif // PATHPLANNER_H
