#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QObject>
#include <QFuture>
#include <QTimer>

#include "common.h"
#include "waypoint.h"
#include "grid.cuh"
#include "waypointlist.h"
#include "pointcloudcuda.h"
#include "pathplanner.cuh"
#include "parameterspathplanner.cuh"

//#include <QAbstractOpenGLFunctions>

class PathPlanner : public QObject
{
    Q_OBJECT

private:
    static const int mMaxWaypoints = 100; // should be more than enough
    ParametersPathPlanner mParametersPathPlanner;
    PointCloudCuda* mPointCloudColliders;
    bool mRepopulateOccupanccyGrid;
    cudaStream_t mCudaStream;

    // This points to float4 elements in GPU memory with waypoints as computed on the GPU.
    // It  is used when computing subpaths and when checking waypoint safety.
    float* mDeviceWaypoints;

    GLuint mVboGridOccupancy;
    GLuint mVboGridPathFinder;
    struct cudaGraphicsResource *mCudaVboResourceGridOccupancy; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceGridPathFinder; // handles OpenGL-CUDA exchange

    void populateOccupancyGrid(quint8 *gridOccupancy = nullptr);
    //void printHostOccupancyGrid(quint8* deviceGridOccupancy);


public:
    explicit PathPlanner(QObject *parent = 0);
    ~PathPlanner();

    void checkWayPointSafety(const QVector3D &vehiclePosition, const WayPointList* const wayPointsAhead);

signals:
    // To allow others to render our occupancy grid.
    void vboInfoGridOccupancy(quint32 vboPressure, Box3D gridBoundingBox, Vector3i grid);
    void vboInfoGridPathFinder(quint32 vboPressure, Box3D gridBoundingBox, Vector3i grid);

    // This is ALWAYS emitted after a call to slotRequestPath(). If the list is empty, no path was found.
    // If it is not empty, the first and last elements MUST be start and goal, respectively.
    //
    // Can also be emitted after checkWayPointSafety() foud an unsafe waypoint and the path was re-planned.
    void path(const QList<WayPoint>* const, WayPointListSource);

    void message(const LogImportance& importance, const QString& source, const QString& message);

public slots:
    void slotInitialize();
    void slotColliderCloudInsertedPoints();
    void slotSetPointCloudColliders(PointCloudCuda* const);
    void slotComputePath(const QVector3D& vehiclePosition, const WayPointList &wayPointList);
};

#endif // PATHPLANNER_H
