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

// This path planner is supposed to run asynchronously on the GPU. For this, we execute the actual planning
// in its own thread on the host, which executes computePath(). This uses the GPU in its own stream (not
// stream 0, which is the default stream used e.g. by thrust).
//
// http://on-demand.gputechconf.com/gtc-express/2011/presentations/StreamsAndConcurrencyWebinar.pdf

class PathPlanner : public QObject
{
    Q_OBJECT
private:
    static const int mMaxWaypoints = 100; // should be more than enough

    WayPointList mWayPointListForRequestedPath;
    WayPointList mComputedPath;
    ParametersPathPlanner mParametersPathPlanner;
    PointCloudCuda* mPointCloudColliders;
    bool mRepopulateOccupanccyGrid;
    cudaStream_t mCudaStream;
    quint8*  mHostOccupancyGrid;
    float*  mDeviceWaypoints;
    float* mHostWaypoints;

    GLuint mVboGridOccupancy;
    GLuint mVboGridPathFinder;
    struct cudaGraphicsResource *mCudaVboResourceGridOccupancy; // handles OpenGL-CUDA exchange
    struct cudaGraphicsResource *mCudaVboResourceGridPathFinder; // handles OpenGL-CUDA exchange

    // When a path is requested, we QtConcurrent::run() the computePath() member function, which uses the
    // GPU. Functions launched in this way cannot be cancelled using QFuture::cancel(). So, when a path is
    // requested while a previous computation is still running, we're out of luck.
    // not true, no extra thread!
    bool mCancelComputation;
    QFuture<void> mFuture;

    void printHostOccupancyGrid(quint8* deviceGridOccupancy);


public:
    explicit PathPlanner(QObject *parent = 0);
    ~PathPlanner();

signals:
    // To allow others to render our occupancy grid.
    void vboInfoGridOccupancy(quint32 vboPressure, Box3D gridBoundingBox, Vector3i grid);
    void vboInfoGridPathFinder(quint32 vboPressure, Box3D gridBoundingBox, Vector3i grid);

    // This is ALWAYS emitted after a call to slotRequestPath(). If the list is empty, no path was found.
    // If it is not empty, the first and last elements MUST be start and goal, respectively.
    void pathFound(const QList<WayPoint>* const, WayPointListSource);

private slots:
    void slotComputePathOnGpu();

public slots:
    void slotInitialize();
//    void slotSetVolume(const QVector3D& min, const QVector3D& max);
    void slotColliderCloudInsertedPoints();
    void slotSetPointCloudColliders(PointCloudCuda* const);
    void slotRequestPath(const QVector3D& vehiclePosition, const WayPointList &wayPointList);
};

#endif // PATHPLANNER_H
