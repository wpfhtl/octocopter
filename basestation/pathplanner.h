#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QObject>
#include <QFuture>
#include <QTimer>
#include "common.h"
#include "waypoint.h"
#include "grid.cuh"
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
    static const int mMaxWaypoints = 5000; // should be more than enough

    ParametersPathPlanner mParametersPathPlanner;
    PointCloudCuda* mPointCloudColliders;
    bool mPointCloudCollidersChanged;
    cudaStream_t mCudaStream;
    quint8*  mDeviceOccupancyGrid;
    float*  mDeviceWaypoints;
    float* mHostWaypoints;
    QList<WayPoint> mComputedPath;

    struct cudaGraphicsResource *mCudaVboResourceColliderPositions; // handles OpenGL-CUDA exchange

    // When a path is requested, we QtConcurrent::run() the computePath() member function, which uses the
    // GPU. Functions launched in this way cannot be cancelled using QFuture::cancel(). So, when a path is
    // requested while a previous computation is still running, we're out of luck.
    bool mCancelComputation;
    QFuture<void> mFuture;


public:
    explicit PathPlanner(QObject *parent = 0);
    ~PathPlanner();

signals:
    // This is ALWAYS emitted after a call to slotRequestPath(). If the list is empty, no path was found.
    // If it is not empty, the first and last elements MUST be start and goal, respectively.
    void pathFound(const QList<WayPoint>* const);

private slots:
    void slotComputePathOnGpu();

public slots:
    void slotInitialize();
    void slotColliderCloudInsertedPoints();
    void slotSetPointCloudColliders(PointCloudCuda* const);
    void slotRequestPath(const QVector3D& start, const QVector3D& goal);
};

#endif // PATHPLANNER_H
