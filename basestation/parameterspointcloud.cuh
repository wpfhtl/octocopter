#ifndef PARAMETERSPOINTCLOUD_CUH
#define PARAMETERSPOINTCLOUD_CUH

#include "grid.cuh"

// pointcloud parameters
struct __align__(16) ParametersPointCloud
{
    Grid grid;

    // minimum distance to closest neighbor for new points
    float minimumDistance;

    // capacity of pointcloud
    unsigned int capacity;

    // how many points are currently stored (not counting the points queued thereafter)
    unsigned int elementCount;

    // how many points were appended/queued after the last reduction. After
    // reducing, elementCount += reduce(elementQueue); elementQueueCount = 0.
    //
    // Note that if the underlying hardware doesn't support CUDA/reduction,
    // the VBO is used as a ring buffer. In this case, elementCount will remain
    // 0 (as no reduction takes place) and elementQueueCount grows even past
    // @capacity, indicating NOT the number of points stored, but the number of
    // points inserted.
    unsigned int elementQueueCount;

    void initialize()
    {
        grid.initialize();
        minimumDistance = 0.0f;
        elementCount = 0;
        elementQueueCount = 0;
        capacity = 0;
    }
};

#endif
