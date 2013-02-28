#ifndef PARAMETERSPOINTCLOUD_CUH
#define PARAMETERSPOINTCLOUD_CUH

#include "grid.cuh"

// pointcloud parameters
struct ParametersPointCloud
{
    Grid grid;

    // minimum distance to closest neighbor for new points
    float minimumDistance;

    // how many points are currently stored (not counting the points queued thereafter)
    unsigned int elementCount;

    // how many points were appended/queued after the last reduction. After
    // reducing, elementCount += reduce(elementQueue); elementQueueCount = 0.
    unsigned int elementQueueCount;

    // capacity of pointcloud
    unsigned int capacity;

    void initialize()
    {
        grid.cells.x = grid.cells.y = grid.cells.z = 0;
        minimumDistance = 0.0f;
        elementCount = 0;
        elementQueueCount = 0;
        capacity = 0;
    }
};

#endif
