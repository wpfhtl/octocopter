#ifndef PARAMETERSPOINTCLOUD_CUH
#define PARAMETERSPOINTCLOUD_CUH

#include "grid.cuh"

// pointcloud parameters
struct ParametersPointCloud
{
    Grid grid;

    // minimum distance to closest neighbor for new points
    float minimumDistance;

    // When using the VBO as a ring buffer, where to insert?
    unsigned int insertionCursor;

    // how many points are currently stored (not counting the points queued thereafter)
    unsigned int elementCount;

    // how many points were appended/queued after the last reduction. After
    // reducing, elementCount += reduce(elementQueue); elementQueueCount = 0.
    unsigned int elementQueueCount;

    // capacity of pointcloud
    unsigned int capacity;

    void initialize()
    {
        grid.initialize();
        minimumDistance = 0.0f;
        insertionCursor = 0;
        elementCount = 0;
        elementQueueCount = 0;
        capacity = 0;
    }
};

#endif
