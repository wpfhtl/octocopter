#ifndef POINTCLOUD_CUH
#define POINTCLOUD_CUH

#include "vector_types.h"

// pointcloud parameters
struct PointCloudParameters
{
    // gridSize contains the points, its used to find neighbors for keeping it sparse
    uint3 gridSize;

    // minimum distance to closest neighbor for new points
    float minimumDistance;

    // the world size of the reduced cloud
    float3 bBoxMin, bBoxMax;

    // the world size of the queued cloud should probably be determined on the GPU

    // how many points are currently stored (not counting the points queued thereafter)
    unsigned int elementCount;

    // how many points were appended/queued after the last reduction. After
    // reducing, elementCount += reduce(elementQueue); elementQueueCount = 0.
    unsigned int elementQueueCount;

    // capacity of pointcloud
    unsigned int capacity;
};


// Declare all functions that wrap cuda kernel invocations

void setPointCloudParameters(PointCloudParameters *hostParams);

void getBoundingBox(
        float *dPoints,
        uint numPoints,
        float3& min,
        float3& max);

void computeMappingFromGridCellToPoint(
        unsigned int*  gridCellIndex,
        unsigned int*  gridPointIndex,
        float* pos,
        int numPoints);

void sortPosAccordingToGridCellAndFillCellStartAndEndArrays(
        unsigned int*  cellStart,
        unsigned int*  cellEnd,
        float* sortedPos,
        unsigned int*  gridCellIndex,
        unsigned int*  gridPointIndex,
        float* oldPos,
        unsigned int   numParticles,
        unsigned int   numCells);

void markCollidingPoints(
        float* posOriginal,
        float* posSorted,
        unsigned int*  gridPointIndex,
        unsigned int*  cellStart,
        unsigned int*  cellEnd,
        unsigned int   numPoints,
        unsigned int   numCells);

void sortMapAccordingToKeys(
        unsigned int *gridCellIndex,
        unsigned int *gridPointIndex,
        unsigned int numPoints);

// checks all @numPoints starting at @devicePoints for a value of 0/0/0/0 and removes those points matching.
// returns the remaining number of points.
unsigned int removeRedundantPoints(
        float* devicePoints,
        unsigned int numPoints);

#endif
