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

    // how many points are currently stored (not counting the points queued thereafter)
    unsigned int elementCount;

    // how many points were appended/queued after the last reduction. After
    // reducing, elementCount += reduce(elementQueue); elementQueueCount = 0.
    unsigned int elementQueueCount;

    // capacity of pointcloud
    unsigned int capacity;

    // should colliding points be deleted from even (0) or odd (1) indices?
//    unsigned int remainder;
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
unsigned int removeZeroPoints(
        float* devicePoints,
        unsigned int numPoints);

// test!
unsigned int snapToGridAndMakeUnique(float *devicePoints, unsigned int numPoints, float minimumDistance);

unsigned int replaceCellPointsByMeanValue(
        float *devicePoints,
        float *devicePointsSorted,
        unsigned int *pointCellStart,
        unsigned int *pointCellStopp,
        unsigned int *gridCellIndex,
        unsigned int *gridPointIndex, unsigned int numPoints,
        unsigned int numCells);

/*unsigned int removePointsOutsideBoundingBox(
        float* devicePointsBase,
        unsigned int numberOfPoints,
        float* bBoxMin,
        float* bBoxMax);*/

unsigned int copyPoints(
        float* devicePointsBaseDst,
        float* devicePointsBaseSrc,
        unsigned int numberOfPointsToCopy);

unsigned int copyPointsInBoundingBox(
        float* devicePointsBaseDst,
        float* devicePointsBaseSrc,
        float3& bBoxMin,
        float3& bBoxMax,
        unsigned int numberOfPointsToCopy);

#endif
