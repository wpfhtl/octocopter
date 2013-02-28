#ifndef POINTCLOUDCUDA_CUH
#define POINTCLOUDCUDA_CUH

#include "parameterspointcloud.cuh"

// Declare all functions that wrap cuda kernel invocations

void setPointCloudParameters(ParametersPointCloud *hostParams);

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

void markCollidingPoints(float* posOriginal,
        float* posSorted,
        unsigned int*  gridPointIndex,
        unsigned int*  cellStart,
        unsigned int*  cellEnd, Grid *grid,
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

void sortPointsToGridCellOrder(float* devicePoints, unsigned int* mDeviceMapGridCell, Grid *grid, unsigned int numberOfPoints);

void buildGridOccupancyMap(float* devicePoints, unsigned int* mDeviceMapGridCell, unsigned int numberOfPoints);

unsigned int convertOccupiedCellsToPoints(float* devicePoints, unsigned int* mDeviceMapGridCell, unsigned int numberOfPoints);


#endif
