#ifndef POINTCLOUDCUDA_CUH
#define POINTCLOUDCUDA_CUH

#include "parameterspointcloud.cuh"

// Declare all functions that wrap cuda kernel invocations

void getDeviceAddressOfParametersPointCloud(ParametersPointCloud** ptr);

void copyParametersToGpu(ParametersPointCloud *hostParams);

void getBoundingBox(
        float *dPoints,
        uint numPoints,
        float3& min,
        float3& max);

void markCollidingPoints(float* posOriginal,
        float* posSorted,
        unsigned int*  gridPointIndex,
        unsigned int*  pointCellStart,
        unsigned int*  pointCellStopp,
        unsigned int   numPoints);

void sortMapAccordingToKeys(
        unsigned int *gridCellIndex,
        unsigned int *gridPointIndex,
        unsigned int numPoints);

// checks all @numPoints starting at @devicePoints for a value of 0/0/0/0 and removes those points matching.
// returns the remaining number of points.
unsigned int removeClearedPoints(
        float* devicePoints,
        unsigned int numberOfPoints);

unsigned int removePointsOutsideBoundingBox(float* points,
        unsigned int numberOfPoints,
        Grid *grid);

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
