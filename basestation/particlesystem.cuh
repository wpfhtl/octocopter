#ifndef PARTICLESYSTEM_CUH
#define PARTICLESYSTEM_CUH

#include "parametersparticlesystem.cuh"

void copyArrayFromDevice(
        void*         host,
        const void*   device,
        struct cudaGraphicsResource **cuda_vbo_resource,
        int           size);

void copyArrayToDevice(
        void*         device,
        const void*   host,
        int           offset,
        int           size);

void copyParametersToGpu(ParametersParticleSystem *hostParams);

void getDeviceAddressOfParametersParticleSystem(ParametersParticleSystem** ptr);

void integrateSystem(
        float*        pos,
        float*        vel,
        uint8_t*      gridWaypointPressure,
        float*        particleCollisionPositions,
        const ParametersParticleSystem* const paramsParticleSystem,
        unsigned int  numParticles);

void collideParticlesWithParticlesAndColliders(
        float*        newVel,
        float*        particlePosVbo, // this is the VBO!
        float*        particleCollisionPositions,

        float*        particlePosSorted,
        float*        particleVelSorted,
        unsigned int* particleMapIndex,
        unsigned int* particleCellStart,
        unsigned int* particleCellEnd,

        float*        colliderPosSorted,
        unsigned int* colliderMapIndex,
        unsigned int* colliderCellStart,
        unsigned int* colliderCellEnd,

        unsigned int  numParticles,
        unsigned int  numCells);

void sortGridOccupancyMap(
        unsigned int* dGridParticleHash,
        unsigned int* dGridParticleIndex,
        unsigned int  numParticles);

void fillGridMapCellWorldPositions(
        float* gridMapCellWorldPositions,
        unsigned int numCells);

void sortGridMapWayPointPressure(
        float* gridMapWayPointPressureSorted,
        float* gridMapCellWorldPositions,
        unsigned int numCells,
        unsigned int numWaypointsRequested);


uint8_t getMaximumWaypointPressure(
        uint8_t* gridMapOfWayPointPressure,
        unsigned int numberOfCells);

void decreaseWaypointPressure(
        uint8_t* gridMapOfWayPointPressure,
        unsigned int numberOfCells);

void computeWaypointBenefit(
        float* gridMapOfWayPointPressureDst,
        uint8_t* gridMapOfWayPointPressureSrc,
        float* vehiclePosition,
        unsigned int numberOfCells);

#endif
