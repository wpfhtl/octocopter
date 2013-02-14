
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

void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource);

void copyParametersToGpu(SimulationParameters *hostParams);

void integrateSystem(
        float*        pos,
        float*        vel,
        uint8_t*      gridWaypointPressure,
        float*        particleCollisionPositions,
        float         deltaTime,
        unsigned int  numParticles);

void computeMappingFromGridCellToParticle(
        unsigned int* gridParticleHash,
        unsigned int* gridParticleIndex,
        float*        pos,
        int           numParticles);

void sortParticlePosAndVelAccordingToGridCellAndFillCellStartAndEndArrays(
        unsigned int* cellStart,
        unsigned int* cellEnd,
        float*        sortedPos,
        float*        sortedVel,
        unsigned int* gridParticleHash,
        unsigned int* gridParticleIndex,
        float*        oldPos,
        float*        oldVel,
        unsigned int  numParticles,
        unsigned int  numCells);

void collideParticlesWithParticlesAndColliders(
        float*        newVel,
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
        uint8_t* gridMapWayPointPressureSorted,
        float* gridMapCellWorldPositions,
        unsigned int numCells,
        unsigned int numWaypointsRequested);

void moveGridMapWayPointPressureValuesByWorldPositionOffset(
        quint8* gridMapOfWayPointPressure,
        float* offset,
        unsigned int numberOfCells);
