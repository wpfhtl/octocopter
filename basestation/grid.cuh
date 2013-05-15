#ifndef GRID_CUH
#define GRID_CUH

#include <stdint.h>
#include "vector_types.h"
#include "vector_functions.h"

struct Grid
{
    uint3 cells;
    float3 worldMin;
    float3 worldMax;

    __host__ __device__ Grid& operator=(const Grid& other)
    {
        cells = other.cells;
        worldMin = other.worldMin;
        worldMax = other.worldMax;
        return *this;
    }


    __host__ void initialize()
    {
        cells.x = cells.y = cells.z = 0;
    }

    // Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
    __host__ __device__ unsigned int getCellHash(int3 gridCellCoordinate) const;

    uint32_t cellCount() const {return cells.x * cells.y * cells.z;}

    // If the Grid bounds are defined by worldMin and worldMax, then what is the best cells-configuration
    // given a fixed @minDist? result would be e.g. 256x32x256 cells.
    __host__ __device__ uint3 getOptimalResolution(const float minDist);

    // Given the cell hash (=gl_PrimitiveIDIn), whats the 3d-grid-coordinate of the cell's center?
    // This is the reverse of particleskernel.cu -> calcGridHash(int3 gridCell).
    __host__ __device__ int3 getCellCoordinate(const unsigned int hash) const;
    __host__ __device__ int3 getCellCoordinate(const float3& worldPos) const;

    __host__ __device__ float3 getCellSize() const;

    __host__ __device__ float3 getCellCenter(const int3& gridCellCoordinate) const;

    __host__ __device__ float3 getWorldSize() const;

    __host__ __device__ float3 getWorldCenter() const;

};


void computeMappingFromPointToGridCell(
        unsigned int* gridParticleHash,
        unsigned int* gridParticleIndex,
        float*        pos,
        const Grid* const grid,
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


#endif
