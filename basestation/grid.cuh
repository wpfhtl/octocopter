#ifndef GRID_CUH
#define GRID_CUH

#include <stdint.h>
#include "vector_types.h"
#include "vector_functions.h"
#include "cudahelper.cuh"

struct __align__(16) Grid
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

    __host__ __device__ bool hasSameExtents(const Grid& other)
    {
        return
                   worldMin.x == other.worldMin.x
                && worldMin.y == other.worldMin.y
                && worldMin.z == other.worldMin.z
                && worldMax.x == other.worldMax.x
                && worldMax.y == other.worldMax.y
                && worldMax.z == other.worldMax.z;
    }

    __host__ __device__ int3 clampCellCoordinate(const int3 c)
    {
        int3 result;
        result.x = cudaBound(0, c.x, cells.x-1);
        result.y = cudaBound(0, c.y, cells.y-1);
        result.z = cudaBound(0, c.z, cells.z-1);
        return result;
    }

    __host__ __device__ bool isCellInGrid(int3 gridCellCoordinate) const
    {
        return
                gridCellCoordinate.x < cells.x && gridCellCoordinate.y < cells.y && gridCellCoordinate.z < cells.z &&
                gridCellCoordinate.x >= 0 && gridCellCoordinate.y >= 0 && gridCellCoordinate.z >= 0;
    }

    __host__ __device__ bool isPositionInGrid(float4 position) const
    {
        return
                position.x <= worldMax.x && position.y <= worldMax.y && position.z <= worldMax.z &&
                position.x >= worldMin.x && position.y >= worldMin.y && position.z >= worldMin.z;
    }

    __host__ void initialize()
    {
        cells.x = cells.y = cells.z = 0;
    }

    __host__ __device__ unsigned int getLongestSideCellCount() const
    {
        if(cells.x >= cells.y && cells.x >= cells.z) return cells.x;
        else if(cells.y >= cells.x && cells.y >= cells.z) return cells.y;
        else return cells.z;
    }

    // Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
    __host__ __device__ unsigned int getCellHash(int3 gridCellCoordinate) const;
    __host__ __device__ int getSafeCellHash(int3 gridCellCoordinate) const; // returns -1 for an outside-cell

    __host__ __device__ uint32_t getCellCount() const {return cells.x * cells.y * cells.z;}

    // If the Grid bounds are defined by worldMin and worldMax, then what is the best cells-configuration
    // given a fixed @minDist? result would be e.g. 256x32x256 cells.
    __host__ __device__ uint3 getOptimumResolution(const float minDist);

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
