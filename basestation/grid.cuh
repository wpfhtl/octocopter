#ifndef GRID_CUH
#define GRID_CUH

#include <QVector3D>
#include "vector_types.h"
#include "vector_functions.h"

struct Grid
{
    uint3 cells;
    float3 worldMin;
    float3 worldMax;

    // Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
    __device__ unsigned int getCellHash(int3 gridCellCoordinate) const;

    quint32 cellCount() const {return cells.x * cells.y * cells.z;}

    // Given the cell hash (=gl_PrimitiveIDIn), whats the 3d-grid-coordinate of the cell's center?
    // This is the reverse of particleskernel.cu -> calcGridHash(int3 gridCell).
    __host__ __device__ int3 getCellCoordinate(const unsigned int hash) const;
    __device__ int3 getCellCoordinate(const float3& worldPos) const;
    int3 getCellCoordinate(const QVector3D& worldPos) const;

    __device__ float3 getCellSize() const;
//    QVector3D getCellSizeQt() const;

    __device__ float3 getCellCenter(const int3& gridCellCoordinate) const;
//    QVector3D getCellCenterQt(const int3& gridCellCoordinate) const;

    __device__ float3 getWorldSize() const;
//    QVector3D getWorldSizeQt() const;

    __device__ float3 getWorldCenter() const;
//    QVector3D getWorldCenterQt() const;

};


void computeMappingFromGridCellToParticle(
        unsigned int* gridParticleHash,
        unsigned int* gridParticleIndex,
        float*        pos,
        Grid*         grid,
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
