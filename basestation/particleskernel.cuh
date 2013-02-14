#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

// This file exists to share the SimParams-Struct between .cpp and .cu-code,
// both particlesystem.h and particleskernel.cu include it.

#include <QVector3D>
#include "vector_types.h"

// simulation parameters

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
    QVector3D getCellSizeQt() const;

    __device__ float3 getCellCenter(const int3& gridCellCoordinate) const;
    QVector3D getCellCenterQt(const int3& gridCellCoordinate) const;

    __device__ float3 getWorldSize() const;
    QVector3D getWorldSizeQt() const;

    __device__ float3 getWorldCenter() const;
    QVector3D getWorldCenterQt() const;

};

struct SimulationParameters
{
    // gridSize is used for TWO grids: (we might split this up into two members later)
    // - one grid contains the particles, its used to find neighbors for collisions
    // - one grid contains the colliders, its used to find neighbors for keeping it sparse
    Grid gridParticleSystem;

    Grid gridWaypointPressure;

/*    uint3 particleSystem.cells;
    float3 particleSystem.worldMin, particleSystem.worldMax;

    uint3 waypointPressure.cells;
    float3 waypointPressure.worldMin, waypointPressure.worldMax;
    */

    unsigned int particleCount;
    unsigned int colliderCountMax;

    float3 gravity;
    float particleRadius;
    float spring;
    float shear;
    float attraction;
    float dampingMotion;
    float velocityFactorCollisionParticle;
    float velocityFactorCollisionBoundary;
};

#endif
