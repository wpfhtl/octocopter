// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#ifndef PARTICLES_KERNEL_H_
#define PARTICLES_KERNEL_H_

#include "helper_math.h"
#include "particleskernel.cuh"

#include "thrust/tuple.h"

// 0 seems to be 10% faster with 256k particles
#define USE_TEX 1

#if USE_TEX
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif


#if USE_TEX
// textures for particle position and velocity
texture<float4, 1, cudaReadModeElementType> oldPosTex;
texture<float4, 1, cudaReadModeElementType> oldVelTex;

texture<uint, 1, cudaReadModeElementType> gridParticleHashTex;
texture<uint, 1, cudaReadModeElementType> cellStartTex;
texture<uint, 1, cudaReadModeElementType> cellEndTex;
#endif

// simulation parameters in constant memory
__constant__ SimParams params;

struct integrate_functor
{
    float deltaTime;

    __host__ __device__
    integrate_functor(float delta_time) : deltaTime(delta_time) {}

    template <typename Tuple>
    //__host__ otherwise we get warnings that params (global mem) cannot be read directly in a host function
    __device__
    void operator()(Tuple t)
    {
        volatile float4 posData = thrust::get<0>(t);
        volatile float4 velData = thrust::get<1>(t);
        float3 pos = make_float3(posData.x, posData.y, posData.z);
        float3 vel = make_float3(velData.x, velData.y, velData.z);

        // Sample geometry has a w component of 1.0, fixed points from the pointcloud have a w component of 0.0.
        // Of course, fixed points do not need integration.
        if(posData.w < 0.5f)
        {
            thrust::get<1>(t) = make_float4(0.0, 0.0, 0.0, velData.w); // set velocity to zero. Needed?
            return;
        }

        vel += params.gravity * deltaTime;
        vel *= params.dampingMotion;

        // new position = old position + velocity * deltaTime
        pos += vel * deltaTime;

        // collisions with cube sides
        if (pos.x > params.worldMax.x - params.particleRadius) { pos.x = params.worldMax.x - params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
        if (pos.x < params.worldMin.x + params.particleRadius) { pos.x = params.worldMin.x + params.particleRadius; vel.x *= params.velocityFactorCollisionBoundary;}
        if (pos.z > params.worldMax.z - params.particleRadius) { pos.z = params.worldMax.z - params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
        if (pos.z < params.worldMin.z + params.particleRadius) { pos.z = params.worldMin.z + params.particleRadius; vel.z *= params.velocityFactorCollisionBoundary;}
        if (pos.y > params.worldMax.y - params.particleRadius) { pos.y = params.worldMax.y - params.particleRadius; vel.y *= params.velocityFactorCollisionBoundary;}

        // special case: hitting bottom plane of bounding box
        if (pos.y < params.worldMin.y + params.particleRadius)
        {
//            pos.y = params.worldMin.y + params.particleRadius;
//            vel.y *= params.velocityFactorCollisionBoundary;
//            vel.y = 3.0f;
            pos.y = params.worldMax.y - params.particleRadius;
            vel.y *= 0.1f;

            // delete sphere and look up its last collision
        }

        // store new position and velocity
        thrust::get<0>(t) = make_float4(pos, posData.w);
        thrust::get<1>(t) = make_float4(vel, velData.w);
    }
};

// Calculate's a particle's containing cell in the uniform grid
__device__ int3 calcGridPos(float3 p)
{
    float3 cellSize;
    cellSize.x = (params.worldMax.x - params.worldMin.x) / params.gridSize.x;
    cellSize.y = (params.worldMax.y - params.worldMin.y) / params.gridSize.y;
    cellSize.z = (params.worldMax.z - params.worldMin.z) / params.gridSize.z;

    int3 gridPos;
    gridPos.x = floor((p.x - params.worldMin.x) / cellSize.x);
    gridPos.y = floor((p.y - params.worldMin.y) / cellSize.y);
    gridPos.z = floor((p.z - params.worldMin.z) / cellSize.z);
    return gridPos;
}

// Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
__device__ uint calcGridHash(int3 gridPos)
{
    gridPos.x = gridPos.x & (params.gridSize.x-1);  // wrap grid, assumes size is power of 2
    gridPos.y = gridPos.y & (params.gridSize.y-1);
    gridPos.z = gridPos.z & (params.gridSize.z-1);
    return __umul24(__umul24(gridPos.z, params.gridSize.y), params.gridSize.x) + __umul24(gridPos.y, params.gridSize.x) + gridPos.x;
}

// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToParticleD(uint*   gridParticleHash,  // output
               uint*   gridParticleIndex, // output
               float4* pos,               // input: particle positions
               uint    numParticles)
{
    uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    if(index >= numParticles) return;

    volatile float4 p = pos[index];

    // In which grid cell does the particle live?
    int3 gridPos = calcGridPos(make_float3(p.x, p.y, p.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    uint hash = calcGridHash(gridPos);

    // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
    // exactly correct, because there can be multiple keys (because one cell can store many particles)
    gridParticleHash[index] = hash;

    // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
    gridParticleIndex[index] = index;
}

// rearrange particle data into sorted order (sorted according to containing grid cell), and find the start of each cell in the sorted hash array
__global__
void sortPosAndVelAccordingToGridCellAndFillCellStartAndEndArraysD(
        uint*   cellStart,        // output: cell start index
        uint*   cellEnd,          // output: cell end index
        float4* sortedPos,        // output: sorted positions, sorted according to the containing gridcell
        float4* sortedVel,        // output: sorted velocities, sorted according to the containing gridcell
        uint *  gridParticleHash, // input: sorted grid hashes
        uint *  gridParticleIndex,// input: sorted particle indices
        float4* oldPos,           // input: UNsorted position array
        float4* oldVel,           // input: UNsorted velocity array
        uint    numParticles)
{
    uint threadIndex = __umul24(blockIdx.x,blockDim.x) + threadIdx.x;

    // This resides in shared memory space of the threadBlock, lives as
    // long as the block and is accessible from all threads in the block.
    // Its size (in bytes) is defined at runtime through the Ns parameter
    // in the <<Dg, Db, Ns, S>> expression of the caller.
    // Here, its set to ((ThreadsInBlock + 1) elements)
    extern __shared__ uint sharedHash[];

    uint hash;

    // When particleCount is smaller than a multiple of the block size, the remaining threads do nothing.
    if(threadIndex < numParticles)
    {
        hash = gridParticleHash[threadIndex];

        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x+1] = hash; // => key of the sorted map

        if(threadIndex > 0 && threadIdx.x == 0)
        {
            // first thread in block must load neighbor particle hash
            sharedHash[0] = gridParticleHash[threadIndex-1];
        }
    }

    __syncthreads();

    if (threadIndex < numParticles)
    {
        // If this particle has a different cell index to the previous particle then it must be the
        // first particle in the cell, so store the index of this particle in the cell. As it isn't
        // the first particle, it must also be the cell end of the previous particle's cell
        if(threadIndex == 0 || hash != sharedHash[threadIdx.x])
        {
            cellStart[hash] = threadIndex;
            if (threadIndex > 0)
                cellEnd[sharedHash[threadIdx.x]] = threadIndex;
        }

        if(threadIndex == numParticles - 1)
        {
            cellEnd[hash] = threadIndex + 1;
        }

        // Now use the sorted index to reorder the pos and vel data
        uint sortedIndex = gridParticleIndex[threadIndex]; // => value of the sorted map
        float4 pos = FETCH(oldPos, sortedIndex);       // macro does either global read or texture fetch,
        float4 vel = FETCH(oldVel, sortedIndex);       // see particles_kernel.cuh

        // ben: hier if() beenden, dann syncthreads() und dann nicht in sortedPos schreiben, sondern in oldPos? Bräuchte ich dann noch zwei pos/vel container?
        sortedPos[threadIndex] = pos;
        sortedVel[threadIndex] = vel;
    }
}

// collide two spheres using DEM method
__device__
float3 collideSpheres(float3 posA, float3 posB,
                      float3 velA, float3 velB,
                      float radiusA, float radiusB,
                      float attraction)
{
    // calculate relative position
    float3 relPos = posB - posA;

    float dist = length(relPos);
    float collideDist = radiusA + radiusB;

    float3 force = make_float3(0.0f);
    if (dist < collideDist)
    {
        float3 norm = relPos / dist;

        // relative velocity
        float3 relVel = velB - velA;

        // relative tangential velocity
        float3 tanVel = relVel - (dot(relVel, norm) * norm);

        // spring force
        force = -params.spring*(collideDist - dist) * norm;
        // dashpot (damping) force
        force += params.velocityFactorCollisionParticle*relVel;
        // tangential shear force
        force += params.shear*tanVel;
        // attraction
        force += attraction*relPos;
    }

    return force;
}



// collide a particle against all other particles in a given cell
__device__
float3 collideCell(int3    gridPos,     // grid cell to search for particles than could collide
                   uint    index,       // index of particle that is being collided
                   float3  pos,         // position of particle that is being collided
                   float3  vel,         // velocity of particle that is being collided
                   float4* oldPos,
                   float4* oldVel,
                   uint*   cellStart,
                   uint*   cellEnd)
{
    uint gridHash = calcGridHash(gridPos);

    // get start of bucket for this cell
    uint startIndex = FETCH(cellStart, gridHash);

    float3 force = make_float3(0.0f);

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = FETCH(cellEnd, gridHash);
        for(uint j=startIndex; j<endIndex; j++)
        {
            // check not colliding with self
            if (j != index)
            {
                float3 pos2 = make_float3(FETCH(oldPos, j));
                float3 vel2 = make_float3(FETCH(oldVel, j));

                // collide two spheres
                force += collideSpheres(pos, pos2, vel, vel2, params.particleRadius, params.particleRadius, params.attraction);
            }
        }
    }
    return force;
}

// Collide a single particle (given by thread-id through @index) against all spheres in own and neighboring cells
__global__
void collideD(float4* newVel,               // output: new velocities. This is actually mDeviceVel, so its the original velocity location
              float4* oldPos,            // input: positions sorted according to containing grid cell
              float4* oldVel,            // input: velocities sorted according to containing grid cell
              uint*   gridParticleIndex,    // input: particle indices sorted according to containing grid cell
              uint*   cellStart,            // input: cellStart[19] contains the index of gridParticleIndex in which cell 19 starts
              uint*   cellEnd,              // input: cellEnd[19] contains the index of gridParticleIndex in which cell 19 ends
              uint    numParticles)         // input: number of total particles
{
    uint index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
    if (index >= numParticles) return;

    // read particle data from sorted arrays
    float3 pos = make_float3(FETCH(oldPos, index));
    float3 vel = make_float3(FETCH(oldVel, index));

    // get address of particle in grid
    int3 gridPos = calcGridPos(pos);

    // examine neighbouring cells
    float3 force = make_float3(0.0f);
    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 neighbourPos = gridPos + make_int3(x, y, z);
                force += collideCell(neighbourPos, index, pos, vel, oldPos, oldVel, cellStart, cellEnd);
            }
        }
    }

    // collide with cursor sphere
    /* single sphere removed by ben
    force += collideSpheres(
                pos,
                params.colliderPos,
                vel,
                make_float3(0.0f, 0.0f, 0.0f),
                params.particleRadius,
                params.colliderRadius,
                0.0f);*/

    // write new velocity back to original unsorted location
    uint originalIndex = gridParticleIndex[index];
    newVel[originalIndex] = make_float4(vel + force, 0.0f);
}

#endif
