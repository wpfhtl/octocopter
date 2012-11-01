// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "thrust/device_ptr.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"
#include "thrust/unique.h"
#include "thrust/remove.h"
#include "thrust/reduce.h"
#include "thrust/tuple.h"

#include <QDebug>

#include "cuda.h"
#include "helper_math.h"
#include "pointcloudcuda.cuh"

// 0 seems to be 10% faster with 256k particles
#define USE_TEX 0

#if USE_TEX
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif

#if USE_TEX
// textures for particle position and velocity
texture<float4, 1, cudaReadModeElementType> oldPointPosTex;
//texture<float4, 1, cudaReadModeElementType> oldVelTex;

//texture<uint, 1, cudaReadModeElementType> gridParticleHashTex;
texture<uint, 1, cudaReadModeElementType> pointCellStartTex;
texture<uint, 1, cudaReadModeElementType> pointCellStoppTex;
#endif

inline __host__ __device__ bool operator!=(float3 &a, float3 &b)
{
    return !(a.x == b.x && a.y == b.y && a.z == b.z);
}

inline __host__ __device__ bool operator!=(float4 &a, float4 &b)
{
    return !(a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w);
}

// pointcloud parameters in constant memory
__constant__ PointCloudParameters params;

// Calculate's a particle's containing cell in the uniform grid
__device__ int3 pcdCalcGridPos(float3 p)
{
    float3 cellSize;
    cellSize.x = (params.bBoxMax.x - params.bBoxMin.x) / params.gridSize.x;
    cellSize.y = (params.bBoxMax.y - params.bBoxMin.y) / params.gridSize.y;
    cellSize.z = (params.bBoxMax.z - params.bBoxMin.z) / params.gridSize.z;

    int3 gridPos;
    gridPos.x = floor((p.x - params.bBoxMin.x) / cellSize.x);
    gridPos.y = floor((p.y - params.bBoxMin.y) / cellSize.y);
    gridPos.z = floor((p.z - params.bBoxMin.z) / cellSize.z);
    return gridPos;
}

// Calculate a particle's hash value (=address in grid) from its containing cell (clamping to edges)
__device__ uint pcdCalcGridHash(int3 gridPos)
{
    gridPos.x = gridPos.x & (params.gridSize.x-1);  // wrap grid, assumes size is power of 2
    gridPos.y = gridPos.y & (params.gridSize.y-1);
    gridPos.z = gridPos.z & (params.gridSize.z-1);
    return __umul24(__umul24(gridPos.z, params.gridSize.y), params.gridSize.x) + __umul24(gridPos.y, params.gridSize.x) + gridPos.x;
}

// Calculate grid hash value for each particle
__global__
void computeMappingFromGridCellToPointD(
        uint*   gridCellIndex,  // output
        uint*   gridPointIndex, // output
        float4* pos,            // input: particle positions
        uint    numPoints)
{
    uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    if(index >= numPoints) return;

    volatile float4 p = pos[index];

    // In which grid cell does the particle live?
    int3 gridPos = pcdCalcGridPos(make_float3(p.x, p.y, p.z));

    // Calculate the particle's hash from the grid-cell. This means particles in the same cell have the same hash
    uint hash = pcdCalcGridHash(gridPos);

    // This array is the key-part of the map, mapping cellId (=hash) to particleIndex. The term "map" is not
    // exactly correct, because there can be multiple keys (because one cell can store many particles)
    gridCellIndex[index] = hash;

    // It seems stupid to fill an array like "array[x]=x". But this array is the value-part of a map and will get sorted according to the keys (=gridParticleHash)
    gridPointIndex[index] = index;
}

// rearrange particle data into sorted order (sorted according to containing grid cell), and find the start of each cell in the sorted hash array
__global__
void sortPosAccordingToGridCellAndFillCellStartAndEndArraysD(
        uint*   pointCellStart,          // output: cell start index
        uint*   pointCellStopp,            // output: cell end index
        float4* sortedPos,          // output: sorted positions, sorted according to the containing gridcell
        uint *  gridCellIndex,      // input: sorted grid hashes
        uint *  gridParticleIndex,  // input: sorted particle indices
        float4* oldPointPos,             // input: UNsorted position array
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
        hash = gridCellIndex[threadIndex];

        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x+1] = hash; // => key of the sorted map

        if(threadIndex > 0 && threadIdx.x == 0)
        {
            // first thread in block must load neighbor particle hash
            sharedHash[0] = gridCellIndex[threadIndex-1];
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
            pointCellStart[hash] = threadIndex;
            if (threadIndex > 0)
                pointCellStopp[sharedHash[threadIdx.x]] = threadIndex;
        }

        if(threadIndex == numParticles - 1)
        {
            pointCellStopp[hash] = threadIndex + 1;
        }

        // Now use the sorted index to reorder the pos and vel data
        uint sortedIndex = gridParticleIndex[threadIndex]; // => value of the sorted map
        float4 pos = FETCH(oldPointPos, sortedIndex);       // macro does either global read or texture fetch,
//        float4 vel = FETCH(oldVel, sortedIndex);       // see particles_kernel.cuh

        // ben: hier if() beenden, dann syncthreads() und dann nicht in sortedPos schreiben, sondern in oldPointPos? Bräuchte ich dann noch zwei pos/vel container?
        sortedPos[threadIndex] = pos;
//        sortedVel[threadIndex] = vel;
    }
}

// collide a particle against all other particles in a given cell
__device__
uint checkCellForNeighborsD(
        int3    gridPos,     // grid cell to search for particles that could collide
        uint    index,       // index of particle that is being collided
        float4  posCollider,         // position of particle that is being collided
        float4* oldPointPos,
        uint*   pointCellStart,
        uint*   pointCellStopp)
{
    uint gridHash = pcdCalcGridHash(gridPos);

    // get start of bucket for this cell
    uint startIndex = FETCH(pointCellStart, gridHash);

    uint numberOfCollisions = 0;

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = FETCH(pointCellStopp, gridHash);
        for(uint j=startIndex; j<endIndex; j++)
        {
            // check not colliding with self
            if (j != index)
            {
                float4 posOther = FETCH(oldPointPos, j);

                float4 posRelative = posCollider - posOther;

                float dist = length(posRelative);

                if(dist < params.minimumDistance)
                {
                    numberOfCollisions++;
                }
            }
        }
    }

    return numberOfCollisions;
}

// Collide a single particle (given by thread-id through @index) against all spheres in own and neighboring cells
__global__
void markCollidingPointsD(
        float4* posOriginal,     // output: new positions, same or zeroed. This is actually mDevicePointPos, so its the original position location
        float4* oldPointPos,          // input: positions sorted according to containing grid cell
        uint*   gridPointIndex,  // input: particle indices sorted according to containing grid cell
        uint*   pointCellStart,       // input: pointCellStart[19] contains the index of gridParticleIndex in which cell 19 starts
        uint*   pointCellStopp,         // input: pointCellStopp[19] contains the index of gridParticleIndex in which cell 19 ends
        uint    numPoints)       // input: number of total particles
{
    uint colliderIndex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
    if(colliderIndex >= numPoints) return;

    // read particle data from sorted arrays
    float4 colliderPos = FETCH(oldPointPos, colliderIndex);
//    float3 vel = make_float3(FETCH(oldVel, index));

    // get address of particle in grid
    int3 gridPos = pcdCalcGridPos(make_float3(colliderPos));

    uint originalIndex = gridPointIndex[colliderIndex];

    uint numberOfCollisions = 0;

    // examine neighbouring cells
    for(int z=-1; z<=1; z++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int x=-1; x<=1; x++)
            {
                int3 neighbourCell = gridPos + make_int3(x, y, z);
                numberOfCollisions += checkCellForNeighborsD(neighbourCell, colliderIndex, colliderPos, oldPointPos, pointCellStart, pointCellStopp);
            }
        }
    }

    if(originalIndex % (5-numberOfCollisions) == 0)
        posOriginal[originalIndex] = make_float4(0.0, 0.0, 0.0, 0.0);
}


void setPointCloudParameters(PointCloudParameters *hostParams)
{
    // copy parameters to constant memory
    checkCudaSuccess("setPointCloudParameters(): CUDA error before const mem copy");

    if(hostParams->remainder == 0)
        hostParams->remainder = 1;
    else
        hostParams->remainder = 0;

    cudaMemcpyToSymbol(params, hostParams, sizeof(PointCloudParameters));
    checkCudaSuccess("setPointCloudParameters(): CUDA error after const mem copy");
}

// Calculates a hash for each particle. The hash value is ("based on") its cell id.
void computeMappingFromGridCellToPoint(
        uint*   gridCellIndex,
        uint*   gridPointIndex,
        float* pos,
        int     numPoints)
{
    uint numThreads, numBlocks;
    computeGridSize(numPoints, 256, numBlocks, numThreads);

    checkCudaSuccess("Kernel execution failed BEFORE computeMappingFromGridCellToPoint");

    // execute the kernel
    computeMappingFromGridCellToPointD<<< numBlocks, numThreads >>>(
                                                                      gridCellIndex,
                                                                      gridPointIndex,
                                                                      (float4*) pos,
                                                                      numPoints);

    // check if kernel invocation generated an error
    checkCudaSuccess("Kernel execution failed: computeMappingFromGridCellToPoint");
}

void sortPosAccordingToGridCellAndFillCellStartAndEndArrays(
        uint*  pointCellStart,
        uint*  pointCellStopp,
        float* sortedPos,
        uint*  gridCellIndex,
        uint*  gridPointIndex,
        float* oldPointPos,
        uint   numPoints,
        uint   numCells)
{
    checkCudaSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays(): cuda error present!");

    uint numThreads, numBlocks;
    computeGridSize(numPoints, 256, numBlocks, numThreads);

    // set all cells to empty
    cudaMemset(pointCellStart, 0xffffffff, numCells*sizeof(uint));

    checkCudaSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays(): failed to set memory");

#if USE_TEX
    cudaBindTexture(0, oldPointPosTex, oldPointPos, numPoints*sizeof(float4));
//    cudaBindTexture(0, oldVelTex, oldVel, numPoints*sizeof(float4));
#endif

    checkCudaSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays(): failed to bind texture");

    // Number of bytes in shared memory that is allocated for each (thread)block.
    uint smemSize = sizeof(uint)*(numThreads+1);

    sortPosAccordingToGridCellAndFillCellStartAndEndArraysD<<< numBlocks, numThreads, smemSize>>>(
                                                                                                       pointCellStart,
                                                                                                       pointCellStopp,
                                                                                                       (float4*) sortedPos,
                                                                                                       gridCellIndex,
                                                                                                       gridPointIndex,
                                                                                                       (float4*) oldPointPos,
                                                                                                       numPoints);

    checkCudaSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays(): kernel failed");

#if USE_TEX
    cudaUnbindTexture(oldPointPosTex);
//    cudaUnbindTexture(oldVelTex);
#endif
}



void markCollidingPoints(
        float* posOriginal,
        float* posSorted,
        unsigned int*  gridPointIndex,
        unsigned int*  pointCellStart,
        unsigned int*  pointCellStopp,
        unsigned int   numPoints,
        unsigned int   numCells)
{
#if USE_TEX
    cudaBindTexture(0, oldPointPosTex, posSorted, numPoints*sizeof(float4));
//    cudaBindTexture(0, oldVelTex, sortedVel, numPoints*sizeof(float4));
    cudaBindTexture(0, pointCellStartTex, pointCellStart, numCells*sizeof(uint));
    cudaBindTexture(0, pointCellStoppTex, pointCellStopp, numCells*sizeof(uint));
#endif

    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(numPoints, 64, numBlocks, numThreads);

    // execute the kernel
    markCollidingPointsD<<< numBlocks, numThreads >>>(
                                               (float4*)posOriginal,
                                               (float4*)posSorted,
                                               gridPointIndex,
                                               pointCellStart,
                                               pointCellStopp,
                                               numPoints
                                               );

    // check if kernel invocation generated an error
    checkCudaSuccess("Kernel execution failed: markCollidingPoints");

#if USE_TEX
    cudaUnbindTexture(oldPointPosTex);
//    cudaUnbindTexture(oldVelTex);
    cudaUnbindTexture(pointCellStartTex);
    cudaUnbindTexture(pointCellStoppTex);
#endif
}



// bounding box type
typedef thrust::pair<float4, float4> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox,bbox,bbox>
{
    __host__ __device__
    bbox operator()(bbox a, bbox b)
    {
        // min corner
        float4 min = make_float4(thrust::min(a.first.x, b.first.x), thrust::min(a.first.y, b.first.y), thrust::min(a.first.z, b.first.z), 0);

        // max corner
        float4 max = make_float4(thrust::max(a.second.x, b.second.x), thrust::max(a.second.y, b.second.y), thrust::max(a.second.z, b.second.z), 0);

        return bbox(min, max);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation : public thrust::unary_function<float4,bbox>
{
    __host__ __device__
    bbox operator()(float4 point)
    {
        return bbox(point, point);
    }
};

void getBoundingBox(float *dPoints, uint numPoints, float3& min, float3& max)
{
    float4* points = (float4*)dPoints;

    // wrap raw pointer with a device_ptr
    thrust::device_ptr<float4> dev_ptr = thrust::device_pointer_cast(points);

    bbox init = bbox(dev_ptr[0], dev_ptr[0]);

    // initial bounding box contains first point - does this execute on host? If yes, how can dPoints[0] work?
//    bbox init = bbox(points[0], points[0]);
//    bbox init = bbox(thrust::device_ptr<float4>(dPoints)[0], thrust::device_ptr<float4>(dPoints)[0]);

    // transformation operation
    bbox_transformation opConvertPointToBoundingBox;

    // binary reduction operation
    bbox_reduction opUnifyBoundingBoxes;

    // compute the bounding box for the point set
    bbox result = thrust::transform_reduce(
                thrust::device_ptr<float4>(points),
                thrust::device_ptr<float4>(points + numPoints),
                opConvertPointToBoundingBox,
                init,
                opUnifyBoundingBoxes);

    min = make_float3(result.first);
    max = make_float3(result.second);
}

void sortMapAccordingToKeys(uint *dGridCellIndex, uint *dGridPointIndex, uint numPoints)
{
    checkCudaSuccess("Kernel execution failed BEFORE sortMapAccordingToKeys");

    thrust::sort_by_key(thrust::device_ptr<uint>(dGridCellIndex),                // KeysBeginning
                        thrust::device_ptr<uint>(dGridCellIndex + numPoints),    // KeysEnd
                        thrust::device_ptr<uint>(dGridPointIndex));              // ValuesBeginning

    checkCudaSuccess("Kernel execution failed: sortMapAccordingToKeys");
}

inline __host__ __device__ bool operator==(float4 a, float4 b)
{
    return
            a.x == b.x &&
            a.y == b.y &&
            a.z == b.z &&
            a.w == b.w;
}

unsigned int removeZeroPoints(float *devicePoints, unsigned int numPoints)
{
    float4* points = (float4*)devicePoints;

    checkCudaSuccess("Kernel execution failed BEFORE removeRedundantPoints");

    const thrust::device_ptr<float4> newEnd = thrust::remove(thrust::device_ptr<float4>(points), thrust::device_ptr<float4>(points + numPoints), make_float4(0.0, 0.0, 0.0, 0.0));

    checkCudaSuccess("Kernel execution failed AFTER removeRedundantPoints");

    return newEnd.get() - points;
}


// ##########################################
// This is a test to use thrust::unqiue on the point list instead of all the magic we use above
// ##########################################

struct SnapToGridOp
{
    const float3 mMinDist;

    // if params.minimumDistance is e.g. 0.02 = 2cm, factor should be 50.
    // if params.minimumDistance is e.g. 0.10 = 10cm, factor should be 10.
    SnapToGridOp(float3 minDist) : mMinDist(1.0 / minDist) { }

    __host__ __device__
    float4 operator()(float4 a)
    {
        return make_float4(
                    ceilf(a.x * mMinDist.x) / mMinDist.x,
                    ceilf(a.y * mMinDist.y) / mMinDist.y,
                    ceilf(a.z * mMinDist.z) / mMinDist.z,
                    1.0f);
    }
};

struct closeToEachOther
{
    __host__ __device__
    bool operator()(float4 a, float4 b)
    {
        return length(b-a) < 0.8f;
    }
};


unsigned int snapToGridAndMakeUnique(float *devicePoints, unsigned int numPoints, float minimumDistance)
{
    float4* points = (float4*)devicePoints;
    checkCudaSuccess("Kernel execution failed BEFORE makePointListUnique");

    SnapToGridOp op(make_float3(minimumDistance, minimumDistance*2.0f, minimumDistance));
    thrust::transform(thrust::device_ptr<float4>(points), thrust::device_ptr<float4>(points + numPoints), thrust::device_ptr<float4>(points), op);

    const thrust::device_ptr<float4> newEnd = thrust::unique(thrust::device_ptr<float4>(points), thrust::device_ptr<float4>(points + numPoints)/*, closeToEachOther()*/);

    checkCudaSuccess("Kernel execution failed AFTER makePointListUnique");

    return newEnd.get() - points;
}











__global__ void replaceCellPointsByMeanValueD(
        float4* devicePoints,
        float4* devicePointsSorted,
        uint* pointCellStart,
        uint* pointCellStopp,
        uint* gridCellIndex,
        uint* gridPointIndex,
        uint numPoints,
        uint numCells)
{
    uint cellIndex = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

    // get start of bucket for this cell
    uint startIndex = FETCH(pointCellStart, cellIndex);

    float4 pointAverage = make_float4(0.0, 0.0, 0.0, 0.0);

    // cell is not empty
    if(startIndex != 0xffffffff)
    {
        // iterate over particles in this cell
        uint endIndex = FETCH(pointCellStopp, cellIndex);
        for(uint j=startIndex; j<endIndex; j++)
        {
            pointAverage += FETCH(devicePointsSorted, j);
        }
        pointAverage /= endIndex-startIndex;
    }

    devicePoints[cellIndex] = pointAverage;
}


unsigned int replaceCellPointsByMeanValue(float *devicePoints, float* devicePointsSorted, unsigned int *pointCellStart, unsigned int *pointCellStopp, unsigned int *gridCellIndex, unsigned int *gridPointIndex, unsigned int numPoints, unsigned int numCells)
{
    // The points are already sorted according to the containing cell, cellStart and cellStopp are up to date
    // Start a thread for every CELL, reading all its points, creating a mean-value. Write this value into devicePoints[threadid]

    checkCudaSuccess("replaceCellPointsByMeanValue(): cuda error present!");

    uint numThreads, numBlocks;
    computeGridSize(numCells, 256, numBlocks, numThreads);

    qDebug() << "replaceCellPointsByMeanValue(): using" << numCells << "threads to create as many averaged points";

    replaceCellPointsByMeanValueD<<< numBlocks, numThreads>>>(
                                                                (float4*)devicePoints,
                                                                (float4*)devicePointsSorted,
                                                                pointCellStart,
                                                                pointCellStopp,
                                                                gridCellIndex,
                                                                gridPointIndex,
                                                                numPoints,
                                                                numCells);

    checkCudaSuccess("sortPosAccordingToGridCellAndFillCellStartAndEndArrays(): kernel failed");

    // Every cell created a point, empty cells create zero-points. Delete those.
    return removeZeroPoints(devicePoints, numCells);
}
