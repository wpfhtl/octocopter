#ifndef CUDA_CU
#define CUDA_CU

#include "cudahelper.cuh"

__device__ unsigned int getThreadIndex1D(void)
{
  return blockIdx.x * blockDim.x + threadIdx.x;
}


// compute the next higher power of 2 of 32-bit v
__host__ __device__ unsigned int nextHigherPowerOfTwo(unsigned int v)
{
    // decrements, then sets all bits below its most significant bit to 1, then it increments
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}

__host__ __device__ unsigned int cudaBound(const unsigned int min, const unsigned int value, const unsigned int max)
{
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}

__host__ __device__ uint /*CudaHelper::*/iDivUp(uint a, uint b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// Compute grid and thread block size for a given number of elements
__host__ __device__ void /*CudaHelper::*/computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads)
{
    numThreads = blockSize < n ? blockSize : n;// std::min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}



#endif
