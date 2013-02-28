#ifndef CUDA_CU
#define CUDA_CU

#include "cudahelper.cuh"

__device__ unsigned int getThreadIndex1D(void)
{
  return blockIdx.x * blockDim.x + threadIdx.x;
}

#endif
