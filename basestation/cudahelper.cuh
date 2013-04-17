#ifndef CUDA_CUH
#define CUDA_CUH

// was 64
#define KERNEL_LAUNCH_BLOCKSIZE 256

__device__ unsigned int getThreadIndex1D(void);


// compute the next higher power of 2 of 32-bit v
__host__ __device__ unsigned int nextHigherPowerOfTwo(unsigned int v);

__host__ __device__ unsigned int cudaBound(const unsigned int min, const unsigned int value, const unsigned int max);

#endif
