#ifndef CUDA_CUH
#define CUDA_CUH

// was 64
#define KERNEL_LAUNCH_BLOCKSIZE 256

__device__ unsigned int getThreadIndex1D(void);

#endif
