#ifndef CUDA_CUH
#define CUDA_CUH

#include  <iostream>

// was 64
#define KERNEL_LAUNCH_BLOCKSIZE 256


#define CUDA_ERROR_CHECK
#define cudaSafeCall(err) __cudaSafeCall( err, __FILE__, __LINE__ )
#define cudaCheckSuccess(src)  __cudaCheckSuccess( src, __FILE__, __LINE__ )

inline void __cudaSafeCall(cudaError err, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
    if(cudaSuccess != err)
    {
        std::cout << "cudaSafeCall(): failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        std::cout.flush();
        abort();
    }
#endif
    return;
}

inline void __cudaCheckSuccess(const char *errorSource, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
    cudaError err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        std::cout << "cudaCheckError(): " << errorSource << " failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        std::cout.flush();
        abort();
    }

    // More careful checking, using sync (will slow down performance)
    err = cudaDeviceSynchronize();
    if(cudaSuccess != err)
    {
        std::cout << "cudaCheckError(): with sync failed at " << file << ":" << line << ": " << cudaGetErrorString(err);
        std::cout.flush();
        abort();
    }
#endif

    return;
}

// Round a / b to nearest higher integer value
__host__ __device__ uint /*CudaHelper::*/iDivUp(uint a, uint b);

// compute grid and thread block size for a given number of elements
__host__ __device__ /*static*/ void computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

__device__ unsigned int getThreadIndex1D(void);

// compute the next higher power of 2 of 32-bit v
__host__ __device__ unsigned int nextHigherPowerOfTwo(unsigned int v);

__host__ __device__ unsigned int cudaBound(const unsigned int min, const unsigned int value, const unsigned int max);

#endif
