#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <cstdlib>
#include <cstdio>
#include <string.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <QDebug>

#define CUDA_ERROR_CHECK
#define cudaSafeCall(err) __cudaSafeCall( err, __FILE__, __LINE__ )
#define cudaCheckSuccess(src)  __cudaCheckSuccess( src, __FILE__, __LINE__ )

inline void __cudaSafeCall(cudaError err, const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
    if(cudaSuccess != err)
    {
        qFatal("cudaSafeCall() failed at %s:%i : %s\n", file, line, cudaGetErrorString(err));
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
        qFatal("cudaCheckError() %s failed at %s:%i : %s", errorSource, file, line, cudaGetErrorString(err));
    }

    // More careful checking, using sync (will slow down performance)
    err = cudaDeviceSynchronize();
    if(cudaSuccess != err)
    {
        qFatal("cudaCheckError() with sync failed at %s:%i : %s", file, line, cudaGetErrorString(err));
    }
#endif

    return;
}

void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource);

void copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size);

//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b);

// compute grid and thread block size for a given number of elements
void computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

#endif
