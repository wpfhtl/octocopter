#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

// Fix for gcc 4.7
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <cstdlib>
#include <cstdio>
#include <string.h>

#include <cuda_gl_interop.h>

#include "thrust/device_ptr.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

void checkCudaSuccess(const char *errorMessage);

void *mapGLBufferObject(struct cudaGraphicsResource **cuda_vbo_resource);

void copyArrayFromDevice(void* host, const void* device, struct cudaGraphicsResource **cuda_vbo_resource, int size);

//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b);

// compute grid and thread block size for a given number of elements
void computeExecutionKernelGrid(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

#endif
