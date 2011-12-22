#include <cuda_runtime.h>
#include <QDebug>

// Kernel definition
extern "C" void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap, int x, int y, int z);

__global__ void kernelComputeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap, int x, int y, int z)
{
int i = threadIdx.x;
int j = threadIdx.y;

//pixmap[i] = i*j;//gridPointer[i];
//pixmap[i] = i*j;//gridPointer[i];

i = blockDim.x * blockIdx.x + threadIdx.x;

//gridPointer[i] = i;
pixmap[i] = gridPointer[i];
}

void computeFreeColumns(unsigned char* grid, unsigned char* pixmap, int x, int y, int z)
{
    cudaError_t mCudaError;

    //    int numBlocks = 1;
    //    dim3 threadsPerBlock(N, N);
    //    MatAdd<<<numBlocks, threadsPerBlock>>>(A, B, C);

    // gridPointer points to the host's grid. We need the pointer to the device's version of that data.
    quint8* deviceVolumeDataBasePointer = 0;
    mCudaError = cudaHostGetDevicePointer(&deviceVolumeDataBasePointer, grid, 0);
    if(deviceVolumeDataBasePointer == 0) qDebug("couldn't get device pointer for volume data");
    if(mCudaError != cudaSuccess) qDebug("couldn't get device pointer for volume data: %s", cudaGetErrorString(mCudaError));

    unsigned char *pixmapDevice;
    cudaMalloc((void**)&pixmapDevice, x*z);

    kernelComputeFreeColumns<<<16, 64>>>(deviceVolumeDataBasePointer, pixmapDevice, x, y, z);

    cudaMemcpy(pixmap, pixmapDevice, x*z, cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
    cudaThreadSynchronize();

    qDebug() << "grid:";
    for(int i=0;i<(x*y*z)/8;i++) printf("%d ", grid[i]);
    fflush(stdout);

    qDebug() << "\npixmap:";
    for(int i=0;i<x*z;i++) printf("%d ", pixmap[i]);
    fflush(stdout);

    cudaFree(pixmapDevice);
}



