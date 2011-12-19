// Kernel definition

extern "C" void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap);

__global__ void kernelComputeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap)
{
int i = threadIdx.x;
int j = threadIdx.y;

pixmap[i] = i*j;//gridPointer[i];
pixmap[i] = i*j;//gridPointer[i];

i = blockDim.x * blockIdx.x + threadIdx.x;

gridPointer[i] = 5;
pixmap[i] = 5;
}

void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap)
{
    //    int numBlocks = 1;
    //    dim3 threadsPerBlock(N, N);
    //    MatAdd<<<numBlocks, threadsPerBlock>>>(A, B, C);

    kernelComputeFreeColumns<<<100, 10>>>(gridPointer, pixmap);
}



