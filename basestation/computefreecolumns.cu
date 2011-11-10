// Kernel definition

extern "C" void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap);

__global__ void kernelComputeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap)
{
int i = threadIdx.x;
int j = threadIdx.y;
//b[i] = A[i] + B[i];
}

void computeFreeColumns(unsigned char* gridPointer, unsigned char* pixmap)
{
    kernelComputeFreeColumns<<<1, 1>>>(gridPointer, pixmap);
}



