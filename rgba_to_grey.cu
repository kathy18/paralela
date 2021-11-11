#include "utils.h"
#include <stdio.h>
#include <math.h>       /* ceil */

// Max Threads per block in GeForce 210
#define TxB 512

__global__
void rgba_to_grey_kernel(const uchar4* const rgbaImage,
                       unsigned char* const greyImage,
                       int numRows, int numCols)
{
  
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  uchar4 px = rgbaImage[i]; // thread pixel to process
  greyImage[i] = .299f * px.x +
                 .587f * px.y +
                 .114f * px.z;
}

void rgba_to_grey(uchar4 * const d_rgbaImage,
                  unsigned char* const d_greyImage, size_t numRows, size_t numCols)
{

  
  long long int total_px = numRows * numCols;  // total pixels
  long int grids_n = ceil(total_px / TxB); // grids numer
  const dim3 blockSize(TxB, 1, 1);
  const dim3 gridSize(grids_n, 1, 1);
  rgba_to_grey_kernel<<<gridSize, blockSize>>>(d_rgbaImage, d_greyImage, numRows, numCols);
  
  cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());
}
