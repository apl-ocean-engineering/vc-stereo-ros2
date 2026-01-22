//
//
// Copyright 2025 University of Washington

#include <stdio.h>

#include "vc_argus_ros2/convert.h"

__device__ inline uint8_t clamp(float val, float mn, float mx) {
  return (uint8_t)((val >= mn) ? ((val <= mx) ? val : mx) : mn);
}

__global__ void convert_kernel(CUsurfObject surface1, CUsurfObject surface2,
                               unsigned int width, unsigned int height,
                               const float gamma, uint8_t* out) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int nx = blockDim.x * gridDim.x;
  int ny = blockDim.y * gridDim.y;

  const float inv_gamma = 1.0f / gamma;

  for (int col = x; col < width; col += nx) {
    for (int row = y; row < height; row += ny) {
      uchar1 Ydata, Cbdata, Crdata;
      surf2Dread(&Ydata, surface1, col, row);
      surf2Dread(&Cbdata, surface2, ((int)col / 2) * 2 + 1, (int)(row / 2));
      surf2Dread(&Crdata, surface2, ((int)col / 2) * 2 + 0, (int)(row / 2));

      const float Rint = (Ydata.x + 1.402f * (Crdata.x - 128));
      uint8_t Rval =
          clamp(round(pow(Rint / 255.0, inv_gamma) * 255.0f), 0.0f, 255.0f);

      const float Gint =
          Ydata.x - 0.344136f * (Cbdata.x - 128) - 0.714136 * (Crdata.x - 128);
      uint8_t Gval =
          clamp(round(pow(Gint / 255.0, inv_gamma) * 255.0f), 0.0f, 255.0f);

      const float Bint = Ydata.x + 1.772f * (Cbdata.x - 128);
      uint8_t Bval =
          clamp(round(pow(Bint / 255.0, inv_gamma) * 255.0f), 0.0f, 255.0f);

      out[3 * (row * width + col) + 0] = Rval;
      out[3 * (row * width + col) + 1] = Gval;
      out[3 * (row * width + col) + 2] = Bval;
    }
  }
}

float run_smem_atomics(CUsurfObject surface1, CUsurfObject surface2,
                       unsigned int width, unsigned int height,
                       const float gamma, uint8_t* oBuffer) {
  cudaError_t err = cudaSuccess;
  dim3 block(32, 4);
  dim3 grid(16, 16);

  uint8_t* d_buffer;
  err = cudaMalloc(&d_buffer, 3 * width * height * sizeof(uint8_t));
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to allocate device buffer (%s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  cudaEvent_t start;
  cudaEvent_t stop;
  cudaEventCreate(&stop);
  cudaEventCreate(&start);
  cudaEventRecord(start, 0);

  convert_kernel<<<grid, block>>>(surface1, surface2, width, height, gamma,
                                  d_buffer);
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to launch kernel (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_millis;
  cudaEventElapsedTime(&elapsed_millis, start, stop);

  cudaEventDestroy(start);
  cudaEventDestroy(stop);

  err = cudaMemcpy(oBuffer, d_buffer, 3 * width * height * sizeof(uint8_t),
                   cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to copy into host buffer (%s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  err = cudaFree(d_buffer);
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to free device buffer (%s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  return elapsed_millis;
}

float convertSurfObject(CUsurfObject surface1, CUsurfObject surface2,
                        unsigned int width, unsigned int height, float gamma,
                        uint8_t* oBuffer) {
  return run_smem_atomics(surface1, surface2, width, height, gamma, oBuffer);
}
