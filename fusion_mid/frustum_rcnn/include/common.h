/**
 * @file common.h
 * @brief MACRO for CUDA codes
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>
// headers in STL
#include <stdint.h>
#include <stdio.h>

#include <fstream>
#include <vector>

// headers in CUDA
// #include <cuda_runtime_api.h>

// using MACRO to allocate memory inside CUDA kernel
#define NUM_3D_BOX_CORNERS_MACRO 8
#define NUM_2D_BOX_CORNERS_MACRO 4
#define NUM_THREADS_MACRO 64  // need to be changed when NUM_THREADS is changed

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

#define GPU_CHECK(ans) \
  { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char* file, int line, bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}

template <typename T>
void write_data(const std::vector<T>& data, const std::string& file_name) {
  FILE* fd = fopen(file_name.c_str(), "wb");
  fwrite(data.data(), sizeof(T), data.size(), fd);
  fclose(fd);
}

template <typename T>
void read_data(std::vector<T>& data, const std::string& file_name) {
  FILE* fd = fopen(file_name.c_str(), "rb");
  fread(data.data(), sizeof(T), data.size(), fd);
  fclose(fd);
}

#endif  // COMMON_H