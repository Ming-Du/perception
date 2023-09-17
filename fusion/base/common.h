
#pragma once

#include <cassert>

#if USE_GPU == 1

#include <cublas_v2.h>
#include <cuda_runtime.h>

#endif

namespace perception {
namespace fusion {

#ifndef NO_GPU
#define NO_GPU assert(false)
#endif

#if USE_GPU == 1

#define BASE_CUDA_CHECK(condition) \
  { perception::base::GPUAssert((condition), __FILE__, __LINE__); }

inline void GPUAssert(cudaError_t code, const char* file, int line, bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) {
      exit(code);
    }
  }
}

#endif

}  // namespace fusion
}  // namespace perception
