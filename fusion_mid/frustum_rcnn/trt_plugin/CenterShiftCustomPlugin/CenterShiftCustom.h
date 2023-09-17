#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include "cuda_fp16.h"

void center_shift_cuda(float* pcd_xyzi, float* pred_center, float* pcd_out, int64_t BS, int64_t C, int64_t N);

void center_shift_cuda_half(half* pcd_xyzi, half* pred_center, half* pcd_out, int64_t BS, int64_t C, int64_t N);