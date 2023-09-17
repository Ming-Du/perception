#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include "cuda_fp16.h"

void frcnn_seg_proposal_cuda(float* pcd_xyzi, float* pred_seg, float* pcd_center, float* pcd_out,
int64_t BS, int64_t C, int64_t N, int64_t N_out, cudaStream_t stream);


void frcnn_seg_proposal_cuda_half(half* pcd_xyzi, half* pred_seg, half* pcd_center, half* pcd_out,
int64_t BS, int64_t C, int64_t N, int64_t N_out, cudaStream_t stream);