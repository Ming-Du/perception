#include "CenterShiftCustom.h"

// center_shift_cuda
namespace center_shift{
    // pcd_xyzi, (BS, C, N, 1), C -> (x, y, z, intensity, ...)
    // pred_center, (BS, 3)
    // pcd_out, (BS, C, N, 1)
    __global__ void center_shift_kernel(const float* pcd_xyzi, const float* pred_center, float* pcd_out, int64_t BS, int64_t C, int64_t N)
    {
        int64_t bs = blockIdx.x;
        __shared__ float center[3];
        if(threadIdx.x == 0){
            center[0] = pred_center[bs * 3];
            center[1] = pred_center[bs * 3 + 1];
            center[2] = pred_center[bs * 3 + 2];
        }

        __syncthreads();
        for(int64_t n = threadIdx.x; n < N; n = n + blockDim.x){
            int64_t pcd_xyzi_index0 = bs * C * N + n;

            pcd_out[pcd_xyzi_index0] = pcd_xyzi[pcd_xyzi_index0] - center[0];
            pcd_out[pcd_xyzi_index0 + N] = pcd_xyzi[pcd_xyzi_index0 + N] - center[1];
            pcd_out[pcd_xyzi_index0 + 2 * N] = pcd_xyzi[pcd_xyzi_index0 + 2 * N] - center[2];
            for(int64_t c=3; c < C; c++){
                pcd_out[pcd_xyzi_index0 + c * N] = pcd_xyzi[pcd_xyzi_index0 + c * N];
            }
        }
    }
    __global__ void center_shift_kernel_half(const half* pcd_xyzi, const half* pred_center, half* pcd_out, int64_t BS, int64_t C, int64_t N)
    {
        int64_t bs = blockIdx.x;
        __shared__ float center[3];
        if(threadIdx.x == 0){
            center[0] = pred_center[bs * 3];
            center[1] = pred_center[bs * 3 + 1];
            center[2] = pred_center[bs * 3 + 2];
        }

        __syncthreads();
        for(int64_t n = threadIdx.x; n < N; n = n + blockDim.x){
            int64_t pcd_xyzi_index0 = bs * C * N + n;

            pcd_out[pcd_xyzi_index0] =  half(pcd_xyzi[pcd_xyzi_index0].operator float() - center[0]);
            pcd_out[pcd_xyzi_index0 + N] = half(pcd_xyzi[pcd_xyzi_index0 + N].operator float() - center[1]);
            pcd_out[pcd_xyzi_index0 + 2 * N] = half(pcd_xyzi[pcd_xyzi_index0 + 2 * N].operator float() - center[2]);
            for(int64_t c=3; c < C; c++){
                pcd_out[pcd_xyzi_index0 + c * N] = pcd_xyzi[pcd_xyzi_index0 + c * N];
            }
        }
    }
}


void center_shift_cuda(float* pcd_xyzi, float* pred_center, float* pcd_out, int64_t BS, int64_t C, int64_t N)
{
    cudaMemset(pcd_out, 0, sizeof(float) * BS * C * N);
    center_shift::center_shift_kernel<<<BS, 1024>>>(pcd_xyzi, pred_center,pcd_out, BS, C, N);
}
void center_shift_cuda_half(half* pcd_xyzi, half* pred_center, half* pcd_out, int64_t BS, int64_t C, int64_t N)
{
    cudaMemset(pcd_out, 0, sizeof(half) * BS * C * N);
    center_shift::center_shift_kernel_half<<<BS, 1024>>>(pcd_xyzi, pred_center,pcd_out, BS, C, N);
}