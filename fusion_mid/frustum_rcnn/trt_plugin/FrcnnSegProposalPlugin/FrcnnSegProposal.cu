#include "FrcnnSegProposal.h"

namespace frcnn_seg{
    // pcd_xyzi, (BS, C, N, 1), C -> (x, y, z, intensity, ...)
    // pred_seg, (BS, 1, N, 1)
    // pcd_center, (BS, 3)
    // pcd_out, (BS, C, N_out, 1)
    template<typename T>
    __global__ void frcnn_seg_proposal_kernel(const T* pcd_xyzi, const T* pred_seg,
        T* pcd_center, T* pcd_out, int64_t BS, int64_t C, int64_t N, int64_t N_out)
    {
        int64_t bs = blockIdx.x;
        __shared__ float count[1];
        __shared__ float center[3];
        if(threadIdx.x == 0){
            count[0] = 0;
            center[0] = 0;
            center[1] = 0;
            center[2] = 0;
        }
        __syncthreads();

        for(int64_t n = threadIdx.x; n < N; n = n + blockDim.x){
            int64_t pred_seg_index = bs * N + n;
            if(pred_seg[pred_seg_index] > 0.5){
                float old = atomicAdd(count, 1.0f);

                int64_t pcd_xyzi_index0 = bs * C * N + n;
                atomicAdd(&center[0], pcd_xyzi[pcd_xyzi_index0]);
                atomicAdd(&center[1], pcd_xyzi[pcd_xyzi_index0 + N]);
                atomicAdd(&center[2], pcd_xyzi[pcd_xyzi_index0 + 2 * N]);

                int64_t valid_point_num = int64_t(old + 0.001f);
                if(valid_point_num < N_out){
                    int64_t pcd_out_index0 = bs * C * N_out + valid_point_num;
                    for(int64_t c=0; c < C; c++){
                        pcd_out[pcd_out_index0 + c * N_out] = pcd_xyzi[pcd_xyzi_index0 + c * N];
                    }
                }
            }
        }

        __syncthreads();
        if(count[0] > 0){
            // center norm
            if(threadIdx.x == 0){
                center[0] = center[0] / count[0];
                center[1] = center[1] / count[0];
                center[2] = center[2] / count[0];
            }
            __syncthreads();

            pcd_center[bs * 3] = center[0];
            pcd_center[bs * 3 + 1] = center[1];
            pcd_center[bs * 3 + 2] = center[2];
            for(int64_t n = threadIdx.x; (n < count[0]) && (n < N_out); n = n + blockDim.x){
                int64_t pcd_out_index0 = bs * C * N_out + n;
                pcd_out[pcd_out_index0] -= center[0];
                pcd_out[pcd_out_index0 + 1 * N_out] -= center[1];
                pcd_out[pcd_out_index0 + 2 * N_out] -= center[2];
            }

            __syncthreads();
            int64_t count_num = int64_t(count[0] + 0.001f);
            // padding
            int64_t padding_length = N_out - count_num;
            if(padding_length > 0){
                for(int64_t n = threadIdx.x; n < padding_length; n = n + blockDim.x){
                    int64_t pcd_out_index0 = bs * C * N_out + n + count_num;
                    int64_t pcd_out_copy_index0 = bs * C * N_out + (n % count_num);
                    for(int64_t c=0; c < C; c++){
                        pcd_out[pcd_out_index0 + c * N_out] = pcd_out[pcd_out_copy_index0 + c * N_out];
                    }
                }
            }
        }
    }

    __global__ void frcnn_seg_proposal_kernel_half(const half* pcd_xyzi, const half* pred_seg,
        half* pcd_center, half* pcd_out, int64_t BS, int64_t C, int64_t N, int64_t N_out)
    {
        int64_t bs = blockIdx.x;
        __shared__ float count[1];
        __shared__ float center[3];
        if(threadIdx.x == 0){
            count[0] = 0;
            center[0] = 0;
            center[1] = 0;
            center[2] = 0;
        }
        __syncthreads();

        for(int64_t n = threadIdx.x; n < N; n = n + blockDim.x){
            int64_t pred_seg_index = bs * N + n;
            if(pred_seg[pred_seg_index].operator float() > 0.5f){
                float old = atomicAdd(count, 1.0f);

                int64_t pcd_xyzi_index0 = bs * C * N + n;
                atomicAdd(&center[0], pcd_xyzi[pcd_xyzi_index0].operator float());
                atomicAdd(&center[1], pcd_xyzi[pcd_xyzi_index0 + N].operator float());
                atomicAdd(&center[2], pcd_xyzi[pcd_xyzi_index0 + 2 * N].operator float());

                int64_t valid_point_num = int64_t(old + 0.001f);
                if(valid_point_num < N_out){
                    int64_t pcd_out_index0 = bs * C * N_out + valid_point_num;
                    for(int64_t c=0; c < C; c++){
                        pcd_out[pcd_out_index0 + c * N_out] = pcd_xyzi[pcd_xyzi_index0 + c * N];
                    }
                }
            }
        }

        __syncthreads();
        if(count[0] > 0){
            // center norm
            if(threadIdx.x == 0){
                center[0] = center[0] / count[0];
                center[1] = center[1] / count[0];
                center[2] = center[2] / count[0];
            }
            __syncthreads();

            pcd_center[bs * 3] = half(center[0]);
            pcd_center[bs * 3 + 1] = half(center[1]);
            pcd_center[bs * 3 + 2] = half(center[2]);
            for(int64_t n = threadIdx.x; (n < count[0]) && (n < N_out); n = n + blockDim.x){
                int64_t pcd_out_index0 = bs * C * N_out + n;
                pcd_out[pcd_out_index0] = half( pcd_out[pcd_out_index0].operator float() - center[0] );
                // pcd_out[pcd_out_index0] -= center[0];
                pcd_out[pcd_out_index0 + 1 * N_out] = half(pcd_out[pcd_out_index0 + 1 * N_out].operator float() - center[1]);
                // pcd_out[pcd_out_index0 + 1 * N_out] -= center[1];
                 pcd_out[pcd_out_index0 + 2 * N_out] = half( pcd_out[pcd_out_index0 + 2 * N_out].operator float() - center[2]);
                // pcd_out[pcd_out_index0 + 2 * N_out] -= center[2];
            }

            __syncthreads();
            int64_t count_num = int64_t(count[0] + 0.001f);
            // padding
            int64_t padding_length = N_out - count_num;
            if(padding_length > 0){
                for(int64_t n = threadIdx.x; n < padding_length; n = n + blockDim.x){
                    int64_t pcd_out_index0 = bs * C * N_out + n + count_num;
                    int64_t pcd_out_copy_index0 = bs * C * N_out + (n % count_num);
                    for(int64_t c=0; c < C; c++){
                        pcd_out[pcd_out_index0 + c * N_out] = pcd_out[pcd_out_copy_index0 + c * N_out];
                    }
                }
            }
        }
    }

}


void frcnn_seg_proposal_cuda(float* pcd_xyzi, float* pred_seg, float* pcd_center, float* pcd_out,
int64_t BS, int64_t C, int64_t N, int64_t N_out, cudaStream_t stream)
{
    cudaMemset(pcd_center, 0, sizeof(float) * BS * 3);
    cudaMemset(pcd_out, 0, sizeof(float) * BS * C * N_out);
    frcnn_seg::frcnn_seg_proposal_kernel<float><<<BS, 1024>>>(pcd_xyzi, pred_seg, pcd_center, pcd_out, BS, C, N, N_out);
}

void frcnn_seg_proposal_cuda_half(half* pcd_xyzi, half* pred_seg, half* pcd_center, half* pcd_out,
int64_t BS, int64_t C, int64_t N, int64_t N_out, cudaStream_t stream)
{
    cudaMemset(pcd_center, 0, sizeof(half) * BS * 3);
    cudaMemset(pcd_out, 0, sizeof(half) * BS * C * N_out);
    frcnn_seg::frcnn_seg_proposal_kernel_half<<<BS, 1024>>>(pcd_xyzi, pred_seg, pcd_center, pcd_out, BS, C, N, N_out);
}