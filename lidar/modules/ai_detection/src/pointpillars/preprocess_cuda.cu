/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "common/base_cuda_check.h"
#include "pointpillars/preprocess_cuda.h"

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))
#define TEST_CRASH 0
namespace robosense {
__global__ void make_pillar_histo_kernel(
    const pcl::PointXYZI* in_cloud,
    const int num_points,
    const uint8_t* valid_flag,
    float* dev_pillar_x_in_coors,
    float* dev_pillar_y_in_coors,
    float* dev_pillar_z_in_coors,
    float* dev_pillar_i_in_coors,
    int* points_count,
    const float min_x_range,
    const float min_y_range,
    const float min_z_range,
    const float pillar_x_size,
    const float pillar_y_size,
    const float pillar_z_size,
    const int grid_x_size,
    const int grid_y_size,
    const int grid_z_size,
    const int max_num_points_per_pillar) {
    int th_i = threadIdx.x + blockIdx.x * blockDim.x;
    if (valid_flag[th_i] <= 0) {
        return;
    }
    if (th_i >= num_points) {
        return;
    }
    pcl::PointXYZI p = in_cloud[th_i];
    float px = p.x;
    float py = p.y;
    float pz = p.z;
    int x_coor = floor((px - min_x_range) / pillar_x_size);
    int y_coor = floor((py - min_y_range) / pillar_y_size);
    int z_coor = floor((pz - min_z_range) / pillar_z_size);

    if (x_coor >= 0 && x_coor < grid_x_size &&
        y_coor >= 0 && y_coor < grid_y_size &&
        z_coor >= 0 && z_coor < grid_z_size) {
        int count = atomicAdd(&points_count[y_coor * grid_x_size + x_coor], 1);
        if (count < max_num_points_per_pillar) {
            int ind = y_coor * grid_x_size * max_num_points_per_pillar + x_coor * max_num_points_per_pillar + count;
            dev_pillar_x_in_coors[ind] = px;
            dev_pillar_y_in_coors[ind] = py;
            dev_pillar_z_in_coors[ind] = pz;
            dev_pillar_i_in_coors[ind] = p.intensity / 255.;
        }
    }
}

__global__ void make_pillar_index_kernel_h(
    int* points_count,
    int* pillar_counter,
    float* x_coors,
    float* y_coors,
    float* x_coors_for_sub,
    float* y_coors_for_sub,
    float* num_points_per_pillar,
    const int grid_x_size,
    const float min_x_range,
    const float min_y_range,
    const int max_num_pillars,
    const int max_num_points_per_pillar,
    const float pillar_x_size,
    const float pillar_y_size,
    const float pillar_z_size) {
    int x = blockIdx.x;
    int y = threadIdx.x;

    int num_points_at_this_pillar = points_count[y * grid_x_size + x];
    if (num_points_at_this_pillar == 0) {
        return;
    }
    // assert(false);//run
    // int a = 1/0;x_coors[(y * grid_x_size + x)]=x_coors[(y * grid_x_size + x)]+a;//run
    // int *p;x_coors[(y * grid_x_size + x)]=p[(y * grid_x_size + x)]+1;//crash

    int count = atomicAdd(pillar_counter, 1);
    if (count < max_num_pillars) {
        if (num_points_at_this_pillar >= max_num_points_per_pillar) {
            num_points_per_pillar[count] = max_num_points_per_pillar;
        } else {
            num_points_per_pillar[count] = num_points_at_this_pillar;
        }
        x_coors[count] = x;
        y_coors[count] = y;
        x_coors_for_sub[count] = x * pillar_x_size + pillar_x_size / 2 + min_x_range;
        y_coors_for_sub[count] = y * pillar_y_size + pillar_y_size / 2 + min_y_range;

        // z_coors_for_sub[count] = PILLAR_Z_SIZE / 2 + MIN_Z_RANGE;
    }
}

// <<< (32,42887/512),512  >>>
__global__ void make_pillar_feature_kernel_h(
    float* dev_pillar_x_in_coors,
    float* dev_pillar_y_in_coors,
    float* dev_pillar_z_in_coors,
    float* dev_pillar_i_in_coors,
    float* pillar_features,
    float* dev_x_sum,
    float* dev_y_sum,
    float* dev_z_sum,
    float* dev_x_coors,
    float* dev_y_coors,
    float* dev_num_points_per_pillar,
    const int pillar_bev,
    const int max_num_points_per_pillar,
    const int grid_x_size,
    int max_pillars_num) {
    int ith_pillar = blockIdx.y * blockDim.x + threadIdx.x;
    if (ith_pillar >= max_pillars_num) {
        return;
    }
    int num_points_at_this_pillar = dev_num_points_per_pillar[ith_pillar];
    int ith_point = blockIdx.x;
    if (ith_point >= num_points_at_this_pillar) {
        return;
    }
    int x_ind = dev_x_coors[ith_pillar];
    int y_ind = dev_y_coors[ith_pillar];
    int point_at_pillar_ind = ith_pillar * max_num_points_per_pillar + ith_point;
    int coors_ind = y_ind * grid_x_size * max_num_points_per_pillar + x_ind * max_num_points_per_pillar + ith_point;
    pillar_features[point_at_pillar_ind] = dev_pillar_x_in_coors[coors_ind];
    pillar_features[point_at_pillar_ind + pillar_bev] = dev_pillar_y_in_coors[coors_ind];
    pillar_features[point_at_pillar_ind + 2 * pillar_bev] = dev_pillar_z_in_coors[coors_ind];
    pillar_features[point_at_pillar_ind + 3 * pillar_bev] = dev_pillar_i_in_coors[coors_ind];

    atomicAdd(&dev_x_sum[ith_pillar], dev_pillar_x_in_coors[coors_ind]);
    atomicAdd(&dev_y_sum[ith_pillar], dev_pillar_y_in_coors[coors_ind]);
    atomicAdd(&dev_z_sum[ith_pillar], dev_pillar_z_in_coors[coors_ind]);
}
__global__ void crashKernel() {
    int* ptr = nullptr;
    *ptr = 0;
}

__global__ void make_pillar_mean_feature_kernel(
    float* pillar_features,
    float* dev_x_coors_for_sub,
    float* dev_y_coors_for_sub,
    float* dev_x_sum_per_pillar,
    float* dev_y_sum_per_pillar,
    float* dev_z_sum_per_pillar,
    float* dev_num_points_per_pillar,
    const float min_z_range,
    const float pillar_z_size,
    const int pillar_bev,
    const int max_num_points_per_pillar) {
    int ith_pillar = blockIdx.x;
    int ith_point = threadIdx.x;

    int num_points_for_a_pillar = dev_num_points_per_pillar[ith_pillar];
    int ind = ith_pillar * max_num_points_per_pillar + ith_point;
    if (ith_point >= num_points_for_a_pillar) {
        pillar_features[ind + 4 * pillar_bev] = 0;
        pillar_features[ind + 5 * pillar_bev] = 0;
        pillar_features[ind + 6 * pillar_bev] = 0;
        pillar_features[ind + 7 * pillar_bev] = 0;
        pillar_features[ind + 8 * pillar_bev] = 0;
        pillar_features[ind + 9 * pillar_bev] = 0;
    } else {
        float x = dev_x_coors_for_sub[ith_pillar];
        float y = dev_y_coors_for_sub[ith_pillar];
        float z = pillar_z_size / 2 + min_z_range;
        float x_mean = dev_x_sum_per_pillar[ith_pillar] / (num_points_for_a_pillar + 1e-8);
        float y_mean = dev_y_sum_per_pillar[ith_pillar] / (num_points_for_a_pillar + 1e-8);
        float z_mean = dev_z_sum_per_pillar[ith_pillar] / (num_points_for_a_pillar + 1e-8);

        pillar_features[ind + 4 * pillar_bev] = pillar_features[ind] - x_mean;
        pillar_features[ind + 5 * pillar_bev] = pillar_features[ind + 1 * pillar_bev] - y_mean;
        pillar_features[ind + 6 * pillar_bev] = pillar_features[ind + 2 * pillar_bev] - z_mean;
        pillar_features[ind + 7 * pillar_bev] = pillar_features[ind] - x;
        pillar_features[ind + 8 * pillar_bev] = pillar_features[ind + 1 * pillar_bev] - y;
        pillar_features[ind + 9 * pillar_bev] = pillar_features[ind + 2 * pillar_bev] - z;
    }
}

void PointPillarsCenterHeadCudaPreprocess::allocateMemory() {
    const int& bev_size = params_ptr_->cols * params_ptr_->rows;
    const int& max_num_pillars = params_ptr_->max_num_pillars;
    const int& max_num_points_per_pillar = params_ptr_->max_num_points_per_pillar;
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&pc_dev_, max_pc_nums_ * sizeof(pcl::PointXYZI)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&valid_flag_dev_, max_pc_nums_ * sizeof(uint8_t)));

    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&dev_pillar_x_in_coors_,
                               bev_size * max_num_points_per_pillar * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&dev_pillar_y_in_coors_,
                               bev_size * max_num_points_per_pillar * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&dev_pillar_z_in_coors_,
                               bev_size * max_num_points_per_pillar * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&dev_pillar_i_in_coors_,
                               bev_size * max_num_points_per_pillar * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&point_count_, bev_size * sizeof(int)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&pillar_counter_, sizeof(int)));
    // TV_CHECK_CUDA_ERR_V3(cudaMalloc((void **)&valid_pillar_count_, sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&x_sum_per_pillar_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&y_sum_per_pillar_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&z_sum_per_pillar_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&x_coors_for_sub_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&y_coors_for_sub_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&z_coors_for_sub_, max_num_pillars * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&num_points_per_pillar_, max_num_pillars * sizeof(float)));

    TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&cpy_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&pre_stream_));
}

void PointPillarsCenterHeadCudaPreprocess::calFeatures() {
    const auto& cloud_ptr = lidar_msg_ptr_->scan_ptr;
    const auto& pc_size = cloud_ptr->size();
    if (max_pc_nums_ < pc_size) {
        TV_CHECK_CUDA_ERR_V3(cudaFree(pc_dev_));
        TV_CHECK_CUDA_ERR_V3(cudaFree(valid_flag_dev_));

        max_pc_nums_ = pc_size;
        TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&pc_dev_, max_pc_nums_ * sizeof(pcl::PointXYZI)));
        TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&valid_flag_dev_, max_pc_nums_ * sizeof(uint8_t)));
    }

    // copy pointcloud to device
    TV_CHECK_CUDA_ERR_V3(
        cudaMemcpyAsync(pc_dev_, cloud_ptr->points.data(), pc_size * sizeof(pcl::PointXYZI), cudaMemcpyHostToDevice, cpy_stream_));

    const auto& valid_indices = lidar_msg_ptr_->valid_indices;
    std::vector<uint8_t> valid_flag_host(cloud_ptr->size(), 0);

    for (size_t i = 0; i < valid_indices.size(); ++i) {
        valid_flag_host[valid_indices[i]] = 1;
    }
    const auto& valid_size = valid_flag_host.size();
    // copy valid flag to device
    TV_CHECK_CUDA_ERR_V3(
        cudaMemcpyAsync(valid_flag_dev_, valid_flag_host.data(), valid_size * sizeof(uint8_t), cudaMemcpyHostToDevice, cpy_stream_));

    const auto& bev_size = params_ptr_->bev_size;
    const auto& xmin = params_ptr_->detection_range.xmin;
    const auto& xmax = params_ptr_->detection_range.xmax;
    const auto& ymin = params_ptr_->detection_range.ymin;
    const auto& ymax = params_ptr_->detection_range.ymax;
    const auto& zmax = params_ptr_->detection_range.zmax;
    const auto& zmin = params_ptr_->detection_range.zmin;
    const auto& pillar_x_size = params_ptr_->pillar_x_size;
    const auto& pillar_y_size = params_ptr_->pillar_y_size;
    const auto& pillar_z_size = params_ptr_->pillar_z_size;
    //const auto& pillar_z_size = params_ptr_->detection_range.zmax-params_ptr_->detection_range.zmin;
    const auto& grid_x_size = params_ptr_->cols;
    const auto& grid_y_size = params_ptr_->rows;
    const auto& max_num_points_per_pillar = params_ptr_->max_num_points_per_pillar;
    const auto& max_num_pillars = params_ptr_->max_num_pillars;

    // memset device  data
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(point_count_, 0, bev_size * sizeof(int), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(pillar_counter_, 0, sizeof(int), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(x_sum_per_pillar_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(y_sum_per_pillar_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(z_sum_per_pillar_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(x_coors_for_sub_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(y_coors_for_sub_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(z_coors_for_sub_, 0, max_num_pillars * sizeof(float), pre_stream_));

    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(feature_data_ptr_, 0,
                                    max_num_pillars * max_num_points_per_pillar * 10 * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(x_coord_ptr_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(y_coord_ptr_, 0, max_num_pillars * sizeof(float), pre_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(valid_pillar_count_, 0, sizeof(float), pre_stream_));

    float num_pillar = 0;
    int threadsInBlock = 1024;  // 64;
    int num_block = DIVUP(pc_size, threadsInBlock);

    // Synchronize data
    cudaStreamSynchronize(cpy_stream_);
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    make_pillar_histo_kernel<<<num_block, threadsInBlock, 0, pre_stream_>>>(
        pc_dev_,
        pc_size,
        valid_flag_dev_,
        dev_pillar_x_in_coors_,
        dev_pillar_y_in_coors_,
        dev_pillar_z_in_coors_,
        dev_pillar_i_in_coors_,
        point_count_,
        xmin,
        ymin,
        zmin,
        pillar_x_size,
        pillar_y_size,
        pillar_z_size,
        grid_x_size,
        grid_y_size,
        1,
        max_num_points_per_pillar);
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    make_pillar_index_kernel_h<<<grid_x_size, grid_y_size, 0, pre_stream_>>>(
        point_count_,
        pillar_counter_,
        x_coord_ptr_,
        y_coord_ptr_,
        x_coors_for_sub_,
        y_coors_for_sub_,
        num_points_per_pillar_,
        grid_x_size,
        xmin,
        ymin,
        max_num_pillars,
        max_num_points_per_pillar,
        pillar_x_size,
        pillar_y_size,
        pillar_z_size);

    int gridSize = 0;
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(&gridSize, pillar_counter_, sizeof(int), cudaMemcpyDeviceToHost));

    if (gridSize > max_num_pillars) {
        gridSize = max_num_pillars;
    }
    num_pillar = static_cast<float>(gridSize);

    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(valid_pillar_count_, &num_pillar, sizeof(float), cudaMemcpyHostToDevice));

    const int pillar_bev = max_num_points_per_pillar * max_num_pillars;
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    if(TEST_CRASH){
        static int crash_idx=0;
        crash_idx++;
        std::cout<<"crash_idx:"<<crash_idx<<std::endl;
        if(crash_idx%20==0){
        // if(1){
            // std::cout<<"crashKernel :"<<crash_idx<<std::endl;
            crashKernel<<<1, 1, 0, pre_stream_>>>();
            cudaStreamSynchronize(pre_stream_);
            TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
            throw std::runtime_error("---THOROW---");
        }
    }

    // <<< (32,42887/512),512  >>>
    int threads_num = 512;
    dim3 dimBlock(threads_num);
    dim3 dimGrid(max_num_points_per_pillar, DIVUP(gridSize, threads_num));
    make_pillar_feature_kernel_h<<<dimGrid, dimBlock, 0, pre_stream_>>>(
        dev_pillar_x_in_coors_,
        dev_pillar_y_in_coors_,
        dev_pillar_z_in_coors_,
        dev_pillar_i_in_coors_,
        feature_data_ptr_,
        x_sum_per_pillar_,
        y_sum_per_pillar_,
        z_sum_per_pillar_,
        x_coord_ptr_,
        y_coord_ptr_,
        num_points_per_pillar_,
        pillar_bev,
        max_num_points_per_pillar,
        grid_x_size,
        gridSize);
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    make_pillar_mean_feature_kernel<<<gridSize, max_num_points_per_pillar, 0, pre_stream_>>>(
        feature_data_ptr_,
        x_coors_for_sub_,
        y_coors_for_sub_,
        x_sum_per_pillar_,
        y_sum_per_pillar_,
        z_sum_per_pillar_,
        num_points_per_pillar_,
        zmin,
        pillar_z_size,
        pillar_bev,
        max_num_points_per_pillar);
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
}

void PointPillarsCenterHeadCudaPreprocess::preprocess(const LidarFrameMsg::Ptr& lidar_msg_ptr) {
    TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));
    lidar_msg_ptr_ = lidar_msg_ptr;
    calFeatures();
}

}  // namespace robosense
