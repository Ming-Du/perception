/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
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


#ifndef RS_SDK_POINTPILLARSCENTERHEAD_PREPROCESS_CUDA_H_
#define RS_SDK_POINTPILLARSCENTERHEAD_PREPROCESS_CUDA_H_

#include "common/base_cuda_check.h"
#include "pointpillars/params.h"

namespace robosense {

struct PointPillarsCenterHeadCudaPreprocessData {
    float* feature_data;
    float* xcoords_data;
    float* ycoords_data;
    float* valid_pillar_count_data;
};

class PointPillarsCenterHeadCudaPreprocess {
public:
    using Ptr = std::shared_ptr<PointPillarsCenterHeadCudaPreprocess>;

    explicit PointPillarsCenterHeadCudaPreprocess(const PointPillarsCenterHeadParams::Ptr &params_ptr) {
        params_ptr_ = params_ptr;
        TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));
        allocateMemory();
    }
    
    void init(const PointPillarsCenterHeadCudaPreprocessData& network_input) {
        feature_data_ptr_ = network_input.feature_data;
        x_coord_ptr_ = network_input.xcoords_data;
        y_coord_ptr_ = network_input.ycoords_data;
        valid_pillar_count_ = network_input.valid_pillar_count_data;
    }

    void preprocess(const LidarFrameMsg::Ptr &lidar_msg_ptr);

    void cleanData();

private:
    std::string name() {
        return params_ptr_->frame_id + "/PointPillarsCenterHeadCudaPreprocess";
    }

    void allocateMemory();

    void calFeatures();

    // Variables for internal usage
    int max_pc_nums_ = 400000;//H: 40000;
    pcl::PointXYZI *pc_dev_;
    uint8_t *valid_flag_dev_;

    float *dev_pillar_x_in_coors_;
    float *dev_pillar_y_in_coors_;
    float *dev_pillar_z_in_coors_;
    float *dev_pillar_i_in_coors_;
    int *point_count_;
    int *pillar_counter_;
    float *x_sum_per_pillar_;
    float *y_sum_per_pillar_;
    float *z_sum_per_pillar_;
    float *x_coors_for_sub_;
    float *y_coors_for_sub_;
    float *z_coors_for_sub_;
    float *num_points_per_pillar_;

    // Variables for external usage
    float* feature_data_ptr_;
    float* x_coord_ptr_;
    float* y_coord_ptr_;
    float* valid_pillar_count_;

    LidarFrameMsg::Ptr lidar_msg_ptr_;
    PointPillarsCenterHeadParams::Ptr params_ptr_;

    cudaStream_t cpy_stream_, pre_stream_;
};

}  // namespace robosense

#endif  // RS_SDK_POINTPILLARSCENTERHEAD_PREPROCESS_CUDA_H_
