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

#ifndef RS_SDK_POINTPILLARSCENTERHEAD_POSTPROCESS_CUDA_H_
#define RS_SDK_POINTPILLARSCENTERHEAD_POSTPROCESS_CUDA_H_

#include "common/include/basic_type/rotate_box.h"
#include "common/base_cuda_check.h"
#include "pointpillars/params.h"


namespace robosense {

struct PointPillarsCenterHeadCudaPostprocessData {
    float* hm_output_data;
    float* center_output_data;
    float* center_z_output_data;
    float* dim_output_data;
    float* rotres_output_data;
    int* rotcls_output_data;
};

class PointPillarsCenterHeadCudaPostprocess {
public:
    using Ptr = std::shared_ptr<PointPillarsCenterHeadCudaPostprocess>;

    explicit PointPillarsCenterHeadCudaPostprocess(const PointPillarsCenterHeadParams::Ptr& params_ptr) {
        params_ptr_ = params_ptr;
        {  // init
          const auto& feature_bev_size = params_ptr_->feature_x_size * params_ptr_->feature_y_size;
          const auto& centerhead_num_classes = params_ptr_->centerhead_num_classes;
          const auto& num_classes = centerhead_num_classes;

        int range_boundary_size=params_ptr_->range_boundary.size();
        float * range_boundary = (float*)malloc(range_boundary_size*sizeof(float));
        float* confidence_map_socre=(float*)malloc((range_boundary_size-1)*num_classes*sizeof(float));
        
        for(int i=0;i<range_boundary_size;i++)
        {
            range_boundary[i]=params_ptr_->range_boundary[i];

        }

        for(int type_i=0;type_i<num_classes;type_i++)
        {
            for(int range_id=0;range_id<range_boundary_size-1;range_id++)
            {
                confidence_map_socre[type_i*(range_boundary_size-1)+range_id]=params_ptr_->vec_type_to_range[type_i][range_id];

            }
        }
        
        TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&range_boundary_,range_boundary_size*sizeof(float)));
        TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&confidence_map_socre_,(range_boundary_size-1)*num_classes*sizeof(float)));
        TV_CHECK_CUDA_ERR_V3(cudaMemcpy(range_boundary_,range_boundary,
                                    range_boundary_size*sizeof(float),cudaMemcpyHostToDevice));
        TV_CHECK_CUDA_ERR_V3(cudaMemcpy(confidence_map_socre_,confidence_map_socre,
                                    (range_boundary_size-1)*num_classes*sizeof(float),cudaMemcpyHostToDevice));


        free(range_boundary);
        free(confidence_map_socre);

        TV_CHECK_CUDA_ERR_V3(cudaMalloc((void **)&bndbox_centerhead_output_, (feature_bev_size * 9 + 1) * sizeof(float)));
        //   BASE_CUDA_CHECK(cudaMalloc((void **)&hm_sigmoid_input_, (feature_bev_size * 2) * sizeof(float)));
        //   BASE_CUDA_CHECK(cudaMalloc((void **)&dim_exp_input_, (feature_bev_size * 3) * sizeof(float)));
        //   BASE_CUDA_CHECK(cudaMalloc((void **)&center_input_, (feature_bev_size * 2) * sizeof(float)));
        //   BASE_CUDA_CHECK(cudaMalloc((void **)&center_z_input_, (feature_bev_size * 1) * sizeof(float)));
        //   BASE_CUDA_CHECK(cudaMalloc((void **)&rot_input_, (feature_bev_size * 2) * sizeof(float)));
        }
        TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&post_stream_));
    }

    void init(const PointPillarsCenterHeadCudaPostprocessData& network_output) {
        hm_input_ptr_ = network_output.hm_output_data;
        center_input_ptr_ = network_output.center_output_data;
        center_z_input_ptr_ = network_output.center_z_output_data;
        dim_input_ptr_ = network_output.dim_output_data;
        rotres_input_ptr_ = network_output.rotres_output_data;
        rotcls_input_ptr_ = network_output.rotcls_output_data;
    }

    void postprocess(const LidarFrameMsg::Ptr& lidar_msg_ptr);

    void cleanData() {
        const auto& feature_bev_size = params_ptr_->feature_x_size * params_ptr_->feature_y_size;
        TV_CHECK_CUDA_ERR_V3(cudaMemset(bndbox_centerhead_output_, 0,(feature_bev_size * 9 + 1) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(hm_sigmoid_input_, 0,(feature_bev_size * 2) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(dim_exp_input_, 0, (feature_bev_size * 3) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(center_input_, 0,(feature_bev_size * 2) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(center_z_input_, 0, (feature_bev_size * 1) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(rot_input_, 0,(feature_bev_size * 2) * sizeof(float)));
    }

private:
    std::string name() {
        return params_ptr_->frame_id + "/PointPillarsCenterHeadCudaPostprocess";
    }
    void centerHeadPostProcessCuda();
    void generateResults();
private:
    float* hm_input_ptr_;
    float* center_input_ptr_;
    float* center_z_input_ptr_;
    float* dim_input_ptr_;
    float* rotres_input_ptr_;
    int* rotcls_input_ptr_;
    float *range_boundary_;
    float *confidence_map_socre_;

    float* bndbox_centerhead_output_; 
    // float* hm_sigmoid_input_;
    // float* dim_exp_input_;
    // float* center_input_;
    // float* center_z_input_;
    // float* rot_input_;
    const std::map<int, ObjectType> kIdxTypeMap = {{0, ObjectType ::PED},
                                                   {1, ObjectType ::BIC},
                                                   {2, ObjectType ::CAR},
                                                   {3, ObjectType ::TRUCK},
                                                   {4, ObjectType ::BUS}};
    LidarFrameMsg::Ptr lidar_msg_ptr_;
    PointPillarsCenterHeadParams::Ptr params_ptr_;

    //==============================================================
    cudaStream_t post_stream_;
};

}  // namespace robosense

#endif  // RS_SDK_POINTPILLARSCENTERHEAD_POSTPROCESS_CUDA_H_
