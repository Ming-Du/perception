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
#include "pointpillars/postprocess_cuda.h"
#include "common/nms.h"
#include <fstream>

namespace robosense {
__device__ void get_confidence_kernal (const float distance, const int cls_id,float *confidence_thres,
                                                                                    const int range_boundary_size,
                                                                                    const float *range_boundary_,
                                                                                    const float *confidence_map_socre_,
                                                                                    const int default_thresh)
{
            int range_idx=-1;
            for(int  idx=0;idx<range_boundary_size-1;idx++){
                if((distance>=range_boundary_[idx])&&(distance<range_boundary_[idx+1]))
                {
                    range_idx=idx;
                    break;
                }
            }
            if (range_idx!=-1)
            {
                    *confidence_thres = confidence_map_socre_[cls_id*(range_boundary_size-1)+range_idx];
            }
            
}

__global__ void make_centerhead_postprocess_kernal(const float *hm_sigmoid_input, 
                                                   const float *dim_exp_input,
                                                   const float *center_input,
                                                   const float *center_z_input,
                                                   const float *rotres_input,
                                                   const int *rotcls_input,
                                                   int *object_counter,
                                                   float *bndbox_centerhead_output_,
                                                   const int feature_x_size,
                                                   const int feature_y_size,
                                                   const int downsample_stride,
                                                   const float pillar_x_size,
                                                   const float pillar_y_size,
                                                   const float xmin,
                                                   const float ymin,
                                                   const float score_thresh,
                                                   const int range_boundary_size,
                                                   const float *range_boundary_,
                                                   const float *confidence_map_socre_,
                                                   const bool use_confidence_map,
                                                   const float angle_per_class,
                                                   const float pi){

    int i = blockIdx.x;
    if (hm_sigmoid_input[i] > score_thresh){
        int ind = i % (feature_x_size * feature_y_size);
        
        float center_x = center_input[ind];
        float center_y = center_input[feature_x_size * feature_y_size + ind];
        float xs = (ind % feature_x_size + center_x) * downsample_stride * pillar_x_size + xmin;
        float ys = (ind / feature_x_size + center_y) * downsample_stride * pillar_y_size + ymin;
        float zs = center_z_input[ind];

        float w = dim_exp_input[ind];
        float l = dim_exp_input[feature_x_size * feature_y_size + ind];
        float h = dim_exp_input[2 * feature_x_size * feature_y_size + ind];

        float rotres = rotres_input[ind];
        int rotcls = rotcls_input[ind];
        // printf("ind rotcls: %d, %d\n", ind, rotcls);
        float shift_angle = rotres * (angle_per_class / 2) + rotcls * angle_per_class;
        float heading_angle = shift_angle < 0 ? 2 * pi + shift_angle : shift_angle;
        float angle = heading_angle - pi;


        int class_id = i / (feature_x_size * feature_y_size);
        // printf("cls, score, x, y, z: %d, %f, %f, %f, %f\n", class_id, hm_sigmoid_input[i], xs, ys, zs);

        if(use_confidence_map)
        {
            float confidence_thres = score_thresh; 
            get_confidence_kernal(xs, class_id, &confidence_thres,
                                                range_boundary_size,
                                                range_boundary_,
                                                confidence_map_socre_,
                                                score_thresh);

            if(hm_sigmoid_input[i] < confidence_thres)
            {
                return;
            }
        }

        int resCount = (int)atomicAdd(object_counter, 1);
        bndbox_centerhead_output_[0] = (float)resCount;
        float *data = bndbox_centerhead_output_ + 1 + resCount * 9;
        data[0] = xs;
        data[1] = ys;
        data[2] = zs;
        data[3] = w;
        data[4] = l;
        data[5] = h;
        data[6] = angle;
        data[7] = class_id;
        data[8] = hm_sigmoid_input[i];
    }


}

void PointPillarsCenterHeadCudaPostprocess::centerHeadPostProcessCuda() {
    const int feature_x_size = params_ptr_->feature_x_size;
    const int feature_y_size = params_ptr_->feature_y_size;
    const int feature_bev_size = feature_x_size * feature_y_size;
    const auto& downsample_stride = params_ptr_->downsample_stride;
    const auto& pillar_x_size = params_ptr_->pillar_x_size;
    const auto& pillar_y_size = params_ptr_->pillar_y_size;
    const auto &xmin = params_ptr_->detection_range.xmin;
    const auto &ymin = params_ptr_->detection_range.ymin;
    const auto &score_thresh = params_ptr_->centerhead_regression_box_confidence_thre;
    const auto &use_confidence_map=params_ptr_->use_confidence_map;
    const auto &range_boundary_size=params_ptr_->range_boundary.size();
    const int num_rotcls = params_ptr_->num_rotcls;
    const float pi = std::acos(-1);
    const float angle_per_class = 2 * pi / num_rotcls;


    int *obj_count;
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void **)&obj_count, sizeof(int)));
    TV_CHECK_CUDA_ERR_V3(cudaMemset(obj_count, 0, sizeof(int)));
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    make_centerhead_postprocess_kernal<<<feature_bev_size*5, 1, 0, post_stream_>>>(hm_input_ptr_, 
                                                                                   dim_input_ptr_,
                                                                                   center_input_ptr_,
                                                                                   center_z_input_ptr_,
                                                                                   rotres_input_ptr_,
                                                                                   rotcls_input_ptr_,
                                                                                   obj_count,
                                                                                   bndbox_centerhead_output_,
                                                                                   feature_x_size,
                                                                                   feature_y_size,
                                                                                   downsample_stride,
                                                                                   pillar_x_size,
                                                                                   pillar_y_size,
                                                                                   xmin,
                                                                                   ymin,
                                                                                   score_thresh,
                                                                                   range_boundary_size,
                                                                                   range_boundary_,
                                                                                   confidence_map_socre_,
                                                                                   use_confidence_map,
                                                                                   angle_per_class,
                                                                                   pi);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
}


void PointPillarsCenterHeadCudaPostprocess::generateResults() {
    const auto &nms_thresh = params_ptr_->nms_thres;
    const auto &ior_thresh = params_ptr_->ior_thres;

    // center head res
    float obj_count2 = 0;    
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(&obj_count2, bndbox_centerhead_output_, sizeof(float),
                    cudaMemcpyDeviceToHost));
    int num_obj2 = static_cast<int>(obj_count2);
    auto output2 = std::shared_ptr<float>(new float[num_obj2 * 9]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output2.get(), bndbox_centerhead_output_ + 1, num_obj2 * 9 * sizeof(float),
                    cudaMemcpyDeviceToHost));


    std::vector<Bndbox> res;
    res.reserve(num_obj2);

    for (int i = 0; i < num_obj2; i++) {
        res.emplace_back(Bndbox(output2.get()[i * 9],     output2.get()[i * 9 + 1], output2.get()[i * 9 + 2], 
                                output2.get()[i * 9 + 3], output2.get()[i * 9 + 4], output2.get()[i * 9 + 5], 
                                output2.get()[i * 9 + 6], static_cast<int>(output2.get()[i * 9 + 7]), 
                                output2.get()[i * 9 + 8]));
    }

    // nms
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    auto nms_pred = nms_cpu(res, nms_thresh, ior_thresh);
    std::vector<Object::Ptr> objects;
    objects.resize(nms_pred.size());
    auto& object_vec = lidar_msg_ptr_->objects_ai;
    object_vec.reserve(nms_pred.size());
    for (size_t i = 0; i < nms_pred.size(); ++i) {
        auto& obj = objects[i];
        obj.reset(new Object);
        obj->type_confidence = nms_pred[i].score;
        obj->exist_confidence = nms_pred[i].score;
        obj->type = kIdxTypeMap.at(nms_pred[i].id);
        // if (nms_pred[i].x > 14 && nms_pred[i].x < 15)
       
        RotateBox box;
        Eigen::Vector3d center, size, angle;
        size.x() = nms_pred[i].w;
        size.y() = nms_pred[i].l;
        size.z() = nms_pred[i].h;
        center.x() = nms_pred[i].x;
        center.y() = nms_pred[i].y;
        center.z() = nms_pred[i].z;

        box = RotateBox(center, size, nms_pred[i].rt);
        obj->center = center;
        obj->size = size;
        obj->direction = box.heading;
        object_vec.emplace_back(obj);
    }
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
}

void PointPillarsCenterHeadCudaPostprocess::postprocess(const LidarFrameMsg::Ptr& lidar_msg_ptr) {
    lidar_msg_ptr_ = lidar_msg_ptr;
    cleanData();
    centerHeadPostProcessCuda();
    generateResults();

}

}  // namespace robosense
