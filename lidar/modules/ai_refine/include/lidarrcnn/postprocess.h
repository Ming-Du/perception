#pragma once

#include "common/base_cuda_check.h"
#include "common/include/basic_type/rotate_box.h"
#include "common/nms.h"
#include "lidarrcnn/params.h"

namespace robosense {

struct LiDARRCNNPostprocessData {
    float *centers;
    float *sizes;
    float *scores;
    int *class_ids;
    float *headings;
};

class LiDARRCNNPostprocess {
public:
    using Ptr = std::shared_ptr<LiDARRCNNPostprocess>;

    explicit LiDARRCNNPostprocess(const LiDARRCNNParams::Ptr &params_ptr) {
        params_ptr_ = params_ptr;
        { // init
            const auto &num_classes = params_ptr_->num_classes;
            const auto &batch_size = params_ptr_->batch_size;
        }
    }

    void init(const LiDARRCNNPostprocessData &network_output) {
        centers_cuda_ptr_ = network_output.centers;
        sizes_cuda_ptr_ = network_output.sizes;
        scores_cuda_ptr_ = network_output.scores;
        class_ids_cuda_ptr_ = network_output.class_ids;
        headings_cuda_ptr_ = network_output.headings;
    }

    void postprocess(const LidarFrameMsg::Ptr &lidar_msg_ptr, const int start_idx);

    void cleanData() {
        // const auto& batch_size = params_ptr_->batch_size;
        // TV_CHECK_CUDA_ERR_V3(cudaMemset(bndbox_lidarrcnn_output_, 0,(batch_size * 9 + 1) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(hm_sigmoid_input_, 0,(feature_bev_size * 2) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(dim_exp_input_, 0, (feature_bev_size * 3) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(center_input_, 0,(feature_bev_size * 2) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(center_z_input_, 0, (feature_bev_size * 1) * sizeof(float)));
        // BASE_CUDA_CHECK(cudaMemset(rot_input_, 0,(feature_bev_size * 2) * sizeof(float)));
    }

private:
    Bndbox decodeLidarrcnn(std::vector<float> &centers, std::vector<float> &sizes, float headings, const Object::Ptr &proposal, int class_id, float score);

private:
    float *centers_cuda_ptr_;
    float *sizes_cuda_ptr_;
    float *scores_cuda_ptr_;
    int *class_ids_cuda_ptr_;
    float *headings_cuda_ptr_;

    std::mutex mutex_;

    const std::map<int, ObjectType> kIdxTypeMap = {{0, ObjectType ::PED},
                                                   {1, ObjectType ::BIC},
                                                   {2, ObjectType ::CAR},
                                                   {3, ObjectType ::TRUCK},
                                                   {4, ObjectType ::BUS}};
    LiDARRCNNParams::Ptr params_ptr_;

    //==============================================================
};

} // namespace robosense
