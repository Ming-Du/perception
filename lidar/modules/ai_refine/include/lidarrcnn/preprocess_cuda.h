#pragma once

#include "common/base_cuda_check.h"
#include "lidarrcnn/params.h"

namespace robosense {

struct LiDARRCNNPreprocessData {
    float *feature_data;
};

class LiDARRCNNPreprocess {
public:
    using Ptr = std::shared_ptr<LiDARRCNNPreprocess>;

    explicit LiDARRCNNPreprocess(const LiDARRCNNParams::Ptr &params_ptr) {
        params_ptr_ = params_ptr;
        TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));
        allocateMemory();
    }

    void init(const LiDARRCNNPreprocessData &network_input) {
        feature_data_ptr_ = network_input.feature_data;
    }

    void preprocess(const LidarFrameMsg::Ptr &lidar_msg_ptr, const int start_idx);

private:
    void allocateMemory();

    float getExpandProposalMeter(const Object::Ptr &proposal);

    void extractAndProcessPoints(const Object::Ptr &proposal,
                                 const float expand_proposal_meter,
                                 const int prop_id_in_batch,
                                 const int pc_size);

    void ProcessPoints(const Object::Ptr &proposal, const int prop_id_in_batch);

private:
    
    int max_pc_nums_ = 400000;
    pcl::PointXYZI *pc_dev_;
    float *feature_data_ptr_;
    int *random_pc_valid_indice_dev_;

    LiDARRCNNParams::Ptr params_ptr_;
    LidarFrameMsg::Ptr lidar_msg_ptr_;

    cudaStream_t cpy_stream_, pre_stream_;
};

} // namespace robosense
