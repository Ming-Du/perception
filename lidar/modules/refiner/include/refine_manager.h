#pragma once

#include <mutex>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

#include "common/include/config/rs_yamlReader.h"
#include "common/include/grid_map/grid_map.h"
#include "common/include/msg/lidar_frame_msg.h"

#include "bev_aggregation/bev_aggregation.h"
#include "modules/ai_refine/include/lidarrcnn/lidarrcnn_ai_refine.h"
#include "modules/denoise/include/denoise_frame.h"
#include "modules/denoise/include/denoise_obj.h"
#include "multi_frame_refiner/multi_frame_refiner.h"
#include "single_frame_refiner/single_frame_refiner.h"

namespace robosense {

class RefineManager {
public:
    using Ptr = std::shared_ptr<RefineManager>;

    RefineManager();
    ~RefineManager();

    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void update_refiner_obj(const LidarFrameMsg::Ptr &msg_ptr);

    void denoise_obj_process();

    void ai_refiner_process();

private:
    RefinerParam params_;
    LidarFrameMsg::Ptr msg_ptr_;

    BevAggregation::Ptr bev_aggregation_ptr;
    SingleFrameRefiner::Ptr single_frame_refiner_ptr;
    MultiFrameRefiner::Ptr multi_frame_refiner_ptr;
    DenoiseObj::Ptr denoise_obj_ptr;
    DenoiseFrame::Ptr denoise_frame_ptr;
    LiDARRCNNAIRefine::Ptr ai_refine_ptr;

    bool exit_flag_;
    std::mutex mutex_;

    bool denoise_obj_processed;
    std::condition_variable denoise_obj_cv_;
    std::shared_ptr<std::thread> denoise_obj_th_;

    bool ai_refiner_enable_;
    bool ai_refiner_processed;
    std::condition_variable ai_refiner_cv_;
    std::shared_ptr<std::thread> ai_refiner_th_;
};

} // namespace robosense
