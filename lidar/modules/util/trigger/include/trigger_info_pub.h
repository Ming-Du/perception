#pragma once

#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/tic_toc.h"
namespace robosense {

class TriggerManager {

#define BUFFER_SIZE 5
#define M_PI 3.14159265358979323
const float distanceThreshold = 10;
const int oversize_thre = 4;
const int traffic_thre = 7;
public:
    using Ptr = std::shared_ptr<TriggerManager>;
    TriggerManager();

    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    TriggerType check_noise(const int tmp_del_noise);
    TriggerType check_yaw(const LidarFrameMsg::Ptr &msg_ptr);
    TriggerType check_traffic(const LidarFrameMsg::Ptr &msg_ptr);
    TriggerType check_oversize_vehicle(const LidarFrameMsg::Ptr &msg_ptr);
    double diff_yaw(const double yaw_a, const double yaw_b);

};



}