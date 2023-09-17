#pragma once

#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "common/include/grid_map/grid_map.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"

namespace robosense {

class BevBuilder {
public:
    using Ptr = std::shared_ptr<BevBuilder>;

    BevBuilder();
    ~BevBuilder();

    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    GridMapPtr grid_map_ptr_;

};

} // namespace robosense
