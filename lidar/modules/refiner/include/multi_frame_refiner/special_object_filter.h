#pragma once

#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"
namespace robosense {

class SpecialObjectFilter {
public:
    using Ptr = std::shared_ptr<SpecialObjectFilter>;

    SpecialObjectFilter();

    void init(const RefinerParam& param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    RefinerParam param_;

};

} // namespace robosense