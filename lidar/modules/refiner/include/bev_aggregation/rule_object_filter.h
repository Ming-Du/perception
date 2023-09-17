#pragma once

#include "common/include/config/rs_yamlReader.h"
#include "common/include/msg/lidar_frame_msg.h"

namespace robosense {

class RuleObjectFilter {
public:
    using Ptr = std::shared_ptr<RuleObjectFilter>;

    RuleObjectFilter();

    void init(const RefinerParam &param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void ghostObjFilter(const LidarFrameMsg::Ptr &msg_ptr);
    void floatingObjFilter(const LidarFrameMsg::Ptr &msg_ptr);
    void flowerbedObjFilter(const LidarFrameMsg::Ptr &msg_ptr);

private:
    RefinerParam param_;
};

} // namespace robosense
