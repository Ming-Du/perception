#pragma once

#include "common/include/config/rs_yamlReader.h"
#include "common/include/msg/lidar_frame_msg.h"

namespace robosense {

enum NameMapping {
    MergeList = 0,
    NoList,
    RefineListTracker,
    RefineListMeasure
};

class ProposalSelector {
public:
    using Ptr = std::shared_ptr<ProposalSelector>;

    ProposalSelector();

    void init(const RefinerParam &param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr, std::vector<std::pair<int, int>> &box_refine_list,
                    std::vector<std::pair<int, std::vector<int>>> &box_merge_list);

private:
    RefinerParam param_;
};

} // namespace robosense