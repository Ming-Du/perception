#pragma once

#include "common/include/basic_type/range.h"
#include "common/include/rs_define.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/grid_map/grid_map.h"

#include <limits.h>

namespace robosense {

class RsGroundFilter {
public:
    using Ptr = std::shared_ptr<RsGroundFilter>;

    RsGroundFilter();

    void init(const Rs_YAMLReader &configParse) ;

    void perception(const LidarFrameMsg::Ptr& msg_ptr) ;

private:
    void linefit(const LidarFrameMsg::Ptr &lidar_msg_ptr);

    void calAroundGridsHeightDiff(const GridMapPtr &grid_map_ptr, int grid_idx);

    double calcAngleErr2D(const double &a, const double &b);

    void RegionGroundFitting(const std::vector<std::vector<pcl::PointXYZ>> &ring_point, std::vector<GroundCoeff> &semantic_map);
    void  GroundMatch(std::vector<GroundCoeff> falcon_semantic_map, const GroundCoeff semantic_map, std::vector<GroundCoeff>  &matched_semantic_map);
private:
    GroundFilterParam params;
    std::vector<std::vector<pcl::PointXYZ>> ring_point;

};

}   // namespace robosense
