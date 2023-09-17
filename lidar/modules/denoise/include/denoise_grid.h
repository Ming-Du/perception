#ifndef DENOISE_GRID_H_
#define DENOISE_GRID_H_

#include <geometry_msgs/Point.h>
#include <iomanip>

#include "common/include/basic_type/range.h"
#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/rs_define.h"
#include "common/include/tic_toc.h"

#include "common/proto/convexhull.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

#include <iostream>
#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "xgb.h"
#include "denoise_feature_extractor.h"
namespace robosense {

class DenoiseGrid {
public:
    using Ptr = std::shared_ptr<DenoiseGrid>;
    DenoiseGrid() {

    }
    void try_grid_data(LidarFrameMsg::Ptr &msg_ptr);
    // void try_grid_data_obj(LidarFrameMsg::Ptr &msg_ptr);
    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);
    // double calcAreaFromConvex(const std::vector<Eigen::Vector3d> &polygon2D);
    // void perception_object(const LidarFrameMsg::Ptr &msg_ptr);
    // double calcWeightObject(double conf_predict,double conf_model);
    // void showObjectPoints(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &obj_ptidx);
    void createGridPointsMap(const LidarFrameMsg::Ptr &msg_ptr);
    // void createObjPointsMap(const LidarFrameMsg::Ptr &msg_ptr,std::vector<int> &cloud_indices, std::vector<int> &obj_ptidx);
private:
    bool isInRange(double x,double y,double z);
public:
    Xgb::Ptr xgb_ptr_;
    // Xgb::Ptr xgb_obj_ptr_;
    DenoiseFeatureExtractor::Ptr feature_extractor_ptr_;
    std::map<int, std::vector<int>> rowcol_ptidxs_map_;
    std::vector<int> rowcol_vec_;
    DenoiseParam params_;
    bool denoise_enable_;
};


} // namespace robosense
#endif