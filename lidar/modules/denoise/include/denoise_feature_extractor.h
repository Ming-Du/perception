#ifndef DENOISE_FEATURE_EXTRACTOR_H_
#define DENOISE_FEATURE_EXTRACTOR_H_

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

// #include <xgboost/c_api.h>
#include <assert.h>
#include <iostream>
#include <memory>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <sensor_msgs/PointCloud.h>

namespace robosense {
class DenoiseFeatureExtractor {
public:
    using Ptr = std::shared_ptr<DenoiseFeatureExtractor>;
    DenoiseFeatureExtractor() {
    }
    void init();
    bool getFeatures_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &points_idx, std::vector<double> &feature,double xy_size);
    bool getFeatures_grid(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &points_idx, std::vector<double> &feature,double xy_size);

private:
    void getZRangeIdx( std::vector<int> &points_idx, std::vector<int> &points_zrange_idx);
    int prepare_xybin_grid(std::vector<int> &point_valid_num_inxybin,int &point_num,std::vector<int> &points_idx,double xy_size);
    int prepare_xybin_obj(std::vector<int> &point_valid_num_inxybin,int &point_num,std::vector<int> &points_idx,double xy_size);

    void points_num( std::vector<int> &points_idx, double area);
    void z_range( std::vector<int> &points_idx);

    void z_bin_ratio_std( std::vector<int> &points_idx);
    void intensity_std(std::vector<int> &points_idx );
    void intensity_mean(std::vector<int> &points_idx );
    void feature_height_spin_h( std::vector<int> &points_idx);
    void feature_height_spin( std::vector<int> &points_idx);
    void xy_bin_histogram_pareto( std::vector<int> &point_valid_num_inxybin,int point_num, std::vector<int> &points_idx);
    void xy_bin_valid_ratio(std::vector<int> &point_valid_num_inxybin,double xy_size, int valid_num, std::vector<int> &points_idx);
    std::vector<double> calcPareto( std::vector<int> &data,int totalnum);
    double stdev( std::vector<double> &data);
    double mean( std::vector<double> &data);
public:
    LidarFrameMsg::Ptr msg_ptr_;
    std::vector<double> *feature_;
    // std::vector<int> *points_idx_;
private:
};
} // namespace robosense
#endif