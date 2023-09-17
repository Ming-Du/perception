#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "common/include/basic_type/range.h"
#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/rs_define.h"

#include "common/proto/localization.pb.h"
#include "common/include/rs_util.h"
#include "common/include/tic_toc.h"
#include "frame_transform.h"

#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <proj_api.h>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace robosense {
class RoiFilter {
public:
    using Ptr = std::shared_ptr<RoiFilter>;

    RoiFilter();
    ~RoiFilter();

    void init(const Rs_YAMLReader &configParse);
    void updataLocalization(const localization::Localization &Localization, bool is_parser_map_info);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void pngLoadThread();

    void imreadPng(std::string png_path, cv::Mat &img);
    void imreadOnes(cv::Mat &img, unsigned char value);
    bool checkAndLoadImgs(const std::vector<std::pair<int, int>> &idx_idy_vec);
    void idxyToUtm(const int x_id, const int y_id, double &center_utm_x, double &center_utm_y);
    void getIdxIdyList(double utm_x, double utm_y, double search_range,
                       std::vector<std::pair<int, int>> &idx_idy_vec);

private:
    RoiFilterParam params_;
    Rs_YAMLReader configParse_;

    mapDefine map_info_;
    std::map<int, YamlRoadMap> map_list_;

    bool exit_flag_;
    std::mutex mutex_;
    std::condition_variable img_load_cv_;
    std::shared_ptr<std::thread> t_img_load_;

    std::string img_path_;

    int idx_min_;
    int idy_min_;
    cv::Mat road_map_mat;
    bool valid_img_loaded_flag_;
    std::map<std::pair<int, int>, cv::Mat> valid_img_map_; 
    
    localization::Localization local_current_;

};

} // namespace robosense

