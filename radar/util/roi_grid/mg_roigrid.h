#ifndef MG_ROIGRID_H_
#define MG_ROIGRID_H_

#include <geometry_msgs/Point.h>
#include <iomanip>

#include "autopilot_msgs/ros_proto.h"
#include "common/proto/localization.pb.h"
#include "perception/radar/common/include/radar_define.h"

#include "common/proto/convexhull.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

#include <proj_api.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define UINT32_MAX_DEFINE 4294967296L
namespace perception {
namespace radar {

class MgRoigrid {
 public:
  using Ptr = std::shared_ptr<MgRoigrid>;

  MgRoigrid() {}
  bool isInRoadMap(double local_x, double local_y, int& road_type);
  void delOldMap();
  void showPngsUsedThisFrame() {
    ROS_DEBUG_STREAM("rand num mark:" << rand() << "This Frame Used png:");
    for (auto ci : curfrm_used_paths) {
      ROS_DEBUG_STREAM(HDEBUG_B << ci);
    }
  }
  void setShrinkSize(const int shrink_size) { roadmap_shrink_ = shrink_size; }

 public:
  std::string grid_map_data_path;
  std::map<int64_t, cv::Mat> tile_maps;
  std::map<int64_t, int> tile_ids_lostnum;  // id , num of continuous lost frames
  const localization::Localization* local_cur_ptr;
  std::vector<std::string> curfrm_used_paths;

 private:
  void setBinaryInRoad(cv::Mat& imgGray);
  int tileMapFiltering(double distance_x, double distance_y);
  int getRoadElement(double& current_lon,
                     double& current_lat,
                     const int64_t& level,
                     const int64_t tile_id);
  int64_t getTileID(const double& lon, const double& lat, const int64_t& level);
  void convertLocalToGlobal(double utm_x,
                            double utm_y,
                            double utm_z,
                            double heading,
                            double& distance_x,
                            double& distance_y,
                            std::string UTM_ZONE);

 private:
  std::vector<std::string> invalid_paths;
  const int64_t NDS_90_DEGREES = 0x3fffffff;
  const int64_t NDS_180_DEGREES = 0x7fffffff;
  const int64_t NDS_360_DEGREES = UINT32_MAX_DEFINE;
  double RAD_TO_DEG_LOCAL = 180.0 / M_PI;
  double EARTH_RADIUS = 6378137;
  projPJ wgs84pj_target_;
  projPJ utm_source_;
  const int64_t level = 16;
  const double tile_length = 180 * 1.0 / (1 << level);
  int roadmap_shrink_ = 0;
};

struct YamlRoadMap {
  std::string city;
  int utm_zone;
  std::string mappath;
  std::string version;
};
}  // namespace radar
}  // namespace perception
#endif