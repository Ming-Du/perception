#pragma once

#include "perception/base/object.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "perception/base/timer.h"
#include "proto/radar_fusion_component.pb.h"
#include "common/include/pb_utils.h"
#include "common/proto/sensor_meta.pb.h"
#include "common/proto/geometry.pb.h"

#include <common/proto/localization.pb.h>
#include "autopilot_msgs/BinaryData.h"
#include "autopilot_msgs/ros_proto.h"
#include "perception/radar/common/include/yaml.h"
#include "perception/radar/rviz_display/rviz_display.h"
#include "perception/radar/util/roi_grid/mg_roigrid.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

namespace perception {
namespace radar {

const double RADAR_OBJS_MATCH_MIN_DIS_THRESH = 0.3;  //

class alignas(16) Range3D {
 public:
  Range3D() = default;

  Range3D(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) {
    this->xmin = x_min;
    this->xmax = x_max;
    this->ymin = y_min;
    this->ymax = y_max;
    this->zmin = z_min;
    this->zmax = z_max;
  }

  double xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
};

class RadarFusion {
 public:
  RadarFusion(const std::shared_ptr<ros::NodeHandle>& node);
  ~RadarFusion() = default;

  bool Init();
  std::string Name() const { return "RadarFusion"; }

 private:
  void RadarObstacleCallback(const std_msgs::StringConstPtr& msg);
  double GetRadarObjDis2D(const perception::RadarObject* radar_object1,
                          const perception::RadarObject* radar_object2);
  double GetRadarObjSpeed2D(const perception::RadarObject* radar_object1,
                          const perception::RadarObject* radar_object2);
  bool filter_front_308(const perception::RadarObject& radar_object);
  void LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  bool UpdateRadarToImu(const double& timestamp, localization::Localization& localization);
  void RadarFusionShowRoadMap(ros::Publisher& publisher);
  bool AttributeFilter(const perception::RadarObject* radar_object);
  bool FovFilter(const perception::RadarObject* radar_object);
  void ChangeObjectId(const size_t maxId);
  void radarFilter(const std::map<std::string, perception::RadarObjects>::iterator frame, 
                   perception::RadarObjects* radar_objects_fusion, bool is_main_sensor, 
                   localization::Localization local_current);
  void ModifyShape(perception::RadarObjects& radar_objects_fusion,
                   localization::Localization local_current);
  void ModifyPolygon(perception::Object* object, geometry::Point& contour_point, 
                     double sin_angle, double cos_angle, 
                     double corner_x, double corner_y);


 private:
  perception::base::SensorInfo sensor_info_;
  std::shared_ptr<ros::NodeHandle> node_ = nullptr;
  ros::Publisher radar_objects_pub_;
  ros::Publisher radar_objects_viz_pub, pub_roadmap_;
  std::map<std::string, perception::RadarObjects> sensor_frames_;
  std::vector<ros::Subscriber> sensor_subscribers_;
  double min_distance_;
  double delta_distance_;
  double delta_speed_;
  double abs_speed_;
  double match_min_distance_thresh_;
  double x_offset_;
  std::string fusion_main_sensor_;
  bool is_main_sensor_;

  ros::Subscriber localization_subscriber_;
  std::list<localization::Localization> global_localizations_;
  // grid map data path
  std::string perception_grid_map_path_;
  std::string grid_map_data_path_;
  std::string radar_config_path_;
  bool utm_zone_flag_ = false;
  std::string UTM_ZONE_;
  bool lock_localization_ = false;
  const std::string map_folder_hengyang_ = "hengyang";
  const std::string map_folder_beijing_ = "beijing";
  MgRoigrid::Ptr roigrid_ptr_ = nullptr;
  bool is_use_roadmap_ = true;
  bool is_show_roadmap_ = false;
  int roadmap_shrink_ = 0;
  std::map<int, YamlRoadMap> yaml_roadmap;
  // rviz display roadmap
  float road_map_size_;
  std::vector<Eigen::Vector2d> roadmap_;
  Range3D range = Range3D(-70., 70., -50., 50., 0.5, 2.5);
  size_t single_radar_max_id_;
  double lowspeed_threshold_;
  double lowspeed_distance_threshold_;
  double highspeed_distance_threshold_;
  double lowspeed_speed_threshold_;
  double highspeed_speed_threshold_;
  double size_lowspeed_threshold_;
  double lowspeed_size_length_threshold_;
  double lowspeed_size_wideth_threshold_;
  double lowspeed_size_height_threshold_;
  double highspeed_size_length_threshold_;
  double highspeed_size_wideth_threshold_;
  double highspeed_size_height_threshold_;
  Params param_;
};

}  // namespace radar
}  // namespace perception
