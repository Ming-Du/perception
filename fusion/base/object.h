#pragma once
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "Eigen/Core"
#include "object_supplement.h"
#include "object_types.h"
#include "point_cloud.h"
#include "vehicle_struct.h"
// #include "prediction/proto/feature.pb.h"

namespace perception {
namespace fusion {

struct alignas(16) Object {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Object();
  std::string ToString() const;
  void Reset();

  // @brief object id per frame, required
  int id = -1;

  // @brief convex hull of the object, required
  PointCloud<PointD> polygon;
  //@lijian,添加一个utm坐标系下的polygon
  PointCloud<PointD> polygon_utm;
  //@lijian,添加一个ego坐标系下的polygon（不旋转）
  PointCloud<PointD> polygon_ego;
  //@lijian,添加存储lidar的像素坐标下的中心点或者轮廓点。
  std::vector<Eigen::Vector2f> centers_image;
  //@lijian,添加记录融合匹配到的camera目标的次数
  int hit_camera_times = 0;
  int hit_lslidar_times = 0;  //雷神的匹配次数
  int hit_flidar_times = 0;   //图达通匹配次数
  // oriented boundingbox information
  // @brief main direction of the object, required
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
  /*@brief the yaw angle, theta = 0.0 <=> direction(1, 0, 0),
    currently roll and pitch are not considered,
    make sure direction and theta are consistent, required
  */
  // Modify @jiangnan:  add ego host
  float host_yaw = 0.0f;
  float theta = 0.0f;
  // Modify @ jiangnan : yaw (global state)
  float yaw = 0.0f;
  // @brief theta variance, required
  float theta_variance = 0.0f;
  // Modify @ jiangnan : is ai  or  rulebase
  bool is_lidar_rb = false;
  bool is_falcon = false;  // modify by duming
  // Modify @jiangnan : is  motion or static;
  bool is_static = false;
  // Modify @jiangnan : is  predict or detect;
  bool is_predicted = false;
  // Modify @jiangnan : transform status from lidar
  int status = 0;

  // Modify @jiangnan : lidar grouo type : 1: group  0 :single
  int group_type = 0;
  int filtered_by_map = 0;

  // Modify @jiangnan ::use   global position  and relative position (center)
  // ,they are all in UTM ;
  Eigen::Vector3d host_position = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3f position_uncertainty;
  Eigen::Vector3d center_ego = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the center uncertainty, required
  Eigen::Matrix3f center_uncertainty;
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief size variance, required
  Eigen::Vector3f size_variance = Eigen::Vector3f(0, 0, 0);
  // @brief anchor point, required
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);

  // @brief object type, required
  ObjectType type = ObjectType::UNKNOWN;
  // @brief probability for each type, required
  std::vector<float> type_probs;

  // @brief object sub-type, optional
  ObjectSubType sub_type = ObjectSubType::UNKNOWN;
  // @brief probability for each sub-type, optional
  std::vector<float> sub_type_probs;

  // @brief existence confidence, required
  float confidence = 1.0f;

  // tracking information
  // @brief track id, required
  int track_id = -1;
  int match_id_radar = -1;
  int match_id_lidar = -1;
  int match_id_camera_60f = -1;
  int match_id_camera_30f = -1;
  int match_id_camera_120r = -1;
  int match_id_camera_120f = -1;
  int match_id_falcon = -1;
  int match_id_vidar = -1;
  int match_id_obu = -1;
  // @brief velocity of the object, required
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f velocity_uncertainty;
  // @brief if the velocity estimation is converged, true by default
  bool velocity_converged = true;
  // @brief velocity confidence, required
  float velocity_confidence = 1.0f;
  // @brief acceleration of the object, required
  Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the acceleration uncertainty, required
  Eigen::Matrix3f acceleration_uncertainty;
  // acceleration in ego
  Eigen::Vector3f acceleration_ego = Eigen::Vector3f(0, 0, 0);

  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;

  // @brief motion state of the tracked object, required
  MotionState motion_state = MotionState::UNKNOWN;
  int noise_state = 0;
  // // Tailgating (trajectory of objects)
  std::deque<Eigen::Vector3d> trajectory;
  // // CIPV
  bool b_cipv = false;
  // @brief brake light, left-turn light and right-turn light score, optional
  CarLight car_light;
  // @brief sensor-specific object supplements, optional
  LidarObjectSupplement lidar_supplement;
  RadarObjectSupplement radar_supplement;
  CameraObjectSupplement camera_supplement;
  FusionObjectSupplement fusion_supplement;
  // Modify(@liuxinyu): obu_test
  ObuObjectSupplement obu_supplement;

  // add by ming, vidar has same struct as camera
  CameraObjectSupplement vidar_supplement;
  ObjectSource source;

  // add falcon lidar   by jiangnan
  LidarObjectSupplement falcon_lidar_supplement;

  // @debug feature to be used for semantic mapping
  //  std::shared_ptr<apollo::prediction::Feature> feature;
};

using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

}  // namespace fusion
}  // namespace perception
