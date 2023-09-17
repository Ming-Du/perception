#include "object.h"

namespace perception {
namespace fusion {

Object::Object() {
  center_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  velocity_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  acceleration_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  sub_type_probs.resize(static_cast<int>(ObjectSubType::MAX_OBJECT_TYPE), 0.0f);
  b_cipv = false;
  //  feature.reset();
}

void Object::Reset() {
  id = -1;
  polygon.clear();
  direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  theta = 0.0f;
  yaw = 0.0f;
  host_yaw = 0.0f;
  theta_variance = 0.0f;
  center = Eigen::Vector3d(0.0, 0.0, 0.0);
  center_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  // Modify @ jiangnan :  reset  position and position_position_uncertainty
  center_ego = Eigen::Vector3d(0.0, 0.0, 0.0);
  position = Eigen::Vector3d(0, 0, 0);
  host_position = Eigen::Vector3d(0, 0, 0);
  position_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

  size = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  size_variance = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  anchor_point = Eigen::Vector3d(0.0, 0.0, 0.0);
  type = ObjectType::UNKNOWN;
  type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0.0f);
  sub_type = ObjectSubType::UNKNOWN;
  sub_type_probs.assign(static_cast<int>(ObjectSubType::MAX_OBJECT_TYPE), 0.0f);

  group_type = 0;
  is_lidar_rb = false;
  is_static = false;
  is_predicted = false;
  confidence = 1.0f;
  status = 0;  

  track_id = -1;
  velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  velocity_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  velocity_converged = true;
  velocity_confidence = 1.0;
  acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  acceleration_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  acceleration_ego = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

  tracking_time = 0.0;
  latest_tracked_time = 0.0;

  car_light.Reset();
  motion_state = MotionState::UNKNOWN;

  lidar_supplement.Reset();
  radar_supplement.Reset();
  camera_supplement.Reset();
  fusion_supplement.Reset();
  // Nodify @jiangnan add falcon_lidar;
  falcon_lidar_supplement.Reset();
  vidar_supplement.Reset();
  // Modify(@liuxinyu): obu_test
  obu_supplement.Reset();
  //  feature.reset();
}

std::string Object::ToString() const {
  std::ostringstream oss;
  oss << "Object [id: " << id << ", track_id: " << track_id << ", direction: (" << direction[0]
      << "," << direction[1] << "," << direction[2] << "), theta: " << theta
      << ", theta_variance: " << theta_variance << ", center: (" << center[0] << "," << center[1]
      << "," << center[2] << ")"
      << ", center_uncertainty: (" << center_uncertainty(0, 0) << "," << center_uncertainty(0, 1)
      << "," << center_uncertainty(0, 2) << "," << center_uncertainty(1, 0) << ","
      << center_uncertainty(1, 1) << "," << center_uncertainty(1, 2) << ","
      << center_uncertainty(2, 0) << "," << center_uncertainty(2, 1) << ","
      << center_uncertainty(2, 2) << "), size: (" << size[0] << "," << size[1] << "," << size[2]
      << "), size_variance: (" << size_variance[0] << "," << size_variance[1] << ","
      << size_variance[2] << "), anchor_point: (" << anchor_point[0] << "," << anchor_point[1]
      << "," << anchor_point[2] << "), type: " << static_cast<int>(type)
      << ", confidence: " << confidence << ", track_id: " << track_id << ", velocity: ("
      << velocity[0] << "," << velocity[1] << "," << velocity[2]
      << "), velocity_confidence: " << velocity_confidence << ", acceleration: (" << acceleration[0]
      << "," << acceleration[1] << "," << acceleration[2] << "), tracking_time: " << tracking_time
      << ", latest_tracked_time: " << latest_tracked_time;
  return oss.str();
}

}  // namespace fusion
}  // namespace perception
