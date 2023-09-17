#include "object.h"

namespace perception {
namespace mid_fusion {
Object::Object() {
  position_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  velocity_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  center_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
}

void Object::Reset() {
  meas_id = -1;
  track_id = -1;
  timestamp = 0.0;
  tracking_time = 0.0;
  latest_tracked_time = 0.0;
  frame_id = "";
  position = Eigen::Vector3d(0, 0, 0);
  position_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  center = Eigen::Vector3d(0, 0, 0);
  center_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  polygon.clear();
  velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  velocity_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  velocity_converged = true;
  velocity_confidence = 1.0f;
  theta = 0.0f;
  size = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  size_variance = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  type = ObjectType::UNKNOWN;
  type_variance = 1.0;
  type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0.0f);
  confidence = 1.0f;
  direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
}

std::string Object::ToString() const {
  std::ostringstream oss;
  oss << "Object [id: " << meas_id << ", track_id: " << track_id << ", theta: " << theta
      << ", center: (" << center[0] << "," << center[1] << "," << center[2] << ")"
      << ", center_uncertainty: (" << center_uncertainty(0, 0) << "," << center_uncertainty(0, 1)
      << "," << center_uncertainty(0, 2) << "," << center_uncertainty(1, 0) << ","
      << center_uncertainty(1, 1) << "," << center_uncertainty(1, 2) << ","
      << center_uncertainty(2, 0) << "," << center_uncertainty(2, 1) << ","
      << center_uncertainty(2, 2) << "), size: (" << size[0] << "," << size[1] << "," << size[2]
      << "), size_variance: (" << size_variance[0] << "," << size_variance[1] << ","
      << size_variance[2] << ", type: " << static_cast<int>(type) << ", confidence: " << confidence
      << ", velocity: (" << velocity[0] << "," << velocity[1] << "," << velocity[2]
      << "), velocity_confidence: " << velocity_confidence << ", tracking_time: " << tracking_time
      << ", latest_tracked_time: " << latest_tracked_time;
  return oss.str();
}

}  // namespace mid_fusion
}  // namespace perception
