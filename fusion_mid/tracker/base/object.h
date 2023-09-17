#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "object_types.h"

namespace perception {
namespace mid_fusion {

struct Point3D {
  double x;
  double y;
  double z;
};

struct Object {
  Object();
  ~Object() = default;
  std::string ToString() const;
  void Reset();
  // object id per frame, required
  int meas_id = -1;
  // @brief track id, required
  int track_id = -1;
  // message timestamp
  double timestamp = 0.0;
  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;
  std::string frame_id = "";
  // @brief center position of the boundingbox (x, y, z), required
  Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f position_uncertainty;
  // @brief center position of the boundingbox (x, y, z), required
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f center_uncertainty;
  // @brief corner point, required
  std::vector<Eigen::Vector3f> polygon;
  // @brief velocity of the object, required
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f velocity_uncertainty;
  // @brief if the velocity estimation is converged, true by default
  bool velocity_converged = true;
  // @brief velocity confidence, required
  float velocity_confidence = 1.0f;
  // 0-2Pi from east
  float theta;
  // @brief theta variance, required
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief size variance, required
  Eigen::Vector3f size_variance = Eigen::Vector3f(0, 0, 0);
  // @brief convex hull of the object, required
  // Point3f polygon;
  // @brief object type, required
  ObjectType type = ObjectType::UNKNOWN;

  // @brief type variance, required
  double type_variance = 1.0;
  // @brief probability for each type, required
  std::vector<float> type_probs;
  // @brief existence confidence, required
  float confidence = 1.0f;
  // oriented boundingbox information
  // @brief main direction of the object, required
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);

  // std::string DebugString() const {
  //   return absl::StrCat("id: ", track_id, ", ",      //
  //                       "time: ", timestamp, ", ",   //
  //                       "x: ", position.x(), ", ",   //
  //                       "y: ", position.y(), ", ",   //
  //                       "vx: ", velocity.x(), ", ",  //
  //                       "vy: ", velocity.y(), ", ",  //
  //                       "yaw: ", theta.Value());
  // }
};

using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

}  // namespace mid_fusion
}  // namespace perception
