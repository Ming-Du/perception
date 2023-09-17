#ifndef MID_FUSION_SENSOR_DATA_POSE_DATA_HPP_
#define MID_FUSION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace perception {
namespace mid_fusion {
class PoseData {
 public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double time = 0.0;

 public:
  Eigen::Quaternionf GetQuaternion();
};
}  // namespace mid_fusion
}  // namespace perception

#endif