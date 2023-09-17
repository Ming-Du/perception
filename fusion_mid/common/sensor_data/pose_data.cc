#include "pose_data.h"

namespace perception {
namespace mid_fusion {
Eigen::Quaternionf PoseData::GetQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3, 3>(0, 0);

  return q;
}
}  // namespace mid_fusion
}  // namespace perception
