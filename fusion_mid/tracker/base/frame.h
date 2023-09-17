#pragma once

#include <memory>
#include <vector>

#include "perception/fusion_mid/tracker/base/object.h"
#include "perception/fusion_mid/tracker/base/object_types.h"
#include "Eigen/Dense"

namespace perception {
namespace mid_fusion {

struct Frame {
  Frame() { sensor2world_pose.setIdentity(); }

  void Reset() {
    timestamp = 0.0;
    objects.clear();
    sensor2world_pose.setIdentity();

  }
  // @brief sensor information

  double timestamp = 0.0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Affine3d sensor2world_pose;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<const Frame> FrameConstPtr;

}  // namespace mid_fusion
}  // namespace perception
