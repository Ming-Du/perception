#ifndef PERCEPTION_FUSION2_TRACKER_OPTIMIZER_H
#define PERCEPTION_FUSION2_TRACKER_OPTIMIZER_H

#include <queue>
#include "Eigen/Core"
#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "iostream"
#include "perception/lib/registerer/registerer.h"
#include "stdio.h"
#include "vector"

namespace perception {
namespace fusion {
/**
 * Add by jiangnan :tracker optimizer
 * Classification of dynamic and static trackedobject , setting of static  motion state
 **/
class tracker_optimizer {
 public:
  tracker_optimizer();
  ~tracker_optimizer();
  explicit tracker_optimizer(TrackPtr track) : track_ref_(track){};
  tracker_optimizer(const tracker_optimizer&) = delete;
  tracker_optimizer& operator=(const tracker_optimizer&) = delete;

  void Init();

  void StoreHistoryTrajectory(const Eigen::Vector3d& position, const Eigen::Vector3f& velocity);

  bool CheckMotionStateIsStatic();

  void SetStaticMotion();

  void CheckYawIsReasonable();

  void GetStaticPosition();

 private:
  int window_size_;
  float position_threshold;
  std::vector<Eigen::Vector3d> history_position;
  std::vector<Eigen::Vector3f> history_velocity;

  Eigen::Vector3d static_position;
  // Modify @ jiangnan :static or motin state  counter
  int static_count;
  int motion_count;

 protected:
  TrackPtr track_ref_;
};
}  // namespace fusion
}  // namespace perception
#endif  // PERCEPTION_FUSION2_TRACKER_OPTIMIZER_H
