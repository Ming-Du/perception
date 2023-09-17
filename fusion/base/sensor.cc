#include "sensor.h"

#include "common/include/log.h"

namespace perception {
namespace fusion {

size_t Sensor::kMaxCachedFrameNum = 10;

void Sensor::QueryLatestFrames(double timestamp, std::vector<SensorFramePtr>* frames) {
  if (frames == nullptr) {
    ROS_ERROR("QueryLatestFrames: frames are not available!");
    return;
  }

  frames->clear();
  for (size_t i = 0; i < frames_.size(); ++i) {
    if (frames_[i]->GetTimestamp() > latest_query_timestamp_ &&
        frames_[i]->GetTimestamp() <= timestamp) {
      frames->push_back(frames_[i]);
    }
  }
  latest_query_timestamp_ = timestamp;
}

SensorFramePtr Sensor::QueryLatestFrame(double timestamp) {
  SensorFramePtr latest_frame = nullptr;
  for (size_t i = 0; i < frames_.size(); ++i) {
    if (frames_[i]->GetTimestamp() > latest_query_timestamp_ &&
        frames_[i]->GetTimestamp() <= timestamp) {
      latest_frame = frames_[i];
      latest_query_timestamp_ = frames_[i]->GetTimestamp();
    }
  }
  return latest_frame;
}

bool Sensor::GetPose(double timestamp, Eigen::Affine3d* pose) const {
  if (pose == nullptr) {
    ROS_ERROR("GetPose: pose is not available!");
    return false;
  }
  for (int i = static_cast<int>(frames_.size()) - 1; i >= 0; --i) {
    double time_diff = timestamp - frames_[i]->GetTimestamp();
    if (fabs(time_diff) < 1.0e-3) {  // > ?
      return frames_[i]->GetPose(pose);
    }
  }
  ROS_WARN_STREAM("GetPose: Failed to find pose for timestamp: " << timestamp);
  return false;
}

void Sensor::AddFrame(const fusion::FrameConstPtr& frame_ptr) {
  SensorFramePtr frame(new SensorFrame(frame_ptr));
  if (frames_.size() == kMaxCachedFrameNum) {
    frames_.pop_front();
  }
  frames_.emplace_back(frame);
}

}  // namespace fusion
}  // namespace perception
