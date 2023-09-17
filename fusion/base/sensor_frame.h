#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "base/base_forward_declaration.h"
#include "base/sensor_object.h"
#include "frame.h"

namespace perception {
namespace fusion {

struct SensorFrameHeader {
  base::SensorInfo sensor_info;
  double timestamp = 0.0;
  Eigen::Affine3d sensor2world_pose;

  SensorFrameHeader() = default;
  SensorFrameHeader(const base::SensorInfo& info, double ts, const Eigen::Affine3d& pose)
      : sensor_info(info), timestamp(ts), sensor2world_pose(pose) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class SensorFrame {
 public:
  SensorFrame();

  explicit SensorFrame(const fusion::FrameConstPtr& base_frame_ptr);

  void Initialize(const fusion::FrameConstPtr& base_frame_ptr);

  void Initialize(const fusion::FrameConstPtr& base_frame_ptr, const SensorPtr& sensor);

  // Getter
  inline double GetTimestamp() const { return header_->timestamp; }

  inline bool GetPose(Eigen::Affine3d* pose) const {
    if (pose == nullptr) {
      ROS_ERROR("GetPose: pose is not available!");
      return false;
    }
    *pose = header_->sensor2world_pose;
    return true;
  }

  inline std::vector<SensorObjectPtr>& GetForegroundObjects() { return foreground_objects_; }

  inline const std::vector<SensorObjectPtr>& GetForegroundObjects() const {
    return foreground_objects_;
  }

  inline std::vector<SensorObjectPtr>& GetBackgroundObjects() { return background_objects_; }

  inline const std::vector<SensorObjectPtr>& GetBackgroundObjects() const {
    return background_objects_;
  }

  std::string GetSensorId() const;

  base::SensorType GetSensorType() const;

  SensorFrameHeaderConstPtr GetHeader() const { return header_; }

 private:
  std::vector<SensorObjectPtr> foreground_objects_;
  std::vector<SensorObjectPtr> background_objects_;

  // sensor-specific frame supplements
  LidarFrameSupplement lidar_frame_supplement_;
  RadarFrameSupplement radar_frame_supplement_;
  CameraFrameSupplement camera_frame_supplement_;
  CameraFrameSupplement vidar_frame_supplement_;
  LidarFrameSupplement falcon_lidar_frame_supplement_;  // Modify @jiangnan : add falcon lidar;
  // Modify(@liuxinyu): obu_test
  ObuFrameSupplement obu_frame_supplement_;

  SensorFrameHeaderPtr header_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
