#pragma once

#include <memory>
#include <vector>

#include "Eigen/Dense"

#include "frame_supplement.h"
#include "object.h"
#include "object_types.h"
#include "common/proto/sensor_meta.pb.h"

namespace perception {
namespace fusion {

struct alignas(16) Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame() { sensor2world_pose.setIdentity(); }

  void Reset() {
    timestamp = 0.0;
    objects.clear();
    sensor2world_pose.setIdentity();
    // TODO:check whether need to reset the sensor ifo
    // sensor_info.Reset();
    lidar_frame_supplement.Reset();
    radar_frame_supplement.Reset();
    camera_frame_supplement.Reset();
    falcon_lidar_frame_supplement.Reset();  // Modify @jiangnan
    obu_frame_supplement.Reset();
    vidar_frame_supplement.Reset();
  }
  // @brief sensor information
  base::SensorInfo sensor_info;

  double timestamp = 0.0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Affine3d sensor2world_pose;

  // sensor-specific frame supplements
  LidarFrameSupplement lidar_frame_supplement;
  RadarFrameSupplement radar_frame_supplement;
  CameraFrameSupplement camera_frame_supplement;
  CameraFrameSupplement vidar_frame_supplement;  // duming
  UltrasonicFrameSupplement ultrasonic_frame_supplement;
  // Modify @jiangnan :add solid state lidar
  LidarFrameSupplement falcon_lidar_frame_supplement;
  // Modify(@liuxinyu): obu_test
  ObuFrameSupplement obu_frame_supplement;
};

typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<const Frame> FrameConstPtr;

}  // namespace fusion
}  // namespace perception
