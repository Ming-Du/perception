#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <ros/ros.h>

#include "common/include/macros.h"
#include "common/proto/sensor_meta.pb.h"
#include "perception/base/camera.h"
#include "perception/base/distortion_model.h"
#include "perception/base/perception_gflags.h"

namespace perception {
namespace base {

class SensorManager {
 public:
  bool Init();

  bool IsSensorExist(const std::string& name) const;

  bool GetSensorInfo(const std::string& name,
                     perception::base::SensorInfo* sensor_info) const;

  std::shared_ptr<BaseCameraDistortionModel> GetDistortCameraModel(
      const std::string& name) const;

  std::shared_ptr<BaseCameraModel> GetUndistortCameraModel(
      const std::string& name) const;

  // sensor type functions
  // bool IsHdLidar(const std::string& name) const;
  // bool IsHdLidar(const perception::base::SensorType& type) const;
  // bool IsLdLidar(const std::string& name) const;
  // bool IsLdLidar(const perception::base::SensorType& type) const;

  bool IsLidar(const std::string& name) const;
  bool IsLidar(const SensorType& type) const;
  bool IsRadar(const std::string& name) const;
  bool IsRadar(const SensorType& type) const;
  bool IsCamera(const std::string& name) const;
  bool IsCamera(const SensorType& type) const;
  // Modify(@liuxinyu): obu_test
  bool IsObu(const std::string& name) const;
  bool IsObu(const SensorType& type) const;
  // bool IsUltrasonic(const std::string& name) const;
  // bool IsUltrasonic(const perception::base::SensorType& type) const;

  bool IsVidar(const std::string &name) const;
  bool IsVidar(const SensorType &type) const;
  
  //Modify @jiangnan: add falcon lidar
  bool IsFalconLidar(const std::string& name) const;
  bool IsFalconLidar(const SensorType& type) const;

  // sensor frame id function
  std::string GetFrameId(const std::string& name) const;
  // sensor topic
  std::string GetTopic(const std::string& name) const;
  std::string GetBaseFrameId() const { return base_frame_id_; }
  std::unordered_map<std::string, perception::base::SensorInfo>
  GetSensorInfoMap() {
      return sensor_info_map_;
  }

 private:
  // inline std::string IntrinsicPath(const std::string& frame_id) {
  //   std::string intrinsics =
  //       FLAGS_obs_sensor_intrinsic_path + "/" + frame_id + "_intrinsics.yaml";
  //   return intrinsics;
  // }

 private:
  std::mutex mutex_;
  bool inited_ = false;

  std::unordered_map<std::string, perception::base::SensorInfo>
      sensor_info_map_;
  std::unordered_map<std::string, std::shared_ptr<BaseCameraDistortionModel>>
      distort_model_map_;
  std::unordered_map<std::string, std::shared_ptr<BaseCameraModel>>
      undistort_model_map_;
  std::string base_frame_id_;

  DECLARE_SINGLETON(SensorManager)
};

}  // namespace base
}  // namespace perception
