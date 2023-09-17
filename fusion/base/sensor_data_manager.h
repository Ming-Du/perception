
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "frame.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "sensor.h"
#include "sensor_frame.h"
#include "common/include/macros.h"

namespace perception {
namespace fusion {

class SensorDataManager {
 public:
  bool Init();

  void Reset();

  void AddSensorMeasurements(const fusion::FrameConstPtr& frame_ptr);

  bool IsLidar(const fusion::FrameConstPtr& frame_ptr);
  bool IsRadar(const fusion::FrameConstPtr& frame_ptr);
  bool IsCamera(const fusion::FrameConstPtr& frame_ptr);
  // Modify @jiangnan: add falcon lidar
  bool IsFalconLidar(const fusion::FrameConstPtr& frame_ptr);
  // Modify(@liuxinyu): obu_test
  bool IsObu(const fusion::FrameConstPtr& frame_ptr);

  bool IsVidar(const fusion::FrameConstPtr& frame_ptr);

  // Getter
  void GetLatestSensorFrames(double timestamp,
                             const std::string& sensor_id,
                             std::vector<SensorFramePtr>* frames) const;

  void GetLatestFrames(double timestamp, std::vector<SensorFramePtr>* frames) const;

  bool GetPose(const std::string& sensor_id, double timestamp, Eigen::Affine3d* pose) const;

  base::BaseCameraModelPtr GetCameraIntrinsic(const std::string& sensor_id) const;
  base::BaseCameraDistortionModelPtr GetCameraIntrinsicDistortion(const std::string& sensor_id) const;
 private:
  bool inited_ = false;
  std::unordered_map<std::string, SensorPtr> sensors_;

  const perception::base::SensorManager* sensor_manager_ = nullptr;

  FRIEND_TEST(SensorDataManagerTest, test);
  DECLARE_SINGLETON(SensorDataManager)
};

}  // namespace fusion
}  // namespace perception
