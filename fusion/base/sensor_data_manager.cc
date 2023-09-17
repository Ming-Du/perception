#include "sensor_data_manager.h"

#include <algorithm>
#include <utility>

#include "common/include/log.h"

namespace perception {
namespace fusion {

SensorDataManager::SensorDataManager() {
  CHECK_EQ(this->Init(), true);
}

bool SensorDataManager::Init() {
  if (inited_) {
    return true;
  }
  sensor_manager_ = perception::base::SensorManager::Instance();
  inited_ = true;
  return true;
}

void SensorDataManager::Reset() {
  inited_ = false;
  sensor_manager_ = nullptr;
  sensors_.clear();
}

void SensorDataManager::AddSensorMeasurements(const fusion::FrameConstPtr& frame_ptr) {
  const base::SensorInfo& sensor_info = frame_ptr->sensor_info;
  std::string sensor_id = sensor_info.name();
  const auto it = sensors_.find(sensor_id);
  SensorPtr sensor_ptr = nullptr;
  if (it == sensors_.end()) {
    if (!sensor_manager_->IsSensorExist(sensor_id)) {
      ROS_ERROR_STREAM("AddSensorMeasurements: Failed to find sensor " << sensor_id
                                                                       << " in sensor manager.");
      return;
    }
    sensor_ptr.reset(new Sensor(sensor_info));
    sensors_.emplace(sensor_id, sensor_ptr);
  } else {
    sensor_ptr = it->second;
  }
  sensor_ptr->AddFrame(frame_ptr);
}

bool SensorDataManager::IsLidar(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsLidar(type);
}

bool SensorDataManager::IsRadar(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsRadar(type);
}

bool SensorDataManager::IsCamera(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsCamera(type);
}

bool SensorDataManager::IsObu(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsObu(type);
}

bool SensorDataManager::IsVidar(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsVidar(type);
}

// Modify @jiangnan
bool SensorDataManager::IsFalconLidar(const fusion::FrameConstPtr& frame_ptr) {
  base::SensorType type = frame_ptr->sensor_info.type();
  return sensor_manager_->IsFalconLidar(type);
}

void SensorDataManager::GetLatestSensorFrames(double timestamp,
                                              const std::string& sensor_id,
                                              std::vector<SensorFramePtr>* frames) const {
  if (frames == nullptr) {
    ROS_ERROR("GetLatestSensorFrames: Nullptr error.");
    return;
  }
  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    return;
  }
  return it->second->QueryLatestFrames(timestamp, frames);
}

void SensorDataManager::GetLatestFrames(double timestamp,
                                        std::vector<SensorFramePtr>* frames) const {
  if (frames == nullptr) {
    ROS_ERROR("GetLatestFrames: Nullptr error.");
    return;
  }

  frames->clear();
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    SensorFramePtr frame = it->second->QueryLatestFrame(timestamp);
    if (frame != nullptr) {
      frames->push_back(frame);
    }
  }

  if (frames->empty()) {
    return;
  }

  std::sort(frames->begin(), frames->end(), [](const SensorFramePtr& p1, const SensorFramePtr& p2) {
    return p1->GetTimestamp() < p2->GetTimestamp();
  });
}

bool SensorDataManager::GetPose(const std::string& sensor_id,
                                double timestamp,
                                Eigen::Affine3d* pose) const {
  if (pose == nullptr) {
    ROS_ERROR("Nullptr error.");
    return false;
  }
  // ROS_DEBUG_STREAM("sensor_id = " << sensor_id);
  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    ROS_ERROR_STREAM("Failed to find sensor " << sensor_id << " for get pose.");
    return false;
  }
  // ROS_DEBUG_STREAM("it->second->GetSensorId() = " << it->second->GetSensorId());
  return it->second->GetPose(timestamp, pose);
}

base::BaseCameraModelPtr SensorDataManager::GetCameraIntrinsic(const std::string& sensor_id) const {
  return sensor_manager_->GetUndistortCameraModel(sensor_id);
}

base::BaseCameraDistortionModelPtr SensorDataManager::GetCameraIntrinsicDistortion(const std::string& sensor_id) const {
  return sensor_manager_->GetDistortCameraModel(sensor_id);
}
}  // namespace fusion
}  // namespace perception
