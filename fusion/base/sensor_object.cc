#include "sensor_object.h"

#include "object_pool_types.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "sensor_frame.h"

namespace perception {
namespace fusion {

// SensorObject implementations
SensorObject::SensorObject(const std::shared_ptr<const Object>& object_ptr)
    : object_(object_ptr), frame_header_(nullptr) {}

SensorObject::SensorObject(const std::shared_ptr<const Object>& object_ptr,
                           const std::shared_ptr<const SensorFrameHeader>& frame_header)
    : object_(object_ptr), frame_header_(frame_header) {}

SensorObject::SensorObject(const std::shared_ptr<const Object>& object_ptr,
                           const std::shared_ptr<SensorFrame>& frame_ptr)
    : object_(object_ptr),
      frame_header_((frame_ptr == nullptr) ? nullptr : frame_ptr->GetHeader()) {}

double SensorObject::GetTimestamp() const {
  if (frame_header_ == nullptr) {
    return 0.0;
  }

  return frame_header_->timestamp;
}

bool SensorObject::GetRelatedFramePose(Eigen::Affine3d* pose) const {
  if (pose == nullptr) {
    ROS_WARN("GetRelatedFramePose: pose is not available!");
    return false;
  }
  if (frame_header_ == nullptr) {
    return false;
  }

  *pose = frame_header_->sensor2world_pose;
  return true;
}

std::string SensorObject::GetSensorId() const {
  if (frame_header_ == nullptr) {
    return std::string("");
  }

  return frame_header_->sensor_info.name();
}

base::SensorType SensorObject::GetSensorType() const {
  if (frame_header_ == nullptr) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  return frame_header_->sensor_info.type();
}

// FusedObject implementations
FusedObject::FusedObject() {
  ObjectPool& object_pool = ObjectPool::Instance();
  object_ = object_pool.Get();
}

bool IsLidar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsLidar(type);
}

bool IsRadar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsRadar(type);
}

bool IsCamera(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsCamera(type);
}

// Modify @jiangnan:add falcon lidar
bool IsFalconLidar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsFalconLidar(type);
}

bool IsObu(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsObu(type);
}

bool IsVidar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return perception::base::SensorManager::Instance()->IsVidar(type);
}

}  // namespace fusion
}  // namespace perception
