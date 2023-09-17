#include "sensor_frame.h"

namespace perception {
namespace fusion {

SensorFrame::SensorFrame() {
  header_.reset(new SensorFrameHeader());
}

SensorFrame::SensorFrame(const fusion::FrameConstPtr& base_frame_ptr) {
  Initialize(base_frame_ptr);
}

void SensorFrame::Initialize(const fusion::FrameConstPtr& base_frame_ptr) {
  header_.reset(new SensorFrameHeader(base_frame_ptr->sensor_info, base_frame_ptr->timestamp,
                                      base_frame_ptr->sensor2world_pose));

  lidar_frame_supplement_ = base_frame_ptr->lidar_frame_supplement;
  radar_frame_supplement_ = base_frame_ptr->radar_frame_supplement;
  camera_frame_supplement_ = base_frame_ptr->camera_frame_supplement;
  vidar_frame_supplement_ = base_frame_ptr->vidar_frame_supplement;  // duming vidar
  falcon_lidar_frame_supplement_ =
      base_frame_ptr->falcon_lidar_frame_supplement;  // Modify @jiangnan : add falcon lidar;
  // Modify(@liuxinyu): obu_test
  obu_frame_supplement_ = base_frame_ptr->obu_frame_supplement;

  const auto& base_objects = base_frame_ptr->objects;
  foreground_objects_.reserve(base_objects.size());

  for (const auto& base_obj : base_objects) {
    SensorObjectPtr obj(new SensorObject(base_obj, header_));
    if (base_obj->lidar_supplement.is_background) {
      background_objects_.emplace_back(obj);
    } else {
      foreground_objects_.emplace_back(obj);
    }
  }
}

void SensorFrame::Initialize(const fusion::FrameConstPtr& base_frame_ptr, const SensorPtr& sensor) {
  Initialize(base_frame_ptr);
}

std::string SensorFrame::GetSensorId() const {
  if (header_ == nullptr) {
    return std::string("");
  }

  return header_->sensor_info.name();
}

base::SensorType SensorFrame::GetSensorType() const {
  if (header_ == nullptr) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  return header_->sensor_info.type();
}

}  // namespace fusion
}  // namespace perception
