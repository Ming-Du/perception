#include "obstacle_multi_sensor_fusion.h"

namespace perception {
namespace fusion {

bool ObstacleMultiSensorFusion::Init(const ObstacleMultiSensorFusionParam& param) {
  if (fusion_ != nullptr) {
    ROS_DEBUG("Init: Already inited.");
    return true;
  }
  fusion_ = BaseFusionSystemRegisterer::GetInstanceByName(param.fusion_method);

  FusionInitOptions init_options;
  init_options.main_sensors = param.main_sensors;
  if (fusion_ == nullptr || !fusion_->Init(init_options)) {
    ROS_DEBUG_STREAM("Init: Failed to Get Instance or Initialize " << param.fusion_method);
    return false;
  }
  return true;
}

bool ObstacleMultiSensorFusion::Process(const fusion::FrameConstPtr& frame,
                                        std::vector<fusion::ObjectPtr>* objects) {
  FusionOptions options;
  return fusion_->Fuse(options, frame, objects);
}

}  // namespace fusion
}  // namespace perception
