#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>

#include "lib/interface/base_fusion_system.h"

namespace perception {
namespace fusion {

struct ObstacleMultiSensorFusionParam {
  std::vector<std::string> main_sensors;
  std::string fusion_method;
};
class ObstacleMultiSensorFusion {
 public:
  ObstacleMultiSensorFusion() = default;
  ~ObstacleMultiSensorFusion() = default;
  ObstacleMultiSensorFusion(const ObstacleMultiSensorFusion&) = delete;
  ObstacleMultiSensorFusion& operator=(const ObstacleMultiSensorFusion&) = delete;
  bool Init(const ObstacleMultiSensorFusionParam& param);
  bool Process(const fusion::FrameConstPtr& frame, std::vector<fusion::ObjectPtr>* objects);

  std::string Name() const { return "ObstacleMultiSensorFusion"; }

 protected:
  BaseFusionSystem* fusion_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
