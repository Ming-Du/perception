
#include "trigger_info_pub.h"

namespace perception {
namespace mid_fusion {

TriggerManager::TriggerManager() {}

void TriggerManager::init() {}

void TriggerManager::perception(const perception::TrackedObjects& lidar_measurement,
                                std::vector<std::string>& trigger_info) {
  if (lidar_measurement.objs_size() == 0) {
    return;
  }
  trigger_info.clear();
  bool is_vegtation = false;
  bool is_cone = false;
  bool is_ped_congest = false;
  int ped_num = 0;
  for (size_t i = 0; i < lidar_measurement.objs_size(); i++) {
    const perception::Object& object = lidar_measurement.objs(i).obj();
    if (object.type() == perception::ObjectType::TYPE_VEGETATION) {
      is_vegtation = true;
    }

    if (object.type() == perception::ObjectType::TYPE_TRIANGLEROADBLOCK) {
      is_cone = true;
    }

    if (object.center().x() > 1.0 && object.center().y() < 30.0 &&
        (object.type() == perception::ObjectType::TYPE_BICYCLE ||
         object.type() == perception::ObjectType::TYPE_PEDESTRIAN)) {
      ped_num++;
    }
  }
  if (ped_num >= 20) {
    is_ped_congest = true;
  }

  if (is_cone) {
    trigger_info.push_back("Traffic Traffic control devices");
    trigger_info.push_back("cone");
  }

  if (is_vegtation) {
    trigger_info.push_back("Object");
    trigger_info.push_back("vegtation");
  }

  if (is_ped_congest) {
    trigger_info.push_back("Traffic");
    trigger_info.push_back("Pedestrian congestion");
  }
}

}  // namespace mid_fusion

}  // namespace perception
