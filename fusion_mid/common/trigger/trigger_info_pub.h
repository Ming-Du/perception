#pragma once

#include "common/proto/object.pb.h"
namespace perception {
namespace mid_fusion {

class TriggerManager {
 public:
  using Ptr = std::shared_ptr<TriggerManager>;
  TriggerManager();

  void init();
  void perception(const perception::TrackedObjects& lidar_measurement,
                  std::vector<std::string>& trigger_info);

 private:
};

}  // namespace mid_fusion
}
 