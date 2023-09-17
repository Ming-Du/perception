#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "common/proto/object.pb.h"
#include "common/fusion_define.h"

namespace perception {
namespace fusion {

class EventProcessor {
 private:
  /* data */
 public:
  EventProcessor(/* args */) {}
  ~EventProcessor() {}

  void ZombieCarProcessor(TrackedObjects &tracked_objects, double zombie_thr);
  int ZombieCarProcessor(double status_duration, double zombie_thr);
  void V2nEventProcessor(TrackedObjects &tracked_objects);

  std::map<int, perception::AdditionalAttribute> event_map = {
      {0, perception::AdditionalAttribute::ATTR_UNKNOWN},
      {1, perception::AdditionalAttribute::ATTR_ZOMBIE},
      {2, perception::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION},
      {3, perception::AdditionalAttribute::ATTR_STATIC},
      {4, perception::AdditionalAttribute::ATTR_ACCIDENT}};
};

}  // namespace fusion
}  // namespace perception