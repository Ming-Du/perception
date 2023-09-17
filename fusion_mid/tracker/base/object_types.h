#pragma once

#include <map>
#include <string>

namespace perception {
namespace mid_fusion {
// @brief general object type
enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  CAR = 5,
  BUS = 6,
  TRUCK = 7,
  VEGETATION = 8,
  TRAFFICCONE = 9,
  MAX_OBJECT_TYPE = 10,
};

}  // namespace mid_fusion
}  // namespace perception
