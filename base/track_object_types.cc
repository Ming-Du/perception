//
// Created by moriarty on 4/30/21.
//

#include "track_object_types.h"
namespace perception {
namespace base {
//type, {half_width, half_length}
const std::map<perception::ObjectType, std::array<float, 2>> track_object_types = {
  {perception::ObjectType::TYPE_PEDESTRIAN, {1.0 / 2, 1.0 / 2}},
  {perception::ObjectType::TYPE_MOTOR,      {1.0 / 2, 2.0 / 2}},
  {perception::ObjectType::TYPE_CAR,        {1.825 / 2, 4.640 / 2}},
  {perception::ObjectType::TYPE_TRUCK,      {2.5 / 2, 12.0 / 2}},
  {perception::ObjectType::TYPE_BUS,        {2.5 / 2, 12.0 / 2}},
  {perception::ObjectType::TYPE_BICYCLE,      {1.0 / 2, 3.0 / 2}}
};
}//namespace base
}//namespace perception