#include "perception/fusion_mid/common/object_type_convert.h"

namespace perception {
namespace mid_fusion {

int CameraTypeConvertToFpnetIndex(perception::ObjectType object_type) {
  switch (object_type) {
    case perception::ObjectType::TYPE_UNKNOWN:
    case perception::ObjectType::TYPE_UNKNOWN_BIG:
    case perception::ObjectType::TYPE_UNKNOWN_SMALL:
    case perception::ObjectType::TYPE_UNKNOWN_STATIC:
    case perception::ObjectType::TYPE_UNKNOWN_DYNAMIC:
    case perception::ObjectType::TYPE_LIGHT:
    case perception::ObjectType::TYPE_SIGN:
    case perception::ObjectType::TYPE_RED:
    case perception::ObjectType::TYPE_GREEN:
    case perception::ObjectType::TYPE_YELLOW:
    case perception::ObjectType::TYPE_BLACK:
      return 0;
    case perception::ObjectType::TYPE_PEDESTRIAN:
      return 1;
    case perception::ObjectType::TYPE_BICYCLE:
    case perception::ObjectType::TYPE_MOTOR:
      return 3;
    case perception::ObjectType::TYPE_RIDER:
      return 4;
    case perception::ObjectType::TYPE_CAR:
      return 5;
    case perception::ObjectType::TYPE_TRUCK:
    case perception::ObjectType::TYPE_TRAIN:
      return 6;
    case perception::ObjectType::TYPE_BUS:
      return 7;
    case perception::ObjectType::TYPE_TRIANGLEROADBLOCK:
    case perception::ObjectType::TYPE_WARNINGTRIANGLE:
      return 9;
    default:
      return 0;
  }
}

perception::ObjectType FpnetIndexConvertToLidarType(int index) {
  switch (index) {
    case 0:
      return perception::ObjectType::TYPE_UNKNOWN;
    case 1:
      return perception::ObjectType::TYPE_PEDESTRIAN;
    case 2:
    case 3:
    case 4:
      return perception::ObjectType::TYPE_BICYCLE;
    case 5:
      return perception::ObjectType::TYPE_CAR;
    case 6:
      return perception::ObjectType::TYPE_TRUCK;
    case 7:
      return perception::ObjectType::TYPE_BUS;
    case 9:
      return perception::ObjectType::TYPE_TRIANGLEROADBLOCK;
    case 8:
    case 10:
    case 11:
      return perception::ObjectType::TYPE_UNKNOWN;
    case 12:
      return perception::ObjectType::TYPE_VEGETATION;
    default:
      return perception::ObjectType::TYPE_UNKNOWN;
  }
}

}  // namespace mid_fusion
}  // namespace perception
