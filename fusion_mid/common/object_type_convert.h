#pragma once

#include "common/proto/object.pb.h"
#include "perception/fusion_mid/tracker/base/object_types.h"

namespace perception {
namespace mid_fusion {

int CameraTypeConvertToFpnetIndex(perception::ObjectType object_type);
perception::ObjectType FpnetIndexConvertToLidarType(int index);

const std::map<perception::ObjectType, perception::mid_fusion::ObjectType> kMogo2TrackerTypeMap = {
    {perception::ObjectType::TYPE_UNKNOWN, perception::mid_fusion::ObjectType::UNKNOWN},
    {perception::ObjectType::TYPE_CAR, perception::mid_fusion::ObjectType::CAR},
    {perception::ObjectType::TYPE_TRUCK, perception::mid_fusion::ObjectType::TRUCK},
    {perception::ObjectType::TYPE_BUS, perception::mid_fusion::ObjectType::BUS},
    {perception::ObjectType::TYPE_BICYCLE, perception::mid_fusion::ObjectType::BICYCLE},
    {perception::ObjectType::TYPE_PEDESTRIAN, perception::mid_fusion::ObjectType::PEDESTRIAN},
    {perception::ObjectType::TYPE_VEGETATION, perception::mid_fusion::ObjectType::VEGETATION},
    {perception::ObjectType::TYPE_TRIANGLEROADBLOCK,perception::mid_fusion::ObjectType::TRAFFICCONE},
};

const std::map<perception::mid_fusion::ObjectType,perception::ObjectType> kTracker2MogoTypeMap = {
    {perception::mid_fusion::ObjectType::UNKNOWN, perception::ObjectType::TYPE_UNKNOWN},
    {perception::mid_fusion::ObjectType::CAR, perception::ObjectType::TYPE_CAR},
    {perception::mid_fusion::ObjectType::TRUCK, perception::ObjectType::TYPE_TRUCK},
    {perception::mid_fusion::ObjectType::BUS, perception::ObjectType::TYPE_BUS},
    {perception::mid_fusion::ObjectType::BICYCLE, perception::ObjectType::TYPE_BICYCLE},
    {perception::mid_fusion::ObjectType::PEDESTRIAN, perception::ObjectType::TYPE_PEDESTRIAN},
    {perception::mid_fusion::ObjectType::VEGETATION, perception::ObjectType::TYPE_VEGETATION},
    {perception::mid_fusion::ObjectType::TRAFFICCONE,perception::ObjectType::TYPE_TRIANGLEROADBLOCK},
};

}  // namespace mid_fusion
}  // namespace perception
