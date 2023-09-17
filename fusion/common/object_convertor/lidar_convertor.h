#pragma once
#include "base_convertor.h"

namespace perception {
namespace fusion {

class LidarConvertor : public BaseConvertor {
   public:
    LidarConvertor(){};
    virtual ~LidarConvertor(){};


    bool Mogo2Fusion(const perception::TrackedObject& lidar_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object,
                     input_sensor_ input_sensor);

    std::unordered_map<int, perception::RoadType> RoadType = {{0, perception::RoadType::RoadType_ROAD},
                                                              {1, perception::RoadType::RoadType_ROADSIDE},
                                                              {2, perception::RoadType::RoadType_FLOWERBEDSSIDE},
                                                              {3, perception::RoadType::RoadType_FENCESIDE}};
};
}  // namespace fusion
}  // namespace perception