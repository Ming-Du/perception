#pragma once
#include "base_convertor.h"

namespace perception {
namespace fusion {

class RadarConvertor : public BaseConvertor {
   public:
    RadarConvertor(){};
    virtual ~RadarConvertor(){};

    bool Mogo2Fusion(const perception::TrackedObject& sensor_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object,
                      input_sensor_ input_sensor) {
        // this function does not need to be used
        return false;
    };
    bool Mogo2Fusion(const perception::RadarObject& radar_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object);
};
}  // namespace fusion
}  // namespace perception