#pragma once
#include "base_convertor.h"
#include "common/camera_util.h"

namespace perception {
namespace fusion {

class VisualConvertor : public BaseConvertor {
   public:
    VisualConvertor(){};
    virtual ~VisualConvertor(){};
    //  void Fusion2Pnc();
    // void Fusion2App();
    //  void Mogo2Fusion();

    // camera 2D object convert
    bool Mogo2Fusion(const perception::TrackedObject& vidar_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object,
                      input_sensor_ input_sensor);
    // vidar object convert
    bool Mogo2Fusion(const perception::VisualObject& camera_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object);
};
}  // namespace fusion
}  // namespace perception