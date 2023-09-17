#pragma once
#include "base/v2x_param.h"
#include "base_convertor.h"

namespace perception {
namespace fusion {

class ObuConvertor : public BaseConvertor {
   public:
    ObuConvertor(){};
    virtual ~ObuConvertor(){};

    bool Mogo2Fusion(const perception::TrackedObject& sensor_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object,
                      input_sensor_ input_sensor) {
        // this function does not need to be used
        return false;
    };

    bool Mogo2Fusion(const perception::ObuObject& obu_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object);

    void AddObuToPnc(std::list<ObuObjects> obu_objects_list,
                     const localization::Localization& localization,
                     perception::TrackedObjects& output_objects,
                     const v2x_param& v2x_param);

    void Mogo2OBU(perception::TrackedObjects& output_objects, perception::TrackedObjects& output_objects_obu);

    void StoreV2xObjs(perception::ObuObjects& obu_objects,
                      const localization::Localization& localization,
                      std::list<localization::Localization> global_localizations,
                      std::list<ObuObjects>& obu_objects_v2n_,
                      std::list<ObuObjects>& obu_objects_v2i_);

   private:
    void AssginContoursForObuObj(perception::Object* obj);
    void SetPointLFD(const perception::Object& object, geometry::Point& point);
    void SetPointLBD(const perception::Object& object, geometry::Point& point);
    void SetPointRFD(const perception::Object& object, geometry::Point& point);
    void SetPointRBD(const perception::Object& object, geometry::Point& point);
    void SetPointLFU(const perception::Object& object, geometry::Point& point);
    void SetPointLBU(const perception::Object& object, geometry::Point& point);
    void SetPointRFU(const perception::Object& object, geometry::Point& point);
    void SetPointRBU(const perception::Object& object, geometry::Point& point);
    void PointRotate(const double cosYaw, const double sinYaw, const double dcx, const double dcy, geometry::Point& point);
    void V2xSource2FusionConvertor(const perception::ObjectSource& mogo_source, perception::fusion::ObjectSource& fusion_source);
    bool QueryNearestLoc(const double& timestamp,
                         std::list<localization::Localization> global_localizations,
                         localization::Localization& localization);
    bool RewriteObuPNTObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization);
    void RewriteObuRSIObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization);
    void RewriteObuObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization);
    void WGS84Angle2Mct(perception::ObuObject* obu_object_ptr, const localization::Localization& localization);
    bool BlindMode(const perception::ObuObject obu_object,
                               const localization::Localization& localization,
                               perception::TrackedObjects& output_objects,
                               const v2x_param& v2x_param);
    bool BeyondRangeMode( const perception::ObuObject obu_object, const v2x_param& v2x_param);
    float ComputeObjectPolygonIouDistance(const perception::ObuObject obu_object,
                                          const localization::Localization& localization,
                                          const perception::TrackedObject& fusion_object);
};
}  // namespace fusion
}  // namespace perception