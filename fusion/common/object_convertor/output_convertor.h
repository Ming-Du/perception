#pragma once
#include <common/proto/message_pad.pb.h>
#include "base_convertor.h"

namespace perception {
namespace fusion {

class OutputConvertor : public BaseConvertor {
   public:
    OutputConvertor(){};
    virtual ~OutputConvertor(){};

    bool Mogo2Fusion(const perception::TrackedObject& sensor_object,
                     const localization::Localization& localization,
                     fusion::ObjectPtr& fusion_object,
                      input_sensor_ input_sensor) {
        // not use this function
        return false;
    };

    void Fusion2Mogo(const fusion::FrameConstPtr& frame,
                     const std::vector<fusion::ObjectPtr>& fusion_objects,
                     TrackedObjects& tracked_objects,
                     const localization::Localization& localization);

    void Fusion2App(const fusion::FrameConstPtr& frame,
                    const std::vector<fusion::ObjectPtr>& fusion_objects,
                    mogo::telematics::pad::TrackedObjects& app_tracked_objects,
                    const localization::Localization localization,
                    double zombie_thr);

    void ObjectFusion2Mogo(const fusion::ObjectPtr& fusion_object_ptr,
                           perception::TrackedObject* tracker_obj_ptr,
                           const localization::Localization& localization);

    void ApolloSource2MogoConvertor(const perception::fusion::ObjectSource& apollo_source, perception::ObjectSource& mogo_source);

    void AssignPolygonToRadarTrack(const fusion::ObjectPtr& fusion_object_ptr, const localization::Localization& localization);

    bool OutRoiObjectFilter(const fusion::ObjectPtr& fusion_object, bool has_obu);

    void OverlapObjectFilter(const std::vector<fusion::ObjectPtr>& fusion_objects, std::vector<fusion::ObjectPtr>& filter_objects);
  
    void AddVirtualObj(perception::TrackedObjects &tracked_objects,
                       const std::vector<perception::VirtualObject> &virtual_objects,
                       const localization::Localization &localization);

    enum ClassID {
        Background = 0,
        Person = 1,
        Bicycle = 2,
        Car = 3,
        MotorCycle = 4,
        TrafficSign = 5,
        Bus = 6,
        CellPhone = 7,
        Truck = 8,
        Bottle = 9,
        TrafficLight = 10,
        Rider = 11,
        TriangleRoadblock = 12,
        WarningTriangle = 13,
        Unknown = 100,
        RoadWork_occupy_0501 = 501,
        RoadWork_break_0502 = 502
    };

    std::map<perception::fusion::ObjectType, ClassID> typeMap = {
        {perception::fusion::ObjectType::PEDESTRIAN, ClassID::Person},
        {perception::fusion::ObjectType::BICYCLE, ClassID::Bicycle},
        {perception::fusion::ObjectType::CAR, ClassID::Car},
        {perception::fusion::ObjectType::TRUCK, ClassID::Truck},
        {perception::fusion::ObjectType::BUS, ClassID::Bus}
        // add obu type to fusion_type
        // {perception::TYPE_ROADWORK_OCCUPY_0501, ClassID::RoadWork_occupy_0501},
        // {perception::TYPE_ROADWORK_BREAK_0502, ClassID::RoadWork_break_0502},
    };

    std::map<int, mogo::telematics::pad::AdditionalAttribute> event_map = {
        {0, mogo::telematics::pad::AdditionalAttribute::ATTR_UNKNOWN},
        {1, mogo::telematics::pad::AdditionalAttribute::ATTR_ZOMBIE},
        {2, mogo::telematics::pad::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION},
        {3, mogo::telematics::pad::AdditionalAttribute::ATTR_STATIC},
        {4, mogo::telematics::pad::AdditionalAttribute::ATTR_ACCIDENT}};

   private:
    EventProcessor event_processor_;
    static double overlap_iou_thresh_;
    static double center_distance_thresh_;
};
}  // namespace fusion
}  // namespace perception