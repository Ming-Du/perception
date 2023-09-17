#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
#include <unordered_map>
#include "base/frame.h"
#include "base/object.h"
#include "base/sensor_manager/sensor_manager.h"
#include "common/frame_transform.h"
#include "common/fusion_define.h"
#include "common/gps_proj.h"
#include "lib/event_process/event_processor.h"
#include "perception/base/track_object_types.h"
#include "common/iou.h"
#include "base/object_pool_types.h"
#include "perception/base/proto/perception_component.pb.h"

namespace perception {
namespace fusion {

class BaseConvertor {
   public:
    BaseConvertor(){};
    virtual ~BaseConvertor(){};

    enum input_sensor_ {
      use_lidar = 1,
      use_falcon = 2,
      use_vidar = 3
    };

    virtual bool Mogo2Fusion(const perception::TrackedObject& sensor_object,
                             const localization::Localization& localization,
                             fusion::ObjectPtr& fusion_object,
                             input_sensor_ input_sensor) = 0;

    bool ObjectConvertor(const perception::Object& object, fusion::ObjectPtr& fusion_object);

    void subTypeMogo2ApolloConvertor(const std::string& sensor_name,
                                     const perception::ObjectType& mogo_type,
                                     perception::fusion::ObjectSubType& apollo_subtype);

    void typeApollo2MogoConvertor(const perception::fusion::ObjectType& apollo_type,
                                  const perception::fusion::ObjectSubType& apollo_subtype,
                                  perception::ObjectType& mogo_type);

    float ConvertToAngle(float angle, double lon, double lat);

    double DegToRad(double deg);

    double RadToDeg(double rad);

    std::string DecStrToHexStr(std::string str);

    std::string DecIntToHexStr(long long num);

    std::unordered_map<int, perception::RoadType> RoadType = {{0, perception::RoadType::RoadType_ROAD},
                                                              {1, perception::RoadType::RoadType_ROADSIDE},
                                                              {2, perception::RoadType::RoadType_FLOWERBEDSSIDE},
                                                              {3, perception::RoadType::RoadType_FENCESIDE}};
};
}  // namespace fusion
}  // namespace perception