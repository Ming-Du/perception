#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <common/proto/convexhull.pb.h>
#include <common/proto/localization.pb.h>
#include <common/proto/message_pad.pb.h>
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/object.pb.h"
#include "perception/base/object.h"
#include "perception/base/timer.h"
#include "common/fusion_define.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace perception {
namespace fusion {

const int object_filter_miss_time_threshold =
    3;  // object missing frames threshold to delete in the map : object_id_2_miss_time_
class ObjectFilter {
public:
  ObjectFilter(){}
  ~ObjectFilter(){}
  void getConvexHullMarkerArray(const hadmap::MapConvexhull &map_convexhulls,
                                visualization_msgs::MarkerArray &marker_array);
  void OutMapObjectsFilter(TrackedObjects &tracked_objects,
                           const hadmap::MapConvexhull &map_convexhulls,
                           int &pub_obj_num);
  void OutRoiObjectsFilter(TrackedObjects &tracked_objects,
                                         TrackedObjects &tracked_objects_app);
  // Modify-guoxiaoxiao
  bool IsRadarObjInMap(const RadarObject &radar_object,
                       const hadmap::MapConvexhull &map_convexhulls);
  void ObjectsToApp(
      TrackedObjects &tracked_objects,
      mogo::telematics::pad::TrackedObjects &tracked_objects_filtered,
      double ts_fus);

  void ObjectsToApp(TrackedObjects &tracked_objects,
                    mogo::telematics::pad::TrackedObjects &tracked_objects_filtered, double ts_fus,
                    int number);

 private:
  bool IsInConvexhull(const double input_x,
                      const double input_y,
                      const hadmap::MapConvexhull& map_convexhulls);
  int pnpoly(const geometry::Polygon convexhull, const double object_x, const double object_y);
  bool pointInPolygon(const geometry::Polygon convexhull,
                      const double object_x,
                      const double object_y);
  bool isPointInPolygon(const double& input_x,
                        const double& input_y,
                        const hadmap::MapConvexhull& map_convexhulls);
  float convertAngle(float angle, double lon, double lat);
  double DegToRad(double deg);
  double RadToDeg(double rad);
  std::string DecIntToHexStr(long long num);
  std::string DecStrToHexStr(std::string str);

 private:
  //object ids in roi already checked to missing times - syf
  std::map<uint32_t, int> object_id_2_miss_time_;
};

}  // namespace fusion
}  // namespace perception
