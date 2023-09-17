#include "object_filter.h"

namespace perception {
namespace fusion {
void ObjectFilter::OutMapObjectsFilter(TrackedObjects& tracked_objects,
                                       const hadmap::MapConvexhull& map_convexhulls,
                                       int& pub_obj_num) {
  if (map_convexhulls.convexhull().convexhull_points_size() == 0)
    return;

  Object* object;
  pub_obj_num = 0;
  for (size_t i = 0; i < tracked_objects.objs_size(); ++i) {
    object = tracked_objects.mutable_objs(i)->mutable_obj();

    // check if object id is included in object_id_2_miss_time_ map - syf
    std::map<uint32_t, int>::iterator iter = object_id_2_miss_time_.find(object->id());
    if (iter != object_id_2_miss_time_.end()) {  // already checked in_ROI
      pub_obj_num++;
      iter->second = 0;
      continue;
    }
    // Modify-guoxiaoxiao - coordinate with lidar to test version_0113
    if (isPointInPolygon(object->center().x(), object->center().y(), map_convexhulls)) {
      pub_obj_num++;
      (object_id_2_miss_time_)[object->id()] = 0;  // insert new obstacle in_ROI
      continue;
    } else {
      // object->set_status(2);
      continue;
    }
  }

  for (auto it = object_id_2_miss_time_.begin(); it != object_id_2_miss_time_.end();) {
    if (it->second >= object_filter_miss_time_threshold)
      it = object_id_2_miss_time_.erase(it);
    else {
      it->second++;
      ++it;
    }
  }
}

void ObjectFilter::OutRoiObjectsFilter(TrackedObjects &tracked_objects,
                                       TrackedObjects &tracked_objects_app) {
  for (int i = 0; i < tracked_objects.objs_size(); i++) {
    const perception::TrackedObject &tracked_object = tracked_objects.objs(i);
    const ::perception::Object &object = tracked_object.obj();
    // source
    bool _has_obu = object.has_obu_supplement();
    // if (object.tracking_time() < 5) continue;
    if (!_has_obu) {
      // Modify @jiangnan:: filter roi x[-15, 60] y[-15.0, 15.0]
      if (object.center().x() > 60.0 || object.center().x() < -40.0 ||
          object.center().y() > 10.0 || object.center().y() < -10.0) {
        continue;
      }
    }
    TrackedObject *obj = tracked_objects_app.add_objs();
    obj->CopyFrom(tracked_object);
  }
}

// Modify-guoxiaoxiao
bool ObjectFilter::IsRadarObjInMap(const RadarObject& radar_object,
                                   const hadmap::MapConvexhull& map_convexhulls) {
  if (map_convexhulls.convexhull().convexhull_points_size() == 0)
    return true;  // no map convexhull received, keep the radar object

  if (IsInConvexhull(radar_object.obj().center().x(), radar_object.obj().center().y(),
                     map_convexhulls))
    return true;
  else
    return false;
}

// Modify @jiangnan
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

void ObjectFilter::ObjectsToApp(TrackedObjects& tracked_objects,
                                mogo::telematics::pad::TrackedObjects& tracked_objects_filtered,
                                double ts_fus) {
  static std::map<perception::ObjectType, ClassID> typeMap(
      {{perception::TYPE_PEDESTRIAN, ClassID::Person},
       {perception::TYPE_MOTOR, ClassID::MotorCycle},
       {perception::TYPE_BICYCLE, ClassID::Bicycle},
       {perception::TYPE_CAR, ClassID::Car},
       {perception::TYPE_TRUCK, ClassID::Truck},
       {perception::TYPE_BUS, ClassID::Bus},
       {perception::TYPE_ROADWORK_OCCUPY_0501, ClassID::RoadWork_occupy_0501},
       {perception::TYPE_ROADWORK_BREAK_0502, ClassID::RoadWork_break_0502}});

  static std::map<perception::AdditionalAttribute, mogo::telematics::pad::AdditionalAttribute> event_map(
      {{perception::AdditionalAttribute::ATTR_UNKNOWN,
        mogo::telematics::pad::AdditionalAttribute::ATTR_UNKNOWN},
       {perception::AdditionalAttribute::ATTR_ZOMBIE,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ZOMBIE},
       {perception::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION},
       {perception::AdditionalAttribute::ATTR_STATIC,
        mogo::telematics::pad::AdditionalAttribute::ATTR_STATIC},
       {perception::AdditionalAttribute::ATTR_ACCIDENT,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ACCIDENT}});
  for (int i = 0; i < tracked_objects.objs_size(); i++) {
    const perception::TrackedObject& tracked_object = tracked_objects.objs(i);
    const ::perception::Object& object = tracked_object.obj();
    // source
    bool _has_camera = object.has_camera_supplement();
    bool _has_lidar = object.has_lidar_supplement();
    bool _has_radar = object.has_radar_supplement();
    bool _has_falcon_lidar = object.has_falcon_lidar_supplement();
    bool _has_obu = object.has_obu_supplement();
    bool _has_vidar = object.has_vidar_supplement();


    // Filter out zero distance tracked_object
    if (IS_DOUBLE_ZERO(object.y_distance()) && IS_DOUBLE_ZERO(object.x_distance())) {
      ROS_DEBUG("ObjectsToApp: SendFcwMessage: Filter out zero distance object: %u", object.id());
      continue;
    }

    mogo::telematics::pad::TrackedObject* object_app = tracked_objects_filtered.add_objs();

    object_app->set_uuid(object.id());

    // type
    ClassID classid =
        typeMap.find(object.type()) != typeMap.end() ? typeMap[object.type()] : ClassID::Unknown;
    object_app->set_type((unsigned int)classid);

    object_app->set_longitude(tracked_object.longitude());
    object_app->set_latitude(tracked_object.latitude());
    object_app->set_altitude(tracked_object.alt());

    // Add by jiangnan: center(relative position)
    object_app->mutable_center()->set_x(object.center().x());
    object_app->mutable_center()->set_y(object.center().y());
    object_app->mutable_center()->set_z(object.center().z());

    // heading
    // 0~2*PI:East:0, Counterclockwise => 0~2*PI:North:0, Clockwise
    auto heading = M_PI_2 - tracked_object.yaw();
    if (heading > 2 * M_PI) {
      heading -= 2 * M_PI;
    } else if (heading < 0) {
      heading += 2 * M_PI;
    }
    heading *= RAD_TO_DEG;
    float yaw_converted =
        convertAngle(heading, tracked_object.longitude(), tracked_object.latitude());
    object_app->set_heading(yaw_converted);

    // speed
    double speed = std::sqrt(std::pow(tracked_object.velocity().x(), 2) +
                             std::pow(tracked_object.velocity().y(), 2));
    object_app->set_speed(speed);

    // time
    object_app->set_systemtime(ros::Time::now().toSec());
    object_app->set_satellitetime(ts_fus);

    // lidar:1 camera:2 radar:3 v2x:4 vidar:5 falcon:6
    //  [default = 0] v2v_bsm = 1  v2i_rsm = 2 v2v_ssm = 3 v2n_rsm = 4
    if (_has_obu) {
      mogo::telematics::pad::TrackedSource* main_source = object_app->add_tracked_source();
      mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
      main_source->set_source(2);
      int tmp = tracked_object.source();
      sub_source->set_source(tmp);
      if (tracked_object.source() == perception::ObjectSource::V2V_BSM) {
        std::string srt_hex =
            DecStrToHexStr(std::to_string(object.obu_supplement().measurement_id()));
        sub_source->set_id(srt_hex);
      } else if (tracked_object.source() == perception::ObjectSource::V2N_RSI) {
        // TODO(@liuxinyu): 待验证
        int rte_size = tracked_object.polygon_p_size();
        for (size_t i = 0; i < rte_size; ++i) {
          geometry::Point rte_polygon = tracked_object.polygon_p(i);
          mogo::telematics::pad::Location* polygon_point_pad = object_app->add_polygon();
          polygon_point_pad->set_longitude(rte_polygon.x());
          polygon_point_pad->set_latitude(rte_polygon.y());
        }
      }
      ROS_DEBUG_STREAM("ObjectsToApp: obu_sub_source: "
                      << main_source->source() << " sub " << tracked_object.source() << " "
                      << sub_source->source() << " id " << sub_source->id());
    }
    if (_has_lidar || _has_camera || _has_radar || _has_vidar || _has_falcon_lidar) {
      mogo::telematics::pad::TrackedSource* main_source = object_app->add_tracked_source();
      mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
      main_source->set_source(1);
      if (_has_lidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(1);
      }
      if (_has_camera) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(2);
      }
      if (_has_radar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(3);
      }
      if (_has_vidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(4);
      }
      if (_has_falcon_lidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(5);
      }
    }
    object_app->set_add_attribute(event_map.at(tracked_object.obj().add_attribute()));
  }
}

void ObjectFilter::ObjectsToApp(TrackedObjects& tracked_objects,
                                mogo::telematics::pad::TrackedObjects& tracked_objects_filtered,
                                double ts_fus, int number) {
  static std::map<perception::ObjectType, ClassID> typeMap(
      {{perception::TYPE_PEDESTRIAN, ClassID::Person},
       {perception::TYPE_MOTOR, ClassID::MotorCycle},
       {perception::TYPE_BICYCLE, ClassID::Bicycle},
       {perception::TYPE_CAR, ClassID::Car},
       {perception::TYPE_TRUCK, ClassID::Truck},
       {perception::TYPE_BUS, ClassID::Bus},
       {perception::TYPE_ROADWORK_OCCUPY_0501, ClassID::RoadWork_occupy_0501},
       {perception::TYPE_ROADWORK_BREAK_0502, ClassID::RoadWork_break_0502}});

  static std::map<perception::AdditionalAttribute, mogo::telematics::pad::AdditionalAttribute> event_map(
      {{perception::AdditionalAttribute::ATTR_UNKNOWN,
        mogo::telematics::pad::AdditionalAttribute::ATTR_UNKNOWN},
       {perception::AdditionalAttribute::ATTR_ZOMBIE,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ZOMBIE},
       {perception::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ROAD_CONSTRUCTION},
       {perception::AdditionalAttribute::ATTR_STATIC,
        mogo::telematics::pad::AdditionalAttribute::ATTR_STATIC},
       {perception::AdditionalAttribute::ATTR_ACCIDENT,
        mogo::telematics::pad::AdditionalAttribute::ATTR_ACCIDENT}});
  for (int i = number; i < tracked_objects.objs_size(); i++) {
    const perception::TrackedObject& tracked_object = tracked_objects.objs(i);
    const ::perception::Object& object = tracked_object.obj();
    // source
    bool _has_camera = object.has_camera_supplement();
    bool _has_lidar = object.has_lidar_supplement();
    bool _has_radar = object.has_radar_supplement();
    bool _has_falcon_lidar = object.has_falcon_lidar_supplement();
    bool _has_obu = object.has_obu_supplement();
    bool _has_vidar = object.has_vidar_supplement();


    // Filter out zero distance tracked_object
    if (IS_DOUBLE_ZERO(object.y_distance()) && IS_DOUBLE_ZERO(object.x_distance())) {
      ROS_DEBUG("ObjectsToApp: SendFcwMessage: Filter out zero distance object: %u", object.id());
      continue;
    }

    mogo::telematics::pad::TrackedObject* object_app = tracked_objects_filtered.add_objs();

    object_app->set_uuid(object.id());

    // type
    ClassID classid =
        typeMap.find(object.type()) != typeMap.end() ? typeMap[object.type()] : ClassID::Unknown;
    object_app->set_type((unsigned int)classid);

    object_app->set_longitude(tracked_object.longitude());
    object_app->set_latitude(tracked_object.latitude());
    object_app->set_altitude(tracked_object.alt());

    // Add by jiangnan: center(relative position)
    object_app->mutable_center()->set_x(object.center().x());
    object_app->mutable_center()->set_y(object.center().y());
    object_app->mutable_center()->set_z(object.center().z());

    // heading
    // 0~2*PI:East:0, Counterclockwise => 0~2*PI:North:0, Clockwise
    auto heading = M_PI_2 - tracked_object.yaw();
    if (heading > 2 * M_PI) {
      heading -= 2 * M_PI;
    } else if (heading < 0) {
      heading += 2 * M_PI;
    }
    heading *= RAD_TO_DEG;
    float yaw_converted =
        convertAngle(heading, tracked_object.longitude(), tracked_object.latitude());
    object_app->set_heading(yaw_converted);

    // speed
    double speed = std::sqrt(std::pow(tracked_object.velocity().x(), 2) +
                             std::pow(tracked_object.velocity().y(), 2));
    object_app->set_speed(speed);

    // time
    object_app->set_systemtime(ros::Time::now().toSec());
    object_app->set_satellitetime(ts_fus);

    // lidar:1 camera:2 radar:3 v2x:4 vidar:5 falcon:6
    //  [default = 0] v2v_bsm = 1  v2i_rsm = 2 v2v_ssm = 3 v2n_rsm = 4
    if (_has_obu) {
      mogo::telematics::pad::TrackedSource* main_source = object_app->add_tracked_source();
      mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
      main_source->set_source(2);
      int tmp = tracked_object.source();
      sub_source->set_source(tmp);
      if (tracked_object.source() == perception::ObjectSource::V2V_BSM) {
        std::string srt_hex =
            DecStrToHexStr(std::to_string(object.obu_supplement().measurement_id()));
        sub_source->set_id(srt_hex);
      } else if (tracked_object.source() == perception::ObjectSource::V2N_RSI) {
        // TODO(@liuxinyu): 待验证
        int rte_size = tracked_object.polygon_p_size();
        for (size_t i = 0; i < rte_size; ++i) {
          geometry::Point rte_polygon = tracked_object.polygon_p(i);
          mogo::telematics::pad::Location* polygon_point_pad = object_app->add_polygon();
          polygon_point_pad->set_longitude(rte_polygon.x());
          polygon_point_pad->set_latitude(rte_polygon.y());
        }
      }
      ROS_DEBUG_STREAM("ObjectsToApp: obu_sub_source: "
                      << main_source->source() << " sub " << tracked_object.source() << " "
                      << sub_source->source() << " id " << sub_source->id());
    }
    if (_has_lidar || _has_camera || _has_radar || _has_vidar || _has_falcon_lidar) {
      mogo::telematics::pad::TrackedSource* main_source = object_app->add_tracked_source();
      mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
      main_source->set_source(1);
      if (_has_lidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(1);
      }
      if (_has_camera) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(2);
      }
      if (_has_radar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(3);
      }
      if (_has_vidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(4);
      }
      if (_has_falcon_lidar) {
        mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
        sub_source->set_source(5);
      }
    }
    object_app->set_add_attribute(event_map.at(tracked_object.obj().add_attribute()));
  }
}

std::string ObjectFilter::DecIntToHexStr(long long num) {
  std::string str;
  long long Temp = num / 16;
  int left = num % 16;
  if (Temp > 0)
    str += DecIntToHexStr(Temp);
  if (left < 10)
    str += (left + '0');
  else
    str += ('A' + left - 10);
  return str;
}

std::string ObjectFilter::DecStrToHexStr(std::string str) {
  long long Dec = 0;
  for (int i = 0; i < str.size(); ++i)
    Dec = Dec * 10 + str[i] - '0';  //得到相应的整数：ASCII码中：字符0的码值是48；
  return DecIntToHexStr(Dec);
}

void ObjectFilter::getConvexHullMarkerArray(const hadmap::MapConvexhull& map_convexhulls,
                                            visualization_msgs::MarkerArray& marker_array) {
  static int last_marker_size = 0;
  double timestamp =
      map_convexhulls.header().stamp().sec() + map_convexhulls.header().stamp().nsec() * 1e-9;

  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(timestamp);
  point_marker.action = visualization_msgs::Marker::ADD;
  // point_marker.lifetime = ros::Duration(0.1);
  point_marker.ns = "";
  point_marker.type = visualization_msgs::Marker::SPHERE;
  point_marker.scale.x = 1.0;
  point_marker.scale.y = 1.0;
  point_marker.scale.z = 1.0;
  point_marker.color.r = 1.0f;
  point_marker.color.a = 1.0;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "base_link";
  line_strip.header.stamp.fromSec(timestamp);
  line_strip.ns = "";
  line_strip.action = visualization_msgs::Marker::ADD;
  // line_strip.lifetime = ros::Duration(0.1);
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.3;
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;
  line_strip.pose.orientation.w = 1.0;

  int convexhull_size = map_convexhulls.convexhull().convexhull_points_size();
  int marker_id = 0;
  geometry::Polygon convexhull;
  for (size_t i = 0; i < convexhull_size; ++i) {
    convexhull = map_convexhulls.convexhull().convexhull_points(i);
    line_strip.points.clear();
    point_marker.points.clear();

    for (size_t j = 0; j < convexhull.points_size(); ++j) {
      geometry_msgs::Point point;
      point.x = convexhull.points(j).x();
      point.y = convexhull.points(j).y();
      line_strip.points.push_back(point);
      if (j == convexhull.points_size() - 1) {
        geometry_msgs::Point point_;
        point_.x = convexhull.points(0).x();
        point_.y = convexhull.points(0).y();
        line_strip.points.push_back(point_);
      }
      point_marker.pose.position.x = point.x;
      point_marker.pose.position.y = point.y;
      point_marker.id = marker_id++;
      marker_array.markers.push_back(point_marker);
    }
    line_strip.id = marker_id++;
    marker_array.markers.push_back(line_strip);
  }
  while (marker_id < last_marker_size) {
    line_strip.id = marker_id++;
    line_strip.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(line_strip);
  }
  last_marker_size = marker_array.markers.size();
}

bool ObjectFilter::IsInConvexhull(const double input_x,
                                  const double input_y,
                                  const hadmap::MapConvexhull& map_convexhulls) {
  for (int i = 0; i < map_convexhulls.convexhull().convexhull_points_size(); ++i) {
    if (pointInPolygon(map_convexhulls.convexhull().convexhull_points(i), input_x, input_y))
      return true;
    else
      continue;
  }
  return false;
}

int ObjectFilter::pnpoly(const geometry::Polygon convexhull,
                         const double object_x,
                         const double object_y) {
  size_t i, j, result = 0;
  geometry_msgs::Point point1, point2;
  for (i = 0, j = convexhull.points_size() - 1; i < convexhull.points_size(); j = i++) {
    point1.x = convexhull.points(i).x();
    point1.y = convexhull.points(i).y();
    point2.x = convexhull.points(j).x();
    point2.y = convexhull.points(j).y();
    if (((point1.y > object_y) != (point2.y > object_y)) &&
        (object_x <
         (point2.x - point1.x) * (object_y - point1.y) / (point2.y - point1.y) + point1.x))
      result = !result;
  }
  return result;
}

bool ObjectFilter::pointInPolygon(const geometry::Polygon convexhull,
                                  const double object_x,
                                  const double object_y) {
  size_t i, j;
  bool oddNodes = false;
  geometry_msgs::Point point1, point2;
  for (i = 0, j = convexhull.points_size() - 1; i < convexhull.points_size(); j = i++) {
    point1.x = convexhull.points(i).x();
    point1.y = convexhull.points(i).y();
    point2.x = convexhull.points(j).x();
    point2.y = convexhull.points(j).y();
    if ((point1.y < object_y && point2.y >= object_y ||
         point2.y < object_y && point1.y >= object_y) &&
        (point1.x <= object_x || point2.x <= object_x)) {
      oddNodes ^=
          (point1.x + (object_y - point1.y) / (point2.y - point1.y) * (point2.x - point1.x) <
           object_x);
    }
  }
  return oddNodes;
}

bool ObjectFilter::isPointInPolygon(const double& input_x,
                                    const double& input_y,
                                    const hadmap::MapConvexhull& map_convexhulls) {
  for (size_t i = 0; i < map_convexhulls.convexhull().convexhull_points_size(); ++i) {
    int left_cross = 0, right_cross = 0;
    auto& convexhull = map_convexhulls.convexhull().convexhull_points(i);
    for (size_t j = 0; j < convexhull.points_size(); j++) {
      auto& p1 = convexhull.points(j);                                   //当前节点
      auto& p2 = convexhull.points((j + 1) % convexhull.points_size());  //下一个节点

      if (p1.y() == p2.y())  // p1p2 与 y平行，无交点
        continue;

      if (input_y <= std::min(p1.y(), p2.y()))  //线段在上方，无交点
        continue;

      if (input_y > std::max(p1.y(), p2.y()))
        continue;

      // 从P发射一条水平射线 求交点的 X 坐标 ------原理:
      // ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
      //直线k值相等 交点y=p.y
      double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

      if (std::fabs(x - input_x) < 0.00001) {
        return true;
      } else if (x > input_x)
        right_cross++;
      else
        left_cross++;
    }
    // 两边都是单数
    if (right_cross % 2 == 1 && left_cross % 2 == 1) {
      return true;
    }
  }
  return false;
}

float ObjectFilter::convertAngle(float angle, double lon, double lat) {
  double angleOrigin = 450.0 - angle;
  double radianOrigin = DegToRad(angleOrigin);
  double radianLat = DegToRad(lat);
  double radianDeal = atan(tan(radianOrigin) * cos(radianLat));

  double ruler = 270.0;
  if (angle >= 0 && angle <= 180.0) {
    ruler = 90.0;
  } else {
    ruler = 270.0;
  }
  float result = (float)(ruler - RadToDeg(radianDeal));
  return result;
}

double ObjectFilter::DegToRad(double deg) {
  return (deg / 180.0 * M_PI);
}

double ObjectFilter::RadToDeg(double rad) {
  return (rad / M_PI * 180.0);
}

}  // namespace fusion
}  // namespace perception
