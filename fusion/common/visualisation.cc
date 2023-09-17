#include "visualisation.h"

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "global_util.h"

namespace perception {
namespace fusion {

bool Visualization::TextDisplay(localization::Localization localization, TrackedObjects& objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  // marker.lifetime = ros::Duration(0.2);
  marker.header.frame_id = "base_link";
  marker.pose.orientation.w = 1.;
  marker.color.a = 1.f;

  static int last_marker_size = 0;

  int32_t mk_id = 0;
  for (int32_t m = 0; m < objects.objs_size(); ++m) {
    TrackedObject* object = objects.mutable_objs(m);
    auto& obj = object->obj();

    // add text info
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.header.stamp.fromSec(obj.time_stamp());  // syf
    marker.text = "<" + std::to_string(obj.id()) + ">";
    marker.scale.x = 0.;
    marker.scale.y = 0.;
    marker.scale.z = 1.0;
    marker.color.r = 1.f;
    marker.color.g = 0.f;
    marker.color.b = 1.f;
    marker.points.clear();

    const auto& center = obj.center();
    // marker.pose.position.x = obj.x_distance();
    // marker.pose.position.y = obj.y_distance();
    //    std::cout << "hello test" << std::endl;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    // add object velocity
    // Modify-syf
    std::string abs_lon_v = NumToStr<float>(object->velocity().x() * 3.6, 1);
    std::string abs_lat_v = NumToStr<float>(object->velocity().y() * 3.6, 1);
    // marker.text += "\nabs_vel:" + abs_lon_v + ", " + abs_lat_v  + " km/h";
    // Modify(@liuxinyu):
    marker.text += abs_lon_v + "km/h";

    // Modify-syf
    std::string rel_vel_x = NumToStr<float>(obj.velocity().x() * 3.6, 1);
    std::string rel_vel_y = NumToStr<float>(obj.velocity().y() * 3.6, 1);
    // marker.text += "\nrel_vel:" + rel_vel_x + ", " + rel_vel_y  + " km/h";

    AddTypeText(obj, marker);
    AddFusionMark(obj, marker);

    // add object center
    //  marker.text += "\npos_x:" + NumToStr<float>(center.x(), 2)
    //    + ", pos_y:" + NumToStr<float>(center.y(), 2);

    // marker.text += "\nsize_x:" + NumToStr<float>(obj.size().x(), 2)
    //   + ", size_y:" + NumToStr<float>(obj.size().y(), 2)
    //   + ", size_z:" + NumToStr<float>(obj.size().z(), 2);

    marker.pose.position.z = -.6f;
    marker.id = mk_id++;
    marker_array.markers.emplace_back(marker);
  }
  // ego marker
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  double timestamp_loc =
      localization.header().stamp().sec() + localization.header().stamp().nsec() * 1e-9;
  marker.header.stamp.fromSec(timestamp_loc);  // syf

  marker.text = "EGO";
  marker.scale.x = 0.;
  marker.scale.y = 0.;
  marker.scale.z = 1.;
  marker.color.r = 1.f;
  marker.color.g = 0.f;
  marker.color.b = 1.f;
  marker.points.clear();

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.;

  // add ego velocity
  // Modify-syf
  // marker.text += "-vel:" +
  // NumToStr<double>(localization.longitudinal_v()*3.6, 1)
  // + ", " + NumToStr<double>(localization.lateral_v()*3.6, 1) + " km/h";
  // // marker.text += "\nlon_v:" +
  // std::to_string(localization.longitudinal_v())
  // // + ", lat_v:" + std::to_string(localization.lateral_v());

  marker.text += "<" + NumToStr<double>(localization.longitudinal_v() * 3.6, 1) + "km/h";

  marker.pose.position.z = -.6f;
  marker.id = mk_id++;
  marker_array.markers.emplace_back(marker);

  while (mk_id < last_marker_size) {
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  last_marker_size = marker_array.markers.size();
  return true;
}  // end of TextDisplay()

// Modify @jiangnan: output polygon to rviz
bool Visualization::PolygonDisplay(localization::Localization localization,
                                   TrackedObjects& tracked_objects,
                                   visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker tmp_marker;
  tmp_marker.ns = "polygon";
  tmp_marker.id = 0;
  tmp_marker.header.frame_id = "base_link";
  tmp_marker.type = visualization_msgs::Marker::LINE_LIST;
  tmp_marker.action = visualization_msgs::Marker::ADD;
  tmp_marker.scale.x = 0.05;
  tmp_marker.scale.y = 0.05;
  tmp_marker.scale.z = 0.05;
  tmp_marker.color.r = 1.0f;
  tmp_marker.color.g = 0.0f;
  tmp_marker.color.b = 1.0f;
  tmp_marker.color.a = 1.f;

  int contour_size = 0;
  for (size_t it = 0; it < tracked_objects.objs_size(); ++it) {
    const perception::Object& object = tracked_objects.objs(it).obj();
    contour_size += object.contour_size();
  }
  tmp_marker.points.reserve(contour_size * 6);
  tmp_marker.colors.reserve(contour_size * 6);
  tmp_marker.points.clear();

  for (size_t it = 0; it < tracked_objects.objs_size(); ++it) {
    const perception::Object& obj = tracked_objects.objs(it).obj();

    std::vector<geometry_msgs::Point> cub_points;

    size_t p_size = obj.contour_size();
    double p_low = obj.center().z() - obj.size().z() * 0.5f;
    double p_high = obj.center().z() + obj.size().z() * 0.5f;
    for (size_t i = 0; i < p_size; ++i) {
      geometry_msgs::Point pts;
      pts.x = obj.contour(i).x();
      pts.y = obj.contour(i).y();
      pts.z = p_low;
      cub_points.emplace_back(pts);
    }

    for (size_t i = 0; i < p_size; ++i) {
      geometry_msgs::Point pts;
      pts.x = obj.contour(i).x();
      pts.y = obj.contour(i).y();
      pts.z = p_high;
      cub_points.emplace_back(pts);
    }

    for (size_t i = 0; i < p_size; ++i) {
      size_t next = i + 1;
      next = next < p_size ? next : 0;

      tmp_marker.points.emplace_back(cub_points[i]);
      tmp_marker.points.emplace_back(cub_points[next]);
    }
    for (size_t i = 0; i < p_size; ++i) {
      size_t next = i + 1;
      next = next < p_size ? next : 0;

      tmp_marker.points.emplace_back(cub_points[i + p_size]);
      tmp_marker.points.emplace_back(cub_points[next + p_size]);
    }

    for (size_t i = 0; i < p_size; ++i) {
      tmp_marker.points.emplace_back(cub_points[i]);
      tmp_marker.points.emplace_back(cub_points[i + p_size]);
    }
  }
  marker_array.markers.emplace_back(tmp_marker);
  return true;
}
// Modify @ jiangnan

bool Visualization::BboxDisplay(TrackedObjects& tracked_objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  // marker.lifetime = ros::Duration(0.2);
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "base_link";
  marker.pose.orientation.w = 1.;

  marker.scale.y = 0.;
  marker.color.a = 1.f;

  static size_t last_marker_size = 0;
  int mk_id = 0;
  for (size_t it = 0; it < tracked_objects.objs_size(); ++it) {
    const perception::Object& object = tracked_objects.objs(it).obj();

    // text marker for ID
    mk_id++;
    marker.id = mk_id;

    marker.header.stamp.fromSec(object.time_stamp());  // syf

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    std::string text = std::to_string(object.id());

    marker.text = text;
    marker.scale.x = 0.;
    //   marker.scale.y = 1.;
    marker.scale.z = 0.5;
    marker.color.r = 1.f;
    marker.color.g = 1.f;
    marker.color.b = 1.f;
    marker.color.a = 1.0;
    marker.pose.position.x = object.center().x() - object.size().x() / 2;
    if (object.center().y() >= 0) {
      marker.pose.position.y =
          object.center().y() - object.size().y() / 2 + marker.scale.z / 2 + 0.1;
    } else {
      marker.pose.position.y =
          object.center().y() + object.size().y() / 2 - marker.scale.z / 2 - 0.1;
    }
    marker.pose.position.z = .3f;
    marker.points.clear();
    marker_array.markers.emplace_back(marker);

    // draw frame
    mk_id++;
    marker.id = mk_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.text.clear();

    {
      marker.pose.position = geometry_msgs::Point();
      marker.pose.orientation.w = 1.;
      marker.scale.x = 0.05;
      marker.scale.z = 0.;
    }

    AddBboxPoint(object, marker);

    // wireframe
    {
      double x = object.center().x();
      double y = object.center().y();
      double z = object.center().z();

      double x_size = object.size().x();
      double y_size = object.size().y();
      double z_size = object.size().z();
      // marker.scale.z = 3.2;
      double distance_point = sqrt(x * x + y * y + z * z);
      double distance = 0.0;
      distance = distance_point - 1;
      distance -= (double)x_size / 2.0;

      marker.color.a = 1.0;

      if (distance >= 20.0) {
        // printf("distance=%.5lf\n", distance);
        marker.color.r = 3.0 / 255.0;
        marker.color.g = 204.0 / 255.0;
        marker.color.b = 238.0 / 255.0;
        if (abs(y) < 5.0) {
          if (distance_point - 1 - (double)x_size / 2.0 < 5.0) {
            marker.color.r = (double)255.0 / 255.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
          }
        }
      } else {
        // printf("distance=%.5lf\n", distance);
        marker.color.r = 0;
        marker.color.g = 0.9372549;
        marker.color.b = 0.97254902;
        if (abs(y) < 6.0 && x + x_size / 2.0 >= -1.0) {
          marker.color.r = (double)250.0 / 255.0;
          marker.color.g = (double)246.0 / 255.0;
          marker.color.b = 0.0;
          if (abs(y) < 2.0) {
            marker.color.r = (double)255.0 / 255.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
          }
        }
      }
    }  // end of wireframe

    if (object.camera_supplement().on_use() && (!object.lidar_supplement().on_use())) {
      marker.color.r = (double)255.0 / 255.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.scale.x = 0.2;
    }

    if (object.type() == TYPE_UNKNOWN_STATIC) {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      continue;
    }
    // hzm add
    if (object.status() == 2) {
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
    }
    if (marker.pose.orientation.w == 0 && marker.pose.orientation.x == 0 &&
        marker.pose.orientation.y == 0 && marker.pose.orientation.z == 0)
      marker.pose.orientation.w = 1.;
    marker_array.markers.emplace_back(marker);

    // add arrow of obstacle direction
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "base_link";
    arrow_marker.header.stamp.fromSec(object.time_stamp());
    arrow_marker.ns = "rviz";
    mk_id++;
    arrow_marker.id = mk_id;
    arrow_marker.lifetime = ros::Duration(0.2);
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.scale.x = 2.0;
    arrow_marker.scale.y = 0.3;
    arrow_marker.scale.z = 0.3;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.;
    arrow_marker.pose.position.x = object.center().x();
    arrow_marker.pose.position.y = object.center().y();
    arrow_marker.pose.position.z = object.size().z() / 2;

    Eigen::Quaterniond quat = RPYToQuaterniond(0.0, 0.0, static_cast<float>(object.angle()));
    arrow_marker.pose.orientation.w = quat.w();
    arrow_marker.pose.orientation.x = quat.x();
    arrow_marker.pose.orientation.y = quat.y();
    arrow_marker.pose.orientation.z = quat.z();
    marker_array.markers.emplace_back(arrow_marker);
  }  // end of for

  while (mk_id < last_marker_size) {
    marker.id = ++mk_id;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  last_marker_size = marker_array.markers.size();
  return true;
}  // end of BboxDisplay()

// Debug-guoxiaoxiao
void Visualization::RadarObjDisplay(const fusion::FrameConstPtr& radar_frame,
                                    ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;

  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(radar_frame->timestamp);
  text_marker.ns = "conti_radar_points";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(radar_frame->timestamp);
  point_marker.ns = "conti_radar_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::SPHERE;

  text_marker.scale.z = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  point_marker.scale.x = 1.3f;  // 1.6f;      //2.0f;
  point_marker.scale.y = 1.3f;  // 1.6f;      //2.0f;
  point_marker.scale.z = 1.3f;  // 1.6f;      //2.0f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 1.0f;

  int mk_id = 0;
  static size_t last_marker_size_ = 0;

  for (size_t i = 0; i < radar_frame->objects.size(); ++i) {
    text_marker.points.clear();
    text_marker.text.clear();
    point_marker.points.clear();
    point_marker.text.clear();

    double vel_x = radar_frame->objects[i]->velocity(0) * 3.6;
    double vel_y = radar_frame->objects[i]->velocity(1) * 3.6;
    double abs_velocity = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
    mk_id++;
    text_marker.id = mk_id;
    text_marker.pose.position.x = radar_frame->objects[i]->center_ego(0);
    text_marker.pose.position.y = radar_frame->objects[i]->center_ego(1);
    text_marker.pose.position.z = 0;
    std::string text = "ID: ";

    text += std::to_string(radar_frame->objects[i]->id) + "; ";
    text += "\nx: " + NumToStr<float>(radar_frame->objects[i]->center_ego(0), 2) + "; ";
    text += "y: " + NumToStr<float>(radar_frame->objects[i]->center_ego(1), 2) + "; ";
    // text += "\nvx: " + NumToStr<float>(radar_frame->objects[i]->velocity(0),
    // 2) + "; "; text += "\nvy: " +
    // NumToStr<float>(radar_frame->objects[i]->velocity(1), 2) + "; ";
    // text += "\nvx: " + NumToStr<float>(vel_x_rel, 2) + "; ";
    // text += "vy: " + NumToStr<float>(vel_y_rel, 2) + "; ";
    text += "\nabs_v: " + NumToStr<float>(abs_velocity, 2) + "km/h";
    text_marker.text = text;
    marker_array.markers.emplace_back(text_marker);

    mk_id++;
    point_marker.id = mk_id;
    point_marker.pose.position.x = radar_frame->objects[i]->center_ego(0);
    point_marker.pose.position.y = radar_frame->objects[i]->center_ego(1);
    point_marker.pose.position.z = 0;
    marker_array.markers.emplace_back(point_marker);
  }
  while (mk_id < last_marker_size_) {
    text_marker.id = ++mk_id;
    text_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(text_marker);
  }
  last_marker_size_ = marker_array.markers.size();
  publisher.publish(marker_array);
}

void Visualization::VidarObjDisplay(const fusion::FrameConstPtr& vidar_frame,
                                    ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;
  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(vidar_frame->timestamp);
  text_marker.ns = "vidar_points";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(vidar_frame->timestamp);
  point_marker.ns = "vidar_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  // point_marker.type = visualization_msgs::Marker::SPHERE;
  point_marker.type = visualization_msgs::Marker::LINE_LIST;

  text_marker.scale.z = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  point_marker.scale.x = 1.3f;
  point_marker.scale.y = 1.3f;
  point_marker.scale.z = 1.3f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 1.0f;

  int mk_id = 0;
  static size_t last_marker_size_ = 0;
  for (size_t i = 0; i < vidar_frame->objects.size(); ++i) {
    text_marker.points.clear();
    text_marker.text.clear();
    point_marker.points.clear();
    point_marker.text.clear();

    double vx = vidar_frame->objects[i]->velocity(0);
    double vy = vidar_frame->objects[i]->velocity(1);
    double abs_velocity = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2))*3.6;
    mk_id++;  // if want to show text, delete commit
    text_marker.id = mk_id;
    text_marker.pose.position.x = vidar_frame->objects[i]->center_ego(0);
    text_marker.pose.position.y = vidar_frame->objects[i]->center_ego(1);
    text_marker.pose.position.z = 0;
    std::string text = "ID: ";

    // int to hex and string 后续封装
    std::string out;
    std::stringstream ss;
    ss << vidar_frame->objects[i]->id;
    ss >> out;
    std::transform(out.begin(), out.end(), out.begin(), ::toupper);
    //    std::cout << out << std::endl;
    //    printf("\033[31m pos_x_ego:%lf .\033[0m\n",
    //    vidar_frame->objects[i]->center(0)); printf("\033[31m
    //    pos_y_ego:%lf.\033[0m\n", vidar_frame->objects[i]->center(1));

    text += out + "; ";
    text += "\nx_dis: " + NumToStr<float>(vidar_frame->objects[i]->center_ego(0), 2) + "m; ";
    text += "y_dis: " + NumToStr<float>(vidar_frame->objects[i]->center_ego(1), 2) + "m; ";
    text += "\nabs_velocity: " + NumToStr<float>(abs_velocity, 2) + "km/h; ";
    text_marker.text = text;
    marker_array.markers.emplace_back(text_marker);  // if want to show text, delete commit

    mk_id++;
    point_marker.id = mk_id;

    {
      point_marker.pose.position = geometry_msgs::Point();
      point_marker.pose.orientation.w = 1.;
      point_marker.scale.x = 0.05;
      point_marker.scale.z = 0.;
      perception::Object* obj = new perception::Object;
      obj->set_angle(vidar_frame->objects[i]->theta);
      obj->mutable_size()->set_x(vidar_frame->objects[i]->size(0));
      obj->mutable_size()->set_y(vidar_frame->objects[i]->size(1));
      obj->mutable_size()->set_z(vidar_frame->objects[i]->size(2));
      obj->mutable_center()->set_x(vidar_frame->objects[i]->center_ego(0));
      obj->mutable_center()->set_y(vidar_frame->objects[i]->center_ego(1));
      AddBboxPoint(*obj, point_marker);
    }

    marker_array.markers.emplace_back(point_marker);
  }
  while (mk_id < last_marker_size_) {
    text_marker.id = ++mk_id;
    text_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(text_marker);
  }
  last_marker_size_ = marker_array.markers.size();
  publisher.publish(marker_array);
}

// rad to Quaterniond
Eigen::Quaterniond Visualization::RPYToQuaterniond(const float& roll, const float& pitch,
                                                   const float& yaw) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond _orientation = yawAngle * pitchAngle * rollAngle;
  return _orientation;
}

void Visualization::PointRotate(const double cosYaw, const double sinYaw, const double dcx,
                                const double dcy, geometry_msgs::Point& point) {
  double px = point.x;
  double py = point.y;
  point.x = px * cosYaw - py * sinYaw + dcx;
  point.y = px * sinYaw + py * cosYaw + dcy;
}

void Visualization::AddTypeText(const perception::Object& obj, visualization_msgs::Marker& marker) {
  perception::ObjectType type = obj.type();
  switch (type) {
    case perception::ObjectType::TYPE_PEDESTRIAN:
      marker.text += "PED";
      break;
    case perception::ObjectType::TYPE_BICYCLE:
      marker.text += "BIC";
      break;
    case perception::ObjectType::TYPE_MOTOR:
      marker.text += "MOTOR";
      break;
    case perception::ObjectType::TYPE_RIDER:
      marker.text += "RIDER";
      break;
    case perception::ObjectType::TYPE_CAR:
      marker.text += "CAR";
      break;
    case perception::ObjectType::TYPE_TRUCK:
      marker.text += "TRUCK";
      break;
    case perception::ObjectType::TYPE_BUS:
      marker.text += "BUS";
      break;
    case perception::ObjectType::TYPE_TRAIN:
      marker.text += "TRAIN";
      break;
    case perception::ObjectType::TYPE_UNKNOWN:
      marker.text += "UNKNOWN";
      break;
    case perception::ObjectType::TYPE_UNKNOWN_DYNAMIC:
      marker.text += "UNKNOWN_D";
      break;
    case perception::ObjectType::TYPE_UNKNOWN_STATIC:
      marker.text += "UNKNOWN_S";
      break;
    case perception::ObjectType::TYPE_SIGN:
      marker.text += "SIGN";
      break;
    case perception::ObjectType::TYPE_TRIANGLEROADBLOCK:
      marker.text += "TRAFFICCONE";
      break;
    case perception::ObjectType::TYPE_ROADWORK_OCCUPY_0501:
      marker.text += "ROADWORK_OCCUPY_0501";
      break;
    case perception::ObjectType::TYPE_ROADWORK_BREAK_0502:
      marker.text += "ROADWORK_BREAK_0502";
      break;
    default:
      break;
  }
}

void Visualization::AddFusionMark(const perception::Object& obj,
                                  visualization_msgs::Marker& marker) {
  bool _has_camera = obj.has_camera_supplement();
  bool _has_lidar = obj.has_lidar_supplement();
  bool _has_radar = obj.has_radar_supplement();
  bool _has_falcon_lidar = obj.has_falcon_lidar_supplement();
  bool _has_obu = obj.has_obu_supplement();
  bool _has_vidar = obj.has_vidar_supplement();  // ming.du

  if (!(_has_lidar || _has_camera || _has_radar || _has_obu || _has_vidar || _has_falcon_lidar))
    return;
  marker.text += "-";
  if (_has_lidar) {
    marker.text += "L";
    int match_id_lslidar = obj.match_ids(0);
    std::string tmp = std::to_string(match_id_lslidar);
    marker.text += tmp;
    marker.color.r = 0.f;
    marker.color.g = 1.f;
    marker.color.b = 0.f;  // green
  }
  if (_has_camera) {
    marker.text += "C";
    int match_id_camera = obj.match_ids(1);
    std::string tmp = std::to_string(match_id_camera);
    marker.text += tmp;
    marker.color.r = 0.f;
    marker.color.g = 1.f;
    marker.color.b = 1.f;
  }
  if (_has_radar) {
    marker.text += "R";
    int match_id_radar = obj.match_ids(2);
    std::string tmp = std::to_string(match_id_radar);
    marker.text += tmp;
    marker.color.r = 1.f;
    marker.color.g = 1.f;
    marker.color.b = 0.f;
  }
  if (_has_obu) {
    marker.text += "O";
    int match_id_obu = obj.match_ids(8);
    std::string tmp = std::to_string(match_id_obu);
    marker.text += tmp;
    marker.color.r = 1.f;
    marker.color.g = 0.f;
    marker.color.b = 1.f;  // purple
  }
  if (_has_vidar) {
    marker.text += "V";
    marker.color.r = 1.f;
    marker.color.g = 0.f;
    marker.color.b = 1.f;  // purple
  }
  if (_has_falcon_lidar) {
    marker.text += "F";
    int match_id_falcon = obj.match_ids(3);
    std::string tmp = std::to_string(match_id_falcon);
    marker.text += tmp;
    marker.color.r = 1.f;
    marker.color.g = 0.f;
    marker.color.b = 0.f;  // red
  }
}

void Visualization::AddBboxPoint(const perception::Object& obj,
                                 visualization_msgs::Marker& marker) {
  /* lfd lbd  rfd rbd   lfu lbu  rfu rbu*/
  geometry_msgs::Point lfd;  // lbd
  geometry_msgs::Point lbd;  // rbd
  geometry_msgs::Point rfd;  // lfd
  geometry_msgs::Point rbd;  // rfd
  geometry_msgs::Point lfu;  // lbu
  geometry_msgs::Point lbu;  // rbu
  geometry_msgs::Point rfu;  // lfu
  geometry_msgs::Point rbu;  // rfu

  SetPointLFD(obj, lfd);
  SetPointLBD(obj, lbd);
  SetPointRFD(obj, rfd);
  SetPointRBD(obj, rbd);
  SetPointLFU(obj, lfu);
  SetPointLBU(obj, lbu);
  SetPointRFU(obj, rfu);
  SetPointRBU(obj, rbu);

  double yaw = obj.angle();  // tracked_objects.objs(it).yaw();
  if (abs(yaw) > 0.01) {
    double cx = obj.center().x();
    double cy = obj.center().y();

    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);
    double dcx = -cx * cosYaw + cy * sinYaw + cx;
    double dcy = -cx * sinYaw - cy * cosYaw + cy;
    PointRotate(cosYaw, sinYaw, dcx, dcy, lfd);
    PointRotate(cosYaw, sinYaw, dcx, dcy, lbd);
    PointRotate(cosYaw, sinYaw, dcx, dcy, rfd);
    PointRotate(cosYaw, sinYaw, dcx, dcy, rbd);
    PointRotate(cosYaw, sinYaw, dcx, dcy, lfu);
    PointRotate(cosYaw, sinYaw, dcx, dcy, lbu);
    PointRotate(cosYaw, sinYaw, dcx, dcy, rfu);
    PointRotate(cosYaw, sinYaw, dcx, dcy, rbu);
  }

  {
    marker.points.push_back(lfd);
    marker.points.push_back(lbd);
    marker.points.push_back(rfd);
    marker.points.push_back(rbd);
    marker.points.push_back(lfu);
    marker.points.push_back(lbu);
    marker.points.push_back(rfu);
    marker.points.push_back(rbu);
  }

  {
    marker.points.push_back(lfd);
    marker.points.push_back(lfu);
    marker.points.push_back(rfd);
    marker.points.push_back(rfu);
    marker.points.push_back(lbd);
    marker.points.push_back(lbu);
    marker.points.push_back(rbd);
    marker.points.push_back(rbu);
  }
  {
    marker.points.push_back(lfd);
    marker.points.push_back(rfd);
    marker.points.push_back(lfu);
    marker.points.push_back(rfu);
    marker.points.push_back(lbd);
    marker.points.push_back(rbd);
    marker.points.push_back(lbu);
    marker.points.push_back(rbu);
  }
}

//@lijian,显示结果图像
void Visualization::LidarImageDisplay(const fusion::FramePtr& frame,
                                      const std::vector<fusion::ObjectPtr>& fused_objects,
                                      ros::Publisher& publisher) {
  //@lijian,显示lidar的投影点image坐标下
  if (frame->sensor_info.type() == perception::base::SensorType::SENSING_60) {
    cv::Mat image = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < fused_objects.size(); ++i) {
      if (!fused_objects[i]->centers_image.empty()) {
        cv::Point2d pt_ct = cv::Point2d(fused_objects[i]->centers_image[0][0],
                                        fused_objects[i]->centers_image[0][1]);
        cv::Point2d pt_lu = cv::Point2d(fused_objects[i]->centers_image[1][0],
                                        fused_objects[i]->centers_image[1][1]);
        cv::Point2d pt_rd = cv::Point2d(fused_objects[i]->centers_image[2][0],
                                        fused_objects[i]->centers_image[2][1]);
        std::string strobjid = std::to_string(fused_objects[i]->track_id);
        if(pt_ct.x>1920 || pt_ct.x<0 || pt_ct.y>1080 || pt_ct.y<0)continue;
        if(pt_lu.x>1920 || pt_lu.x<0 || pt_lu.y>1080 || pt_lu.y<0)continue;
        if(pt_rd.x>1920 || pt_rd.x<0 || pt_rd.y>1080 || pt_rd.y<0)continue;
        cv::circle(image, pt_ct, 5, cv::Scalar(0, 0, 255), -1);  //画圆，空心的
        cv::putText(image, strobjid, pt_ct, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 43), 2,
                    8);
        cv::rectangle(image, pt_lu, pt_rd, cv::Scalar(0, 0, 255), 3, 8);
      }
    }
    for (int j = 0; j < frame->objects.size(); ++j) {
      fusion::ObjectPtr apollo_object = frame->objects.at(j);
      std::string sobjid = std::to_string(apollo_object->id);
      cv::Point2d pt1 = cv::Point2d(apollo_object->camera_supplement.box.xmin,
                                    apollo_object->camera_supplement.box.ymin);
      cv::Point2d pt2 = cv::Point2d(apollo_object->camera_supplement.box.xmax,
                                    apollo_object->camera_supplement.box.ymax);
      cv::rectangle(image, pt1, pt2, cv::Scalar(0, 255, 0), 10);
      cv::putText(image, sobjid, pt1, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(244, 23, 43), 2, 8);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher.publish(msg);
  }
}

void Visualization::ObuObjDisplay(const fusion::FrameConstPtr& obu_frame,
                                  ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;
  visualization_msgs::Marker arrow_marker;
  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(obu_frame->timestamp);
  text_marker.ns = "obu_points";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(obu_frame->timestamp);
  point_marker.ns = "obu_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::LINE_LIST;

  arrow_marker.header.frame_id = "base_link";
  arrow_marker.header.stamp.fromSec(obu_frame->timestamp);
  arrow_marker.ns = "rviz_fusion";
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;

  text_marker.scale.z = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  point_marker.scale.x = 1.3f;
  point_marker.scale.y = 1.3f;
  point_marker.scale.z = 1.3f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 1.0f;

  arrow_marker.scale.x = 3.0;
  arrow_marker.scale.y = 0.5;
  arrow_marker.scale.z = 0.5;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 1.0;
  arrow_marker.color.a = 1.0;

  int mk_id = 0;
  static size_t last_marker_size_ = 0;
  for (size_t i = 0; i < obu_frame->objects.size(); ++i) {
    if (obu_frame->objects[i]->source == perception::fusion::ObjectSource::V2N_RSI) {
      continue;
    }
    text_marker.points.clear();
    text_marker.text.clear();
    point_marker.points.clear();
    point_marker.text.clear();
    double vel_x_rel = obu_frame->objects[i]->velocity(0);  // + host_velocity;
    double vel_y_rel = obu_frame->objects[i]->velocity(1);
    mk_id++;  // if want to show text, delete commit
    text_marker.id = mk_id;
    text_marker.pose.position.x = obu_frame->objects[i]->center_ego(0);
    text_marker.pose.position.y = obu_frame->objects[i]->center_ego(1);
    text_marker.pose.position.z = 0;
    std::string text = "ID: ";

    // TODO(@liuxinyu):int to hex and string 后续封装
    std::string out;
    std::stringstream ss;
    ss << std::hex << obu_frame->objects[i]->id;
    ss >> out;
    std::transform(out.begin(), out.end(), out.begin(), ::toupper);
    // printf("\033[31m pos_x_ego:%lf .\033[0m\n",
    //        obu_frame->objects[i]->center_ego(0));
    // printf("\033[31m pos_y_ego:%lf.\033[0m\n",
    //        obu_frame->objects[i]->center_ego(1));

    // text += out + "; ";
    text += "<" + std::to_string(obu_frame->objects[i]->id) + ">";
    text += "\nx: " + NumToStr<float>(obu_frame->objects[i]->center_ego(0), 2) + "; ";
    text += "y: " + NumToStr<float>(obu_frame->objects[i]->center_ego(1), 2) + "; ";
    text += "\nvx: " + NumToStr<float>(vel_x_rel, 2) + "; ";
    text += "vy: " + NumToStr<float>(vel_y_rel, 2) + "; ";
    text_marker.text = text;
    {
      perception::fusion::ObjectType type = obu_frame->objects[i]->type;
      switch (type) {
      case perception::fusion::ObjectType::PEDESTRIAN:
        text_marker.text += "\nPED";
        break;
      case perception::fusion::ObjectType::BICYCLE:
        text_marker.text += "\nBIC";
        break;
      case perception::fusion::ObjectType::MOTOR:
        text_marker.text += "\nMOTOR";
        break;
      case perception::fusion::ObjectType::CAR:
        text_marker.text += "\nCAR";
        break;
      case perception::fusion::ObjectType::TRUCK:
        text_marker.text += "\nTRUCK";
        break;
      case perception::fusion::ObjectType::BUS:
        text_marker.text += "\nBUS";
        break;
      default:
        break;
    }
    }
    marker_array.markers.emplace_back(text_marker);  // if want to show text, delete commit
    // add arrow of obstacle
    {
      arrow_marker.id = mk_id++;
      arrow_marker.pose.position.x = obu_frame->objects[i]->center_ego(0);
      arrow_marker.pose.position.y = obu_frame->objects[i]->center_ego(1);
      arrow_marker.pose.position.z = obu_frame->objects[i]->center_ego(2);
      tf::Quaternion quat = tf::createQuaternionFromYaw(obu_frame->objects[i]->theta);
      tf::quaternionTFToMsg(quat, arrow_marker.pose.orientation);
      marker_array.markers.emplace_back(arrow_marker);
    }

    mk_id++;
    point_marker.id = mk_id;

    // LINE_LIST
    {
      point_marker.pose.position = geometry_msgs::Point();
      point_marker.pose.orientation.w = 1.;
      point_marker.scale.x = 0.05;
      point_marker.scale.z = 0.;
      perception::Object* obj = new perception::Object;
      obj->set_angle(obu_frame->objects[i]->theta);
      obj->mutable_size()->set_x(obu_frame->objects[i]->size(0));
      obj->mutable_size()->set_y(obu_frame->objects[i]->size(1));
      obj->mutable_size()->set_z(obu_frame->objects[i]->size(2));
      obj->mutable_center()->set_x(obu_frame->objects[i]->center_ego(0));
      obj->mutable_center()->set_y(obu_frame->objects[i]->center_ego(1));
      AddBboxPoint(*obj, point_marker);
    }

    marker_array.markers.emplace_back(point_marker);
    point_marker.points.clear();

  }
  while (mk_id < last_marker_size_) {
    if (marker_array.markers.size() == 0) {
      break;
    }
    text_marker.id = ++mk_id;
    text_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(text_marker);
  }
  last_marker_size_ = marker_array.markers.size();

  publisher.publish(marker_array);
}

void Visualization::ObuRTEDisplay(const perception::ObuObjects& obu_objets,
                                  const localization::Localization localization,
                                  ros::Publisher& publisher) {
  double timestamp = obu_objets.header().stamp().sec() + obu_objets.header().stamp().nsec() * 1e-9;
  visualization_msgs::MarkerArray marker_array;
  static int last_marker_size = 0;
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

  int marker_id = 0;
  for (size_t i = 0; i < obu_objets.objs_size(); ++i) {
    
    perception::ObuObject obu_object = obu_objets.objs(i);
    perception::ObjectSource obu_source = obu_object.source();
    if (obu_source != perception::ObjectSource::V2N_RSI) continue;
    // std::cout << "--------" << i << std::endl;
    line_strip.points.clear();
    int rte_size = obu_object.obj().contour_size();
    for (size_t i = 0; i < rte_size; ++i) {
      geometry_msgs::Point point;
      point.x = obu_object.obj().contour(i).x();
      point.y = obu_object.obj().contour(i).y();
      line_strip.points.push_back(point);
      if (i == rte_size - 1) {
        geometry_msgs::Point point_0;
        point_0.x = obu_object.obj().contour(0).x();
        point_0.y = obu_object.obj().contour(0).y();
        line_strip.points.push_back(point_0);
      }
      point_marker.pose.position.x = point.x;
      point_marker.pose.position.y = point.y;
      // std::cout << "point.x: " << point.x << "point.y: " << point.y << std::endl;
      point_marker.id = marker_id++;
      marker_array.markers.push_back(point_marker);
    }
    line_strip.id = marker_id++;
    marker_array.markers.push_back(line_strip);
  }
  while (marker_id < last_marker_size) {
    if (marker_array.markers.size() == 0) break;
    line_strip.id = marker_id++;
    line_strip.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(line_strip);
  }
  last_marker_size = marker_array.markers.size();

  publisher.publish(marker_array);
}

void Visualization::ObuTmpObjDisplay(const perception::ObuObjects& obu_objets,
                                     ros::Publisher& publisher) {
  double timestamp = obu_objets.header().stamp().sec() + obu_objets.header().stamp().nsec() * 1e-9;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;
  visualization_msgs::Marker arrow_marker;
  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(timestamp);
  text_marker.ns = "obu_points";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(timestamp);
  point_marker.ns = "obu_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::LINE_LIST;

  arrow_marker.header.frame_id = "base_link";
  arrow_marker.header.stamp.fromSec(timestamp);
  arrow_marker.ns = "rviz_fusion";
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;

  text_marker.scale.z = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  point_marker.scale.x = 1.3f;
  point_marker.scale.y = 1.3f;
  point_marker.scale.z = 1.3f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 1.0f;

  arrow_marker.scale.x = 3.0;
  arrow_marker.scale.y = 0.5;
  arrow_marker.scale.z = 0.5;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 1.0;
  arrow_marker.color.a = 1.0;

  int mk_id = 0;
  static size_t last_marker_size = 0;
  // std::cout << "===========================: obu_objets.objs_size() " << obu_objets.objs_size() << std::endl;
  for (size_t i = 0; i < obu_objets.objs_size(); ++i) {
    perception::ObuObject obu_object = obu_objets.objs(i);
    if (obu_object.source() == perception::ObjectSource::V2N_RSI) continue;
    if (obu_object.obj().center().x() == 0 && obu_object.obj().center().y() == 0) continue;

    double vel_x_rel = obu_object.velocity().x();  // + host_velocity;
    double vel_y_rel = obu_object.velocity().y();
    mk_id++;  // if want to show text, delete commit
    text_marker.id = mk_id;
    // std::cout << "====== " << i << std::endl;
    // std::cout << obu_object.obj().center().x() << " " << obu_object.obj().center().y() << std::endl;
    text_marker.pose.position.x = obu_object.obj().center().x();
    text_marker.pose.position.y = obu_object.obj().center().y();
    text_marker.pose.position.z = obu_object.obj().center().z();
    // std::string text = "ID: ";

    // TODO(@liuxinyu):int to hex and string
    std::string out;
    std::stringstream ss;
    ss << std::hex << obu_object.obj().id();
    ss >> out;
    std::transform(out.begin(), out.end(), out.begin(), ::toupper);

    text_marker.text += "<" + std::to_string(obu_object.obj().id()) + "> ";
    AddTypeText(obu_object.obj(), text_marker);
    text_marker.text += "\nx: " + NumToStr<float>(obu_object.obj().center().x(), 2) + "; ";
    text_marker.text += "y: " + NumToStr<float>(obu_object.obj().center().y(), 2) + "; ";
    text_marker.text += "\nvx: " + NumToStr<float>(vel_x_rel, 2) + "; ";
    text_marker.text += "vy: " + NumToStr<float>(vel_y_rel, 2) + "; ";
    marker_array.markers.push_back(text_marker);  // if want to show text, delete commit
    text_marker.text.clear();

    // add arrow of obstacle
    {
      arrow_marker.id = mk_id++;
      arrow_marker.pose.position.x = obu_object.obj().center().x();
      arrow_marker.pose.position.y = obu_object.obj().center().y();
      arrow_marker.pose.position.z = obu_object.obj().center().z();
      tf::Quaternion quat = tf::createQuaternionFromYaw(obu_object.obj().angle());
      tf::quaternionTFToMsg(quat, arrow_marker.pose.orientation);
      marker_array.markers.emplace_back(arrow_marker);
    }

    mk_id++;
    point_marker.id = mk_id;

    // LINE_LIST
    {
      point_marker.pose.position = geometry_msgs::Point();
      point_marker.pose.orientation.w = 1.;
      point_marker.scale.x = 0.05;
      point_marker.scale.z = 0.;
      perception::Object* obj = new perception::Object;
      obj->set_angle(obu_object.obj().angle());
      obj->mutable_size()->set_x(obu_object.obj().size().x());
      obj->mutable_size()->set_y(obu_object.obj().size().y());
      obj->mutable_size()->set_z(obu_object.obj().size().z());
      obj->mutable_center()->set_x(obu_object.obj().center().x());
      obj->mutable_center()->set_y(obu_object.obj().center().y());
      obj->mutable_center()->set_z(obu_object.obj().center().z());
      AddBboxPoint(*obj, point_marker);
    }
    marker_array.markers.push_back(point_marker);
    point_marker.points.clear();
  }
  while (mk_id < last_marker_size) {
    if (marker_array.markers.size() == 0) break;
    point_marker.id = mk_id++;
    point_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(point_marker);
  }
  last_marker_size = marker_array.markers.size();
  publisher.publish(marker_array);
}

void Visualization::LaneImageDisplay(cv::Mat image,
                                     const std::vector<std::vector<cv::Point2d>>& lanes,
                                     ros::Publisher& publisher,
                                     bool pub) {

  if (lanes.empty()) return;
  // 判断是否在图片范围内并画线
  int H = image.rows;
  int W = image.cols;

  auto drawPoints = [H, W, &image](const std::vector<cv::Point2d>& lane) {
    for (const auto& p : lane) {
      if(p.x>W || p.x<0 || p.y>H || p.y<0) {
        continue;
      }
      cv::circle(image, p, 5, cv::Scalar(0, 0, 255), -1);  //画圆，空心的
    }
  };

  for(const auto& points : lanes) {
    drawPoints(points);
  }

  if (pub) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher.publish(msg);
  }
}

void Visualization::LineImageDisplay(cv::Mat image,
                                       const std::vector<std::pair<double, std::vector<cv::Point2d>>>& lines,
  ros::Publisher& publisher,
  bool pub) {

  // if (lines.empty()) return;
  // 判断是否在图片范围内并画线
  int H = image.rows;
  int W = image.cols;

  auto drawLine = [H, W, &image](const std::pair<double, std::vector<cv::Point2d>>& line) {
    if (line.second.size() != 2) {
      return;
    }
    for (const auto& p : line.second) {
      if(p.x>W || p.x<0 || p.y>H || p.y<0)return;
    }
    cv::line(image, line.second[0], line.second[1], cv::Scalar(0, 2500, 0), 2);
    std::string baselink_x = std::to_string(line.first).substr(0, 4);
    cv::putText(image, baselink_x, line.second[0], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 43), 2, 8);
  };

  for(const auto& line : lines) {
    drawLine(line);
  }

  if (pub) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher.publish(msg);
  }

}

void Visualization::PointsImageDisplay(cv::Mat image,
                                       const std::vector<std::vector<double>>& points,
                                       ros::Publisher& publisher,
                                       bool pub) {

  // if (points.empty()) return;
  // 判断是否在图片范围内并画线
  int H = image.rows;
  int W = image.cols;

  auto drawPoint = [H, W, &image](const std::vector<double>& point) {
    if(point[0]>W || point[0]<0 || point[1]>H || point[1]<0)return;

    std::string baselink_x = std::to_string(point[2]).substr(0, 4) + "," + std::to_string(point[3]).substr(0, 4);
    cv::putText(image, baselink_x, cv::Point2d(point[0], point[1]), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 43), 2, 8);
  };

  for(const auto& point : points) {
    drawPoint(point);
  }

  if (pub) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher.publish(msg);
  }

}

void Visualization::Camera3DDisplay(const std::vector<std::vector<double>>& objs,
                                    ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker point_marker;
  visualization_msgs::Marker label_marker;

  point_marker.header.frame_id = "base_link";
  point_marker.ns = "polygon";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::LINE_STRIP;

  point_marker.scale.x = 0.3f;
  point_marker.scale.y = 0.3f;
  point_marker.scale.z = 0.3f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;

  label_marker.header.frame_id = "base_link";
  label_marker.ns = "id";
  label_marker.action = visualization_msgs::Marker::ADD;
  label_marker.pose.orientation.w = 1.0;
  label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label_marker.scale.z = 1.0f;
  label_marker.color.a = 1.0f;
  label_marker.color.r = 1.0f;
  label_marker.color.g = 1.0f;
  label_marker.color.b = 1.0f;


  int mk_id = 0;
  static size_t last_marker_size_ = 0;
  double x_min, y_max, z_min;
  for (size_t i = 0; i < objs.size(); ++i) {

    // points marker
    point_marker.points.clear();
    point_marker.id = mk_id++;

    x_min = objs[i][2];
    y_max = objs[i][3];
    z_min = objs[i][4];

    geometry_msgs::Point p1;
    p1.x = x_min;
    p1.y = y_max;
    p1.z = z_min;

    geometry_msgs::Point p2;
    p2.x = x_min;
    p2.y = y_max - objs[i][5];
    p2.z = z_min;

    geometry_msgs::Point p3;
    p3.x = x_min;
    p3.y = y_max - objs[i][5];
    p3.z = z_min + objs[i][6];

    geometry_msgs::Point p4;
    p4.x = x_min;
    p4.y = y_max;
    p4.z = z_min + objs[i][6];
    point_marker.points.push_back(p1);
    point_marker.points.push_back(p2);
    point_marker.points.push_back(p3);
    point_marker.points.push_back(p4);
    point_marker.points.push_back(p1);

    marker_array.markers.emplace_back(point_marker);

    // label text marker
    label_marker.pose.position.x = x_min;
    label_marker.pose.position.y = y_max - objs[i][5]/2.0;
    label_marker.pose.position.z = z_min + objs[i][6] + 0.5;
    label_marker.text = std::to_string(int(objs[i][7]));
    label_marker.id = point_marker.id;
    marker_array.markers.emplace_back(label_marker);

  }
  while (mk_id < last_marker_size_) {
    point_marker.id = mk_id++;
    point_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(point_marker);
    label_marker.id = point_marker.id;
    label_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(label_marker);
  }
  last_marker_size_ = marker_array.markers.size()/2;

  publisher.publish(marker_array);
}

}  // end of namespace fusion
}  // end of namespace perception
