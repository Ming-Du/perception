#include "visualisation.h"

#include "../../fusion/common/global_util.h"

namespace perception {
namespace calib_check {

bool Visualization::TextDisplay(TrackedObjects& objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
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
    marker.text.clear();
    marker.text = "<" + std::to_string(obj.id()) + ">";
    marker.scale.x = 0.;
    marker.scale.y = 0.;
    marker.scale.z = 1.2;
    marker.color.r = 1.f;
    marker.color.g = 1.f;
    marker.color.b = 1.f;
    marker.color.a = 1.f;

    const auto& center = obj.center();
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z() + 1;

    AddTypeText(obj, marker);
    // AddFusionText(obj, marker);

    // add object absolute velocity
    std::string abs_x = NumToStr<float>(center.x(), 1);
    std::string abs_y = NumToStr<float>(center.y(), 1);
    marker.text += "x:" + abs_x + "y:" + abs_y + "m";
    float range = std::hypot(object->velocity().x(), object->velocity().y());
    std::string speed = NumToStr<float>(range * 3.6, 1);
    marker.text += "\nV:" + speed + "km/h";

    // Modify-syf
    // add object relative velocity
    std::string rel_vel_x = NumToStr<float>(obj.velocity().x() * 3.6, 1);
    std::string rel_vel_y = NumToStr<float>(obj.velocity().y() * 3.6, 1);
    // marker.text += "\nrel_vel:" + rel_vel_x + ", " + rel_vel_y  + " km/h";

    // add object center
    //  marker.text += "\npos_x:" + NumToStr<float>(center.x(), 2)
    //    + ", pos_y:" + NumToStr<float>(center.y(), 2);

    // add object size
    //  marker.text += "\nsize_x:" + NumToStr<float>(obj.size().x(), 2)
    //    + ", size_y:" + NumToStr<float>(obj.size().y(), 2)
    //    + ", size_z:" + NumToStr<float>(obj.size().z(), 2);

    // Modify-guoxiaoxiao
    // linkColorToSensorType(obj, marker);
    // linkColorToObjType(obj, marker);

    marker.id = mk_id++;
    marker_array.markers.emplace_back(marker);
  }

  // ego marker
  //  double timestamp_loc = localization.header().stamp().sec() +
  //                     localization.header().stamp().nsec() * 1e-9;
  //  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  //  // marker.header.stamp.fromSec(timestamp_loc);     //syf
  //  marker.text.clear();
  //  marker.text = "Ego";
  //  marker.scale.x = 0.;
  //  marker.scale.y = 0.;
  //  marker.scale.z = 1.5;
  //  marker.color.r = 1.f;
  //  marker.color.g = 1.f;
  //  marker.color.b = 0.f;
  //  marker.color.a = 0.f;
  //  marker.pose.position.x = 0;
  //  marker.pose.position.y = 0;
  //  marker.pose.position.z = 0;
  //  marker.pose.orientation.w = 1.;

  // // add ego velocity
  // // marker.text += "-V:" + NumToStr<double>(localization.longitudinal_v()*3.6, 1)
  // // + "," + NumToStr<double>(localization.lateral_v()*3.6, 1) + "km/h";

  // marker.id = mk_id++;
  // marker_array.markers.emplace_back(marker);

  // while (mk_id < last_marker_size)
  // {
  //   marker.id = mk_id++;
  //   marker.action = visualization_msgs::Marker::DELETE;
  //   marker_array.markers.emplace_back(marker);
  // }

  // last_marker_size = marker_array.markers.size();
  return true;
}  // end of TextDisplay()

void Visualization::lineMarker(std::unordered_map<std::string, Eigen::MatrixXd> world_points_map,
                               visualization_msgs::MarkerArray& markers) {
  visualization_msgs::Marker line_markers;
  line_markers.header.frame_id = "base_link";
  line_markers.header.stamp = ros::Time();
  line_markers.ns = "fusion_mid";
  line_markers.id = 0;
  line_markers.lifetime = ros::Duration(1.0);
  line_markers.action = visualization_msgs::Marker::ADD;
  // line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.type = visualization_msgs::Marker::LINE_STRIP;

  line_markers.scale.x = 0.5;
  line_markers.scale.y = 0.5;
  line_markers.scale.z = 0.5;
  line_markers.color.r = 0.0;
  line_markers.color.g = 1.0;
  line_markers.color.b = 0.0;
  line_markers.color.a = 1.0;
  int count = 0;
  for (auto it = world_points_map.begin(); it != world_points_map.end(); it++) {
    for (int i = 0; i < 4; i++) {
      geometry_msgs::Point gp;
      gp.x = (*it).second(0, i);
      gp.y = (*it).second(1, i);
      gp.z = (*it).second(2, i);
      line_markers.points.push_back(gp);
    }
    geometry_msgs::Point gp;
    gp.x = (*it).second(0, 0);
    gp.y = (*it).second(1, 0);
    gp.z = (*it).second(2, 0);
    line_markers.points.push_back(gp);
    line_markers.id = count;
    line_markers.pose.orientation.x = 0.0;
    line_markers.pose.orientation.y = 0.0;
    line_markers.pose.orientation.z = 0.0;
    line_markers.pose.orientation.w = 1.0;
    markers.markers.push_back(line_markers);
    line_markers.points.clear();
    for (int i = 0; i < 4; i++) {
      geometry_msgs::Point gp;
      gp.x = (*it).second(0, i);
      gp.y = (*it).second(1, i);
      gp.z = (*it).second(2, i);
      line_markers.points.push_back(gp);
      geometry_msgs::Point gp2;
      gp.x = 0;
      gp.y = 0;
      gp.z = 0;
      line_markers.points.push_back(gp2);
      line_markers.id = (count + 1) * (5 + i);
      line_markers.pose.orientation.x = 0.0;
      line_markers.pose.orientation.y = 0.0;
      line_markers.pose.orientation.z = 0.0;
      line_markers.pose.orientation.w = 1.0;
      markers.markers.push_back(line_markers);
      line_markers.points.clear();
    }
    count++;
  }
}

bool Visualization::BboxDisplay(TrackedObjects& tracked_objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "rviz_fusion_mid";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.;

  static size_t last_marker_size = 0;
  int mk_id = 0;
  for (size_t it = 0; it < tracked_objects.objs_size(); ++it) {
    const perception::Object& object = tracked_objects.objs(it).obj();
    marker.header.stamp.fromSec(object.time_stamp());

    // Modify-guoxiaoxiao - wireframe
    {
      marker.id = mk_id++;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.pose.position = geometry_msgs::Point();
      marker.pose.orientation.w = 1.;
      marker.scale.x = 0.3;
      marker.scale.y = 0.;
      marker.scale.z = 0.;
      marker.points.clear();
      AddBboxPoint(object, marker);
      marker.color.r = 1.f;
      marker.color.g = 0.65f;
      marker.color.b = 0.f;
      marker.color.a = 1.0;
      // Modify-guoxiaoxiao
      // linkColorToSensorType(object, marker);
      linkColorToObjType(object, marker);
      marker_array.markers.emplace_back(marker);
    }  // end of wireframe

    // add arrow of obstacle direction
    {
      visualization_msgs::Marker arrow_marker;
      arrow_marker.header.frame_id = "base_link";
      arrow_marker.header.stamp.fromSec(object.time_stamp());
      arrow_marker.ns = "rviz_fusion_mid";
      arrow_marker.id = mk_id++;
      arrow_marker.type = visualization_msgs::Marker::ARROW;
      arrow_marker.action = visualization_msgs::Marker::ADD;
      arrow_marker.scale.x = 3.0;
      arrow_marker.scale.y = 0.5;
      arrow_marker.scale.z = 0.5;
      arrow_marker.color.r = 1.0;
      arrow_marker.color.g = 0.0;
      arrow_marker.color.b = 0.0;
      arrow_marker.color.a = 1.;
      arrow_marker.pose.position.x = object.center().x();
      arrow_marker.pose.position.y = object.center().y();
      arrow_marker.pose.position.z = object.center().z();

      tf::Quaternion quat = tf::createQuaternionFromYaw(object.angle());
      tf::quaternionTFToMsg(quat, arrow_marker.pose.orientation);
      marker_array.markers.emplace_back(arrow_marker);
    }

    // Modify-guoxiaoxiao
    // add cube of obstacle
    {
      visualization_msgs::Marker cube_marker;
      cube_marker.header.frame_id = "base_link";
      cube_marker.header.stamp.fromSec(object.time_stamp());
      cube_marker.ns = "rviz_fusion_mid";
      cube_marker.id = mk_id++;
      cube_marker.type = visualization_msgs::Marker::CUBE;
      cube_marker.action = visualization_msgs::Marker::ADD;
      cube_marker.scale.x = object.size().x();
      cube_marker.scale.y = object.size().y();
      cube_marker.scale.z = object.size().z();
      cube_marker.color.a = 0.75;
      linkColorToObjType(object, cube_marker);
      cube_marker.pose.position.x = object.center().x();
      cube_marker.pose.position.y = object.center().y();
      cube_marker.pose.position.z = object.center().z();
      tf::Quaternion quat_ = tf::createQuaternionFromYaw(object.angle());
      tf::quaternionTFToMsg(quat_, cube_marker.pose.orientation);
      marker_array.markers.emplace_back(cube_marker);
    }
  }  // end of for

  while (mk_id < last_marker_size) {
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  last_marker_size = marker_array.markers.size();
  return true;
}  // end of BboxDisplay()

// Debug-guoxiaoxiao
void Visualization::RadarObjDisplay(const fusion::FrameConstPtr& radar_frame,
                                    localization::Localization localization,
                                    ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;

  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(radar_frame->timestamp);
  text_marker.ns = "conti_radar";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(radar_frame->timestamp);
  point_marker.ns = "conti_radar";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::SPHERE;

  text_marker.scale.z = 1.1;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  point_marker.scale.x = 1.5f;
  point_marker.scale.y = 1.5f;
  point_marker.scale.z = 1.5f;
  point_marker.color.a = 1.0f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 1.0f;

  int mk_id = 0;
  static size_t last_marker_size_ = 0;

  double host_vx = localization.longitudinal_v();
  double host_vy = localization.lateral_v();

  for (size_t i = 0; i < radar_frame->objects.size(); ++i) {
    double vel_x_rel = radar_frame->objects[i]->velocity(0);
    double vel_y_rel = radar_frame->objects[i]->velocity(1);
    double vel_x_abs = vel_x_rel + host_vx;
    double vel_y_abs = vel_y_rel + host_vy;

    text_marker.id = mk_id++;
    text_marker.pose.position.x = radar_frame->objects[i]->center(0);
    text_marker.pose.position.y = radar_frame->objects[i]->center(1);
    text_marker.pose.position.z = 1.0f;

    text_marker.text.clear();
    std::string text = "<" + std::to_string(radar_frame->objects[i]->id) + ">";
    text += "\nabs_Vx:" + NumToStr<float>(vel_x_abs * 3.6, 1) + ";";
    text += "Vy:" + NumToStr<float>(vel_y_abs * 3.6, 1) + "km/h";
    // text += "\nx:" + NumToStr<float>(radar_frame->objects[i]->center(0), 1) + ";";
    // text += "y:" + NumToStr<float>(radar_frame->objects[i]->center(1), 1);
    // text += "\nvx:" + NumToStr<float>(vel_x_rel*3.6, 1) + ";";
    // text += "vy:" + NumToStr<float>(vel_y_rel*3.6, 1);
    // text += "\nvx:" + NumToStr<float>(vel_x_abs*3.6, 1) + ";";
    // text += "vy:" + NumToStr<float>(vel_y_abs*3.6, 1);
    text_marker.text = text;
    marker_array.markers.emplace_back(text_marker);

    point_marker.id = mk_id++;
    point_marker.pose.position.x = radar_frame->objects[i]->center(0);
    point_marker.pose.position.y = radar_frame->objects[i]->center(1);
    point_marker.pose.position.z = 0;
    marker_array.markers.emplace_back(point_marker);
  }
  while (mk_id < last_marker_size_) {
    text_marker.id = mk_id++;
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

// Modify-guoxiaoxiao
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
    default:
      break;
  }
}

// Modify-guoxiaoxiao
void Visualization::AddFusionText(const perception::Object& obj,
                                  visualization_msgs::Marker& marker) {
  bool has_camera = obj.has_camera_supplement();
  bool has_lidar = obj.has_lidar_supplement();
  bool has_radar = obj.has_radar_supplement();
  // Modify-guoxiaoxiao
  bool has_vidar = obj.has_vidar_supplement();

  // Modify-guoxiaoxiao
  if (has_lidar) {
    if (has_camera) {
      if (has_radar) {
        if (has_vidar) {
          marker.text += "-LCRV";
        } else {
          marker.text += "-LCR";
        }
      } else {  // has no radar
        if (has_vidar) {
          marker.text += "-LCV";
        } else {
          marker.text += "-LC";
        }
      }
    } else {  // has no camera
      if (has_radar) {
        if (has_vidar) {
          marker.text += "-LRV";
        } else {
          marker.text += "-LR";
        }
      } else {
        if (has_vidar) {
          marker.text += "-LV";
        } else {
          marker.text += "-L";
        }
      }
    }
  } else {  // has no lidar
    if (has_camera) {
      if (has_radar) {
        if (has_vidar) {
          marker.text += "-CRV";
        } else {
          marker.text += "-CR";
        }
      } else {  // has no radar
        if (has_vidar) {
          marker.text += "-CV";
        } else {
          marker.text += "-C";
        }
      }
    } else {  // has no camera
      if (has_radar) {
        if (has_vidar) {
          marker.text += "-RV";
        } else {
          marker.text += "-R";
        }
      } else {
        if (has_vidar) {
          marker.text += "-V";
        } else {
          // no sensor supplement
        }
      }
    }
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

// Modify-guoxiaoxiao
void Visualization::linkColorToSensorType(const perception::Object& obj,
                                          visualization_msgs::Marker& marker) {
  // filter objs not on the map
  if (obj.status() == 2) {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  } else {
    bool has_camera = obj.has_camera_supplement();
    bool has_lidar = obj.has_lidar_supplement();
    bool has_radar = obj.has_radar_supplement();
    // Modify-guoxiaoxiao
    bool has_vidar = obj.has_vidar_supplement();

    // Modify-guoxiaoxiao
    if (has_lidar) {
      if (has_camera) {     // Orange label
        if (has_radar) {    // green label
          if (has_vidar) {  // purple label
            // purple label - LCRV
            marker.color.r = 0.58f;
            marker.color.g = 0.44f;
            marker.color.b = 0.86f;
          } else {
            // green label - LCR
            marker.color.r = 0.f;
            marker.color.g = 1.f;
            marker.color.b = 0.f;
          }
        } else {  // has no radar
          if (has_vidar) {
            // purple label - LCV
            marker.color.r = 0.58f;
            marker.color.g = 0.44f;
            marker.color.b = 0.86f;
          } else {
            // Orange label - LC
            marker.color.r = 1.f;
            marker.color.g = 0.65f;
            marker.color.b = 0.f;
          }
        }
      } else {  // has no camera
        if (has_radar) {
          if (has_vidar) {
            // purple label - LRV
            marker.color.r = 0.58f;
            marker.color.g = 0.44f;
            marker.color.b = 0.86f;
          } else {
            // green label - LR
            marker.color.r = 0.f;
            marker.color.g = 1.f;
            marker.color.b = 0.f;
          }
        } else {
          if (has_vidar) {
            // purple label - LV
            marker.color.r = 0.58f;
            marker.color.g = 0.44f;
            marker.color.b = 0.86f;
          } else {
            // keep origin color - L
          }
        }
      }
    } else {  // has no lidar
      if (has_camera) {
        if (has_radar) {
          if (has_vidar) {
            // red label - CRV
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          } else {
            // red label - CR
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          }
        } else {
          if (has_vidar) {
            // red label - CV
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          } else {
            // red label - C
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          }
        }
      } else {
        if (has_radar) {
          if (has_vidar) {
            // red label - RV
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          } else {
            // red label - R
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          }
        } else {
          if (has_vidar) {
            // red label - V
            marker.color.r = 1.f;
            marker.color.g = 0.f;
            marker.color.b = 0.f;
          } else {
            // no sensor supplement
          }
        }
      }
    }
  }
}

// Modify-guoxiaoxiao
void Visualization::linkColorToObjType(const perception::Object& obj,
                                       visualization_msgs::Marker& marker) {
  // filter objs not on the map
  if (obj.status() == 2) {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  } else {
    perception::ObjectType type = obj.type();
    switch (type) {
      case perception::ObjectType::TYPE_PEDESTRIAN:
        // Yellow label
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        break;
      case perception::ObjectType::TYPE_BICYCLE:
        // green label
        marker.color.r = 0.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        break;
      case perception::ObjectType::TYPE_MOTOR:
        // green label
        marker.color.r = 0.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        break;
      case perception::ObjectType::TYPE_RIDER:
        // green label
        marker.color.r = 0.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        break;
      case perception::ObjectType::TYPE_CAR:
        // bule label
        marker.color.r = 3.0 / 255.0;
        marker.color.g = 204.0 / 255.0;
        marker.color.b = 238.0 / 255.0;
        break;
      case perception::ObjectType::TYPE_TRUCK:
        // purple label
        marker.color.r = 0.58f;
        marker.color.g = 0.44f;
        marker.color.b = 0.86f;
        break;
      case perception::ObjectType::TYPE_BUS:
        // purple label
        marker.color.r = 0.58f;
        marker.color.g = 0.44f;
        marker.color.b = 0.86f;
        break;
      case perception::ObjectType::TYPE_TRAIN:
        // purple label
        marker.color.r = 0.58f;
        marker.color.g = 0.44f;
        marker.color.b = 0.86f;
        break;
      case perception::ObjectType::TYPE_UNKNOWN:
        // red label
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 1.f;
        break;
      case perception::ObjectType::TYPE_UNKNOWN_DYNAMIC:
        // red label
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 1.f;
        break;
      case perception::ObjectType::TYPE_UNKNOWN_STATIC:
        // red label
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 1.f;
        break;
      case perception::ObjectType::TYPE_SIGN:
        // Orange label
        marker.color.r = 1.f;
        marker.color.g = 0.65f;
        marker.color.b = 0.f;
        break;
      case perception::ObjectType::TYPE_TRIANGLEROADBLOCK:
        // Orange label
        marker.color.r = 1.f;
        marker.color.g = 0.65f;
        marker.color.b = 0.f;
        break;
      default:
        // Orange label
        marker.color.r = 1.f;
        marker.color.g = 0.65f;
        marker.color.b = 0.f;
        break;
    }
  }
}

// Debug-guoxiaoxiao
void Visualization::CameraObjDisplay(const fusion::FrameConstPtr& camera_frame,
                                     ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker box_marker;

  box_marker.header.frame_id = "base_link";
  box_marker.header.stamp.fromSec(camera_frame->timestamp);
  box_marker.ns = "camera";
  box_marker.action = visualization_msgs::Marker::ADD;
  box_marker.pose.orientation.w = 1.0;
  box_marker.type = visualization_msgs::Marker::CUBE;

  box_marker.scale.x = 4.0f;
  box_marker.scale.y = 2.0f;
  box_marker.scale.z = 2.3f;
  box_marker.color.a = 0.5f;
  box_marker.color.r = 1.0f;
  box_marker.color.g = 1.0f;
  box_marker.color.b = 1.0f;

  box_marker.pose.orientation.x = 0.0;
  box_marker.pose.orientation.y = 0.0;
  box_marker.pose.orientation.z = 0.0;
  box_marker.pose.orientation.w = 1.0;

  int mk_id = 0;
  static size_t last_marker_size_ = 0;

  for (size_t i = 0; i < camera_frame->objects.size(); ++i) {
    box_marker.id = mk_id++;
    box_marker.pose.position.x = camera_frame->objects[i]->center(0);
    box_marker.pose.position.y = camera_frame->objects[i]->center(1);
    box_marker.pose.position.z = 0;
    marker_array.markers.emplace_back(box_marker);
  }
  while (mk_id < last_marker_size_) {
    box_marker.id = mk_id++;
    box_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(box_marker);
  }
  last_marker_size_ = marker_array.markers.size();
  publisher.publish(marker_array);
}

}  // namespace calib_check
}  // end of namespace perception
