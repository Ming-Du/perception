#include "visualisation.h"

#include "perception/fusion/common/global_util.h"
#include "perception/fusion_mid/common/object_type_convert.h"

namespace perception {
namespace mid_fusion {

bool Visualization::TextDisplay(TrackedObjects& objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "base_link";
  marker.ns = "rviz_fusion_mid_text";
  marker.pose.orientation.w = 1.;
  marker.color.a = 1.f;

  static int test_last_marker_size = 0;
  int32_t mk_id = 0;

  for (int32_t m = 0; m < objects.objs_size(); ++m) {
    TrackedObject* object = objects.mutable_objs(m);
    auto& obj = object->obj();

    // add text info
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.header.stamp.fromSec(obj.time_stamp());  
    marker.text.clear();
    marker.text = "<" + std::to_string(obj.id()) + ">";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 255/255;
    marker.color.g = 255/255;
    marker.color.b = 0/255;
    marker.color.a = 1.f;// Don't forget to set the alpha!

    const auto& center = obj.center();
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z() + 2;

    AddTypeText(obj, marker);
    AddNoiseStateText(obj, marker);
    const auto& vel_xy = obj.velocity();
    double speed =
        std::sqrt(std::pow(object->velocity().x(), 2) + std::pow(object->velocity().y(), 2));

    // add object relative velocity
    std::string rel_x = NumToStr<float>(vel_xy.x() * 3.6, 1);
    std::string rel_y = NumToStr<float>(vel_xy.y() * 3.6, 1);

    std::string px = NumToStr<float>(center.x(), 1);
    std::string py = NumToStr<float>(center.y(), 1);
    // marker.text += "\nvx:" + abs_x + "vy:" + abs_y + "\n";
    marker.text += "\npx:" + px + " py:" + py;
    marker.text += "\nabs_spd:" + NumToStr<double>(speed * 3.6f, 1) + "km/h";
    if (obj.status() == 1) {
      marker.text += "/1";
    } else if (obj.status() == 12) {
      marker.text += "/12";
    }
    marker.id = mk_id++;
    marker_array.markers.emplace_back(marker);
  }

  marker.text.clear();
  marker.text = "Ego";
  marker.scale.x = 0.;
  marker.scale.y = 0.;
  marker.scale.z = 1.5;
  marker.color.r = 1.f;
  marker.color.g = 1.f;
  marker.color.b = 0.f;
  marker.color.a = 0.f;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.;

  marker.id = mk_id++;
  marker_array.markers.emplace_back(marker);

  while (mk_id < test_last_marker_size) {
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  test_last_marker_size = marker_array.markers.size();
  return true;
}  // end of TextDisplay()

void Visualization::lineMarker(std::unordered_map<std::string, Eigen::MatrixXd> world_points_map,
                               visualization_msgs::MarkerArray& markers) {
  visualization_msgs::Marker line_markers;
  line_markers.header.frame_id = "base_link";
  line_markers.header.stamp = ros::Time();
  line_markers.ns = "mid_fusion";
  line_markers.id = 0;
  // line_markers.lifetime = ros::Duration(1.0);
  line_markers.action = visualization_msgs::Marker::ADD;
  // line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.type = visualization_msgs::Marker::LINE_LIST;

  line_markers.scale.x = 0.1;
  line_markers.scale.y = 0.1;
  line_markers.scale.z = 0.1;
  line_markers.color.r = 0.0;
  line_markers.color.g = 1.0;
  line_markers.color.b = 0.0;
  line_markers.color.a = 1.0;// Don't forget to set the alpha!
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

void Visualization::PointsDisplay(const std::vector<std::vector<float>> frustum_points,
                                  const std::vector<float> box_det2d,
                                  const double pub_timestamp,
                                  visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(pub_timestamp);
  point_marker.ns = "2d_frustum_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.type = visualization_msgs::Marker::POINTS;
  point_marker.pose.position = geometry_msgs::Point();
  point_marker.scale.x = 0.3;
  point_marker.scale.y = 0.1;
  point_marker.scale.z = 0.1;
  point_marker.color.a = 1.0; // Don't forget to set the alpha!

  static size_t box_last_marker_size = 0;
  int marker_id = 0;
  for (const std::vector<float>& points : frustum_points) {
    ObjectTypeToColor(FpnetIndexConvertToLidarType(box_det2d[marker_id * 7 + 6]), point_marker);
    point_marker.id = marker_id++;
    point_marker.points.clear();
    const int point_size = points.size() / 3;
    for (int i = 0; i < point_size; i++) {
      geometry_msgs::Point frustum_point;
      frustum_point.x = points[3 * i + 0];
      frustum_point.y = points[3 * i + 1];
      frustum_point.z = points[3 * i + 2];
      point_marker.points.push_back(frustum_point);
      // std::cout << "frustum points: " << frustum_point << "\n";
    }
    marker_array.markers.emplace_back(point_marker);
  }

  while (marker_id < box_last_marker_size) {
    point_marker.id = marker_id++;
    point_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(point_marker);
  }

  box_last_marker_size = marker_array.markers.size();
}

bool Visualization::BboxDisplay(TrackedObjects& tracked_objects,
                                visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "rviz_fusion_mid";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.;

  static size_t box_last_marker_size = 0;
  int mk_id = 0;
  for (size_t it = 0; it < tracked_objects.objs_size(); ++it) {
    const perception::Object& object = tracked_objects.objs(it).obj();
    marker.header.stamp.fromSec(object.time_stamp());
    // Don't display vegetation obstacle box.
    if (object.type() == perception::ObjectType::TYPE_VEGETATION) {
      continue;
    }
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
      marker.color.a = 1.0; // Don't forget to set the alpha!
      // Modify-guoxiaoxiao
      // linkColorToSensorType(object, marker);
      ObjectSetColorByType(object, marker);
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
      arrow_marker.color.a = 1.; // Don't forget to set the alpha!
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
      cube_marker.color.a = 0.75; // Don't forget to set the alpha!
      ObjectSetColorByType(object, cube_marker);
      cube_marker.pose.position.x = object.center().x();
      cube_marker.pose.position.y = object.center().y();
      cube_marker.pose.position.z = object.center().z();
      tf::Quaternion quat_ = tf::createQuaternionFromYaw(object.angle());
      tf::quaternionTFToMsg(quat_, cube_marker.pose.orientation);
      marker_array.markers.emplace_back(cube_marker);
    }
  }  // end of for

  while (mk_id < box_last_marker_size) {
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  box_last_marker_size = marker_array.markers.size();
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
  static size_t radar_last_marker_size_ = 0;

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
    // text += "\nx:" + NumToStr<float>(radar_frame->objects[i]->center(0),
    // 1) + ";"; text += "y:" +
    // NumToStr<float>(radar_frame->objects[i]->center(1), 1); text +=
    // "\nvx:" + NumToStr<float>(vel_x_rel*3.6, 1) + ";"; text += "vy:" +
    // NumToStr<float>(vel_y_rel*3.6, 1); text += "\nvx:" +
    // NumToStr<float>(vel_x_abs*3.6, 1) + ";"; text += "vy:" +
    // NumToStr<float>(vel_y_abs*3.6, 1);
    text_marker.text = text;
    marker_array.markers.emplace_back(text_marker);

    point_marker.id = mk_id++;
    point_marker.pose.position.x = radar_frame->objects[i]->center(0);
    point_marker.pose.position.y = radar_frame->objects[i]->center(1);
    point_marker.pose.position.z = 0;
    marker_array.markers.emplace_back(point_marker);
  }
  while (mk_id < radar_last_marker_size_) {
    text_marker.id = mk_id++;
    text_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(text_marker);
  }
  radar_last_marker_size_ = marker_array.markers.size();
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
      marker.text += "CONE";
      break;
    case perception::ObjectType::TYPE_VEGETATION:
      marker.text += "VEG";
      break;
    default:
      break;
  }
}

void Visualization::AddNoiseStateText(const perception::Object& obj, visualization_msgs::Marker& marker) {
  perception::NoiseState noise_state = obj.noise_state();
  switch (noise_state) {
    case perception::NoiseState::NOISE_OBJECT:
      marker.text += "+N0";
      break;
    case perception::NoiseState::NOISE_NOISE:
      marker.text += "+N1";
      break;
    case perception::NoiseState::NOISE_SUSPECTED:
      marker.text += "+N2";
      break;
    case perception::NoiseState::NOISE_FLOWERBEDS:
      marker.text += "+N3";
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

void Visualization::AddPolygonPoint(const perception::Object& obj,
                                    visualization_msgs::Marker& marker) {
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

    marker.points.emplace_back(cub_points[i]);
    marker.points.emplace_back(cub_points[next]);
  }
  for (size_t i = 0; i < p_size; ++i) {
    size_t next = i + 1;
    next = next < p_size ? next : 0;

    marker.points.emplace_back(cub_points[i + p_size]);
    marker.points.emplace_back(cub_points[next + p_size]);
  }

  for (size_t i = 0; i < p_size; ++i) {
    marker.points.emplace_back(cub_points[i]);
    marker.points.emplace_back(cub_points[i + p_size]);
  }
}
bool Visualization::PolygonDisplay(TrackedObjects& tracked_objects,
                                   visualization_msgs::MarkerArray& marker_array) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "rviz_fusion_mid_polygon";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.orientation.w = 1.;
  marker.color.a = 1.0;

  static size_t polygon_last_marker_size = 0;
  int mk_id = 0;

  int polygon_size = 0;
  for (size_t i = 0; i < tracked_objects.objs_size(); ++i) {
    const perception::Object& object = tracked_objects.objs(i).obj();
    polygon_size += object.polygon_size();
  }
  marker.points.reserve(polygon_size * 6);
  // marker.colors.reserve(polygon_size * 6);
  marker.points.clear();

  for (size_t i = 0; i < tracked_objects.objs_size(); ++i) {
    const perception::Object& object = tracked_objects.objs(i).obj();
    marker.header.stamp.fromSec(object.time_stamp());

    // Modify-guoxiaoxiao - wireframe
    marker.id = mk_id++;
    marker.points.clear();
    AddPolygonPoint(object, marker);
    // Modify-guoxiaoxiao
    // linkColorToSensorType(object, marker);
    ObjectSetColorByType(object, marker);
    marker_array.markers.emplace_back(marker);
  }  // end of for

  while (mk_id < polygon_last_marker_size) {
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(marker);
  }

  polygon_last_marker_size = marker_array.markers.size();
  return true;
}  // end of PolygonDisplay()
// Modify-guoxiaoxiao

void Visualization::ObjectTypeToColor(const perception::ObjectType type,
                                      visualization_msgs::Marker& marker) {
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
    case perception::ObjectType::TYPE_VEGETATION:
      // Deep green label
      marker.color.r = 48.0 / 255.0;
      marker.color.g = 128.0 / 255.0;
      marker.color.b = 20.0 / 255.0;
      break;
    default:
      // Orange label
      marker.color.r = 1.f;
      marker.color.g = 0.65f;
      marker.color.b = 0.f;
      break;
  }
}

void Visualization::ObjectSetColorByType(const perception::Object& obj,
                                          visualization_msgs::Marker& marker) {
  // filter objs not on the map
  if (obj.status() == 2) {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  } else {
    perception::ObjectType type = obj.type();
    ObjectTypeToColor(type, marker);
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
  static size_t cam_last_marker_size_ = 0;

  for (size_t i = 0; i < camera_frame->objects.size(); ++i) {
    box_marker.id = mk_id++;
    box_marker.pose.position.x = camera_frame->objects[i]->center(0);
    box_marker.pose.position.y = camera_frame->objects[i]->center(1);
    box_marker.pose.position.z = 0;
    marker_array.markers.emplace_back(box_marker);
  }
  while (mk_id < cam_last_marker_size_) {
    box_marker.id = mk_id++;
    box_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(box_marker);
  }
  cam_last_marker_size_ = marker_array.markers.size();
  publisher.publish(marker_array);
}

}  // namespace mid_fusion
}  // end of namespace perception
