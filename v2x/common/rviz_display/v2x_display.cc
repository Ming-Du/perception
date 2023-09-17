#include "v2x_display.h"

namespace perception {
namespace v2x {
void V2xDisplay::ObuRTEDisplay(const perception::ObuObjects obu_objets,
                               const localization::Localization localization,
                               ros::Publisher &publisher) {
  double timestamp = obu_objets.header().stamp().sec() +
                     obu_objets.header().stamp().nsec() * 1e-9;
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

  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = "base_link";
  arrow_marker.header.stamp.fromSec(timestamp);
  arrow_marker.ns = "rviz_fusion";
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.scale.x = 3.0;
  arrow_marker.scale.y = 0.5;
  arrow_marker.scale.z = 0.5;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 1.0;
  arrow_marker.color.a = 1.0;

  int marker_id = 0;
  for (size_t i = 0; i < obu_objets.objs_size(); ++i) {
    perception::ObuObject obu_object = obu_objets.objs(i);
    perception::Object obu_obj = obu_object.obj();
    perception::ObjectSource obu_source = obu_object.source();
    if (obu_source != perception::ObjectSource::V2N_RSI)
      continue;
    perception::v2x::point object_gps;
    object_gps.x = obu_object.longitude();
    object_gps.y = obu_object.latitude();
    perception::v2x::point object_utm = GPS2MCT(object_gps);
    double host_yaw_global = localization.yaw();  // 东0，北90，西180，南270
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    double utm_x_diff = object_utm.x - localization.position().x();
    double utm_y_diff = object_utm.y - localization.position().y();
    double pos_x_ego = utm_x_diff * cos_host + utm_y_diff * sin_host;
    double pos_y_ego = -utm_x_diff * sin_host + utm_y_diff * cos_host;

    arrow_marker.id = marker_id++;
    arrow_marker.pose.position.x = pos_x_ego;
    arrow_marker.pose.position.y = pos_y_ego;
    arrow_marker.pose.position.z = 0.0;
    tf::Quaternion quat = tf::createQuaternionFromYaw(obu_object.obj().angle());
    tf::quaternionTFToMsg(quat, arrow_marker.pose.orientation);
    marker_array.markers.push_back(arrow_marker);
    line_strip.points.clear();
    int rte_size = obu_object.obj().polygon_size();
    for (size_t i = 0; i < rte_size; ++i) {
      geometry::Point rte_polygon = obu_obj.polygon(i);
      perception::v2x::point ponit_gps;
      ponit_gps.x = rte_polygon.x();
      ponit_gps.y = rte_polygon.y();
      perception::v2x::point ponit_utm = GPS2MCT(ponit_gps);
      double polygon_utm_x_diff = ponit_utm.x - localization.position().x();
      double polygon_utm_y_diff = ponit_utm.y - localization.position().y();
      double polygon_pos_x_ego = polygon_utm_x_diff * cos_host + polygon_utm_y_diff * sin_host;
      double polygon_pos_y_ego = -polygon_utm_x_diff * sin_host + polygon_utm_y_diff * cos_host;

      geometry_msgs::Point point;
      point.x = polygon_pos_x_ego;
      point.y = polygon_pos_y_ego;
      line_strip.points.push_back(point);
      if (i == rte_size - 1) {
        geometry::Point rte_polygon_0 = obu_obj.polygon(0);
        perception::v2x::point ponit_gps_0;
        ponit_gps_0.x = rte_polygon_0.x();
        ponit_gps_0.y = rte_polygon_0.y();
        perception::v2x::point ponit_utm_0 = GPS2MCT(ponit_gps_0);
        double polygon_utm_x_diff_0 = ponit_utm_0.x - localization.position().x();
        double polygon_utm_y_diff_0 = ponit_utm_0.y - localization.position().y();
        double polygon_pos_x_ego_0 = polygon_utm_x_diff_0 * cos_host + polygon_utm_y_diff_0 * sin_host;
        double polygon_pos_y_ego_0 = -polygon_utm_x_diff_0 * sin_host + polygon_utm_y_diff_0 * cos_host;
        geometry_msgs::Point point_0;
        point_0.x = polygon_pos_x_ego_0;
        point_0.y = polygon_pos_y_ego_0;
        line_strip.points.push_back(point_0);
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
    if (marker_array.markers.size() == 0)
      break;
    line_strip.id = marker_id++;
    line_strip.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(line_strip);
  }
  last_marker_size = marker_array.markers.size();

  publisher.publish(marker_array);
}

void V2xDisplay::ObuPNTDisplay(const perception::ObuObjects obu_objets,
                               const localization::Localization localization,
                               ros::Publisher &publisher) {
  Eigen::Matrix4d transform_mat_g2l;
  transGlobal2VehicleMat(localization, transform_mat_g2l);
  double timestamp = obu_objets.header().stamp().sec() +
                     obu_objets.header().stamp().nsec() * 1e-9;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker point_marker;
  visualization_msgs::Marker arrow_marker;
  text_marker.header.frame_id = "base_link";
  text_marker.header.stamp.fromSec(timestamp);
  text_marker.ns = "label";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp.fromSec(timestamp);
  point_marker.ns = "box";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  // point_marker.type = visualization_msgs::Marker::SPHERE;
  point_marker.type = visualization_msgs::Marker::LINE_LIST;

  arrow_marker.header.frame_id = "base_link";
  arrow_marker.header.stamp.fromSec(timestamp);
  arrow_marker.ns = "arrow";
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
  for (size_t i = 0; i < obu_objets.objs_size(); ++i) {
    perception::ObuObject obu_object = obu_objets.objs(i);
    if (obu_object.source() == perception::ObjectSource::V2N_RSI)
      continue;
    perception::Object obu_obj = obu_object.obj();
    perception::v2x::point object_gps;
    object_gps.x = obu_object.longitude();
    object_gps.y = obu_object.latitude();
    perception::v2x::point object_utm = GPS2MCT(object_gps);
    Eigen::Vector4d center_3d;
    center_3d << object_utm.x, object_utm.y, localization.altitude(), 1;
    center_3d = transform_mat_g2l * center_3d;
    obu_obj.mutable_center()->set_x(center_3d.x());
    obu_obj.mutable_center()->set_y(center_3d.y());
    obu_obj.mutable_center()->set_z(obu_object.alt());
    mk_id++; // if want to show text, delete commit
    text_marker.id = mk_id;
    text_marker.pose.position.x = center_3d.x();
    text_marker.pose.position.y = center_3d.y();
    text_marker.pose.position.z = obu_object.alt();
    float angle = obu_object.heading();
    auto yaw = M_PI_2 - angle * DEG_TO_RAD_V2X;
    double x_velociety_global = obu_object.speed() * std::cos(yaw);
    double y_velociety_global = obu_object.speed() * std::sin(yaw);
    Eigen::Vector4d velocity_3d;
    velocity_3d << x_velociety_global, y_velociety_global, 0, 0;
    velocity_3d = transform_mat_g2l * velocity_3d;
    double vel_x_rel = velocity_3d.x();
    double vel_y_rel = velocity_3d.y();
    text_marker.text += "<" + std::to_string(obu_object.obj().id()) + ">";
    AddTypeText(obu_object.obj(), text_marker);
    text_marker.text += "\nx:" + NumToStr<float>(center_3d.x(), 2) + ",";
    text_marker.text += "y:" + NumToStr<float>(center_3d.y(), 2);
    text_marker.text += "\nVx:" + NumToStr<float>(vel_x_rel * 3.6, 1) + ",";
    text_marker.text += "Vy:" + NumToStr<float>(vel_y_rel * 3.6, 1) + "km/h";
    marker_array.markers.push_back(text_marker); // if want to show text, delete commit
    text_marker.text.clear();
    float angle_temp = yaw - localization.yaw();
    if (angle_temp > M_PI)
      angle_temp -= 2 * M_PI;
    else if (angle_temp < -M_PI)
      angle_temp += 2 * M_PI;
    obu_obj.set_angle(angle_temp);
    // add arrow of obstacle
    {
      arrow_marker.id = mk_id++;
      arrow_marker.pose.position.x = center_3d.x();
      arrow_marker.pose.position.y = center_3d.y();
      arrow_marker.pose.position.z = obu_object.alt();
      tf::Quaternion quat = tf::createQuaternionFromYaw(angle_temp);
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
      AddBboxPoint(obu_obj, point_marker);
    }
    marker_array.markers.push_back(point_marker);
    point_marker.points.clear();
  }
  while (mk_id < last_marker_size) {
    if (marker_array.markers.size() == 0)
      break;
    point_marker.id = mk_id++;
    point_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.emplace_back(point_marker);
  }
  last_marker_size = marker_array.markers.size();
  publisher.publish(marker_array);
}

void V2xDisplay::AddTypeText(const perception::Object obj,
                             visualization_msgs::Marker &marker){
  perception::ObjectType type = obj.type();
  switch (type){
    case perception::ObjectType::TYPE_PEDESTRIAN:
      marker.text += "\nPED";
      break;
    case perception::ObjectType::TYPE_BICYCLE:
      marker.text += "\nBIC";
      break;
    case perception::ObjectType::TYPE_MOTOR:
      marker.text += "\nMOTOR";
      break;
    case perception::ObjectType::TYPE_RIDER:
      marker.text += "\nRIDER";
      break;
    case perception::ObjectType::TYPE_CAR:
      marker.text += "\nCAR";
      break;
    case perception::ObjectType::TYPE_TRUCK:
      marker.text += "\nTRUCK";
      break;
    case perception::ObjectType::TYPE_BUS:
      marker.text += "\nBUS";
      break;
    case perception::ObjectType::TYPE_TRAIN:
      marker.text += "\nTRAIN";
      break;
    case perception::ObjectType::TYPE_UNKNOWN:
      marker.text += "\nUNKNOWN";
      break;
    case perception::ObjectType::TYPE_UNKNOWN_DYNAMIC:
      marker.text += "\nUNKNOWN_D";
      break;
    case perception::ObjectType::TYPE_UNKNOWN_STATIC:
      marker.text += "\nUNKNOWN_S";
      break;
    case perception::ObjectType::TYPE_SIGN:
      marker.text += "\nSIGN";
      break;
    case perception::ObjectType::TYPE_TRIANGLEROADBLOCK:
      marker.text += "\nTRAFFICCONE";
      break;
    case perception::ObjectType::TYPE_WARNINGTRIANGLE:
      marker.text += "\nWARNINGTRIANGLE";
      break;
    case perception::ObjectType::TYPE_ROADWORK_OCCUPY_0501:
      marker.text += "\nROADWORK_OCCUPY_0501";
      break;
    case perception::ObjectType::TYPE_ROADWORK_BREAK_0502:
      marker.text += "\nROADWORK_BREAK_0502";
      break;
    default:
      break;
  }
}

void V2xDisplay::AddBboxPoint(const perception::Object obj,
                              visualization_msgs::Marker &marker) {
  /* lfd lbd  rfd rbd   lfu lbu  rfu rbu*/
  geometry_msgs::Point lfd; //lbd
  geometry_msgs::Point lbd; //rbd
  geometry_msgs::Point rfd; //lfd
  geometry_msgs::Point rbd; //rfd
  geometry_msgs::Point lfu; //lbu
  geometry_msgs::Point lbu; //rbu
  geometry_msgs::Point rfu; //lfu
  geometry_msgs::Point rbu; //rfu

  SetPointLFD(obj, lfd);
  SetPointLBD(obj, lbd);
  SetPointRFD(obj, rfd);
  SetPointRBD(obj, rbd);
  SetPointLFU(obj, lfu);
  SetPointLBU(obj, lbu);
  SetPointRFU(obj, rfu);
  SetPointRBU(obj, rbu);

  double yaw = obj.angle();    //tracked_objects.objs(it).yaw();
  if (abs(yaw) > 0.01) {
    double cx = obj.center().x();
    double cy = obj.center().y();
    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);
    double dcx = -cx*cosYaw+cy*sinYaw+cx;
    double dcy = -cx*sinYaw-cy*cosYaw+cy;
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

void V2xDisplay::PointRotate(const double cosYaw, const double sinYaw, 
                             const double dcx,const double dcy,
                             geometry_msgs::Point &point){
  double px = point.x;
  double py = point.y;
  point.x = px*cosYaw-py*sinYaw+dcx;
  point.y = px*sinYaw+py*cosYaw+dcy;
}

}  // namespace v2x
}  // namespace perception
