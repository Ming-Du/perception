#include "arrow_marker_pub.h"

namespace perception {
namespace fusion {
std::vector<ROS_VISUALIZATION_MARKER>& ArrowMarkerPub::display(
    const TrackedObjects& tracked_objects) {
  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < obj_vec.size()) {
    params_.max_obj_size = obj_vec.size();
    params_.marker_list.resize(obj_vec.size());
  } else {
    for (size_t i = obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    auto& tmp_maker = params_.marker_list[i];
    tmp_maker.color = params_.default_color_type;
    tmp_maker.scale = params_.default_scale_type;
    tmp_maker.ns = "arrow";
    tmp_maker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_maker.id = i;
    tmp_maker.header.frame_id = options_.frame_id;
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& tmp_marker = params_.marker_list[i];
    drawArrow(tmp_obj, tmp_marker, 1);
  }

  return params_.marker_list;
}

std::vector<ROS_VISUALIZATION_MARKER>& ArrowMarkerPub::display_app(
    const TrackedObjects& tracked_objects, const localization::Localization localization, std::string ns, bool is_arrow) {
  localization_ = localization;
  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < obj_vec.size()) {
    params_.max_obj_size = obj_vec.size();
    params_.marker_list.resize(obj_vec.size());
  } else {
    for (size_t i = obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    auto& tmp_maker = params_.marker_list[i];
    tmp_maker.color = params_.default_color_type;
    tmp_maker.scale = params_.default_scale_type;
    tmp_maker.ns = ns;
    tmp_maker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_maker.id = i;
    tmp_maker.header.frame_id = options_.frame_id;
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& tmp_marker = params_.marker_list[i];
    drawArrow(tmp_obj, tmp_marker, is_arrow, 1);
  }

  return params_.marker_list;
}

void ArrowMarkerPub::drawArrow(const TrackedObject& obj,
                               ROS_VISUALIZATION_MARKER& marker,
                               bool is_arrow,
                               double alpha) {
  marker.type = ROS_VISUALIZATION_MARKER::ARROW;
  marker.action = ROS_VISUALIZATION_MARKER::ADD;

  const auto& center = obj.obj().center();
  const auto& size = obj.obj().size();
  marker.color.a = alpha;
  double vx_baselink = .0;
  double vy_baselink = .0;
  if (is_arrow) {
    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 0;
  } else {
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 0;
    //计算base_link方向下面obj的速度
    double host_yaw_global = localization_.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    vx_baselink = obj.velocity().x() * cos_host + obj.velocity().y() * sin_host;
    vy_baselink = -obj.velocity().x() * sin_host + obj.velocity().y() * cos_host;
    if (fabs(vx_baselink) < 0.001 && fabs(vy_baselink) < 0.001) {
      marker.action = ROS_VISUALIZATION_MARKER::DELETE;
      return;
    }
  }

  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();
  marker.scale.x = 3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;
  Eigen::Quaterniond quat = RPYToQuaterniond(0.0, 0.0, is_arrow ? static_cast<float>(obj.obj().angle()) : std::atan2(vy_baselink, vx_baselink));
  marker.pose.orientation.w = quat.w();
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
}

Eigen::Quaterniond ArrowMarkerPub::RPYToQuaterniond(const float& roll,
                                                    const float& pitch,
                                                    const float& yaw) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond _orientation = yawAngle * pitchAngle * rollAngle;
  return _orientation;
}
}  // namespace fusion
}  // namespace perception
