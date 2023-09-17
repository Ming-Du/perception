#include "track_info_marker_pub.h"

namespace perception {
namespace fusion {

std::vector<ROS_VISUALIZATION_MARKER>& TrackInfoPubMarker::display(
    const TrackedObjects& tracked_objects) {
  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < obj_vec.size()) {
    params_.max_obj_size = obj_vec.size();
    params_.marker_list.resize(params_.max_obj_size);
  } else {
    for (size_t i = obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    auto& tmp_marker = params_.marker_list[i];
    tmp_marker.color.r = 0.;
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 0.;
    tmp_marker.scale = params_.default_scale_type;
    tmp_marker.ns = "track_info";
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.id = i;
    tmp_marker.header.frame_id = options_.frame_id;
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& obj = tmp_obj.obj();

    Eigen::Vector3d pos;
    pos << obj.center().x(), obj.center().y(), obj.center().z();
    pos.x() -= 0.5;
    auto& tmp_marker = params_.marker_list[i];

    double speed =
        std::sqrt(std::pow(tmp_obj.velocity().x(), 2) + std::pow(tmp_obj.velocity().y(), 2));

    bool is_static = false;
    if (obj.motion_state() == 0)
      is_static = true;
    std::string text_track = "<" + num2str<int>(obj.id(), 0) + ">" +
                             num2str<double>(speed * 3.6f, 1) + "km/h" + "_s_" +
                             num2str<int>(is_static, 0);

    drawText(pos, text_track, tmp_marker, 1.0, 1.0);
  }

  return params_.marker_list;
}

std::vector<ROS_VISUALIZATION_MARKER>& TrackInfoPubMarker::display_ego_text(
    const localization::Localization localization) {
  std::vector<localization::Localization> loc_vec;
  loc_vec.push_back(localization);

  if (params_.max_obj_size < loc_vec.size()) {
    params_.max_obj_size = loc_vec.size();
    params_.marker_list.resize(params_.max_obj_size);
  } else {
    for (size_t i = loc_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }


  for (size_t i = 0; i < loc_vec.size(); ++i) {
    auto& tmp_marker = params_.marker_list[i];
    tmp_marker.color.r = 1.0;
    tmp_marker.color.g = 0.0;
    tmp_marker.color.b = 1.0;
    tmp_marker.scale = params_.default_scale_type;
    tmp_marker.ns = "ego_info";
    tmp_marker.id = i;
    tmp_marker.header.frame_id = options_.frame_id;
  }

  for (size_t i = 0; i < loc_vec.size(); ++i) {
    const auto& tmp_obj = loc_vec[i];
    auto& tmp_marker = params_.marker_list[i];
    
    Eigen::Vector3d pos;
    pos << -1.5, 0.0, 0.0;
    double host_speed =
        std::sqrt(std::pow(tmp_obj.longitudinal_v(), 2) + std::pow(tmp_obj.lateral_v(), 2));
    std::string text_track = "<ego:" + num2str<double>(host_speed * 3.6, 1) + "km/h ";
    text_track += "Yaw_v: " + num2str<double>(tmp_obj.yaw_v(), 3) + ">";


    drawText(pos, text_track, tmp_marker, 1.0, 3.0);
  }

  return params_.marker_list;
}

}  // namespace fusion
}  // namespace perception
