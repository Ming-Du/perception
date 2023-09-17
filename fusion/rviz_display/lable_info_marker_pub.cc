#include "lable_info_marker_pub.h"

namespace perception {
namespace fusion {
std::vector<ROS_VISUALIZATION_MARKER>& LabelInfosMarkerPub::display(
    const TrackedObjects& tracked_objects) {
  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < 2 * obj_vec.size()) {
    params_.max_obj_size = 2 * obj_vec.size();
    params_.marker_list.resize(params_.max_obj_size);
  } else {
    for (size_t i = obj_vec.size(); i < params_.max_obj_size/2; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
    for (size_t i = params_.max_obj_size/2 + obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    auto& tmp_marker = params_.marker_list[i];
    tmp_marker.color.r = 0.0;
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 1.;
    tmp_marker.scale = params_.default_scale_type;
    tmp_marker.ns = "label_info";
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.id = i;
    tmp_marker.header.frame_id = options_.frame_id;

    auto& camera_marker = params_.marker_list[i+params_.max_obj_size/2];
    camera_marker.color.r = 0.5;
    camera_marker.color.g = 1.0;
    camera_marker.color.b = 0.5;
    camera_marker.scale = params_.default_scale_type;
    camera_marker.ns = "camera_label_info";
    camera_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    camera_marker.id = i;
    camera_marker.header.frame_id = options_.frame_id;
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& obj = tmp_obj.obj();

    bool _has_lidar = obj.has_lidar_supplement();
    bool _has_radar = obj.has_radar_supplement();
    bool _has_falcon_lidar = obj.has_falcon_lidar_supplement();
    bool _has_obu = obj.has_obu_supplement();
    bool _has_vidar = obj.has_vidar_supplement();  // ming.du
    Eigen::Vector3d center;
    center << obj.center().x(), obj.center().y(), obj.center().z();
    Eigen::Vector3d pos = center;
    pos.z() += 0.5;

    auto& tmp_marker = params_.marker_list[i];
    std::string text_label = object_type_2_name_map.at(obj.type()) + "-";
    //@lijian,显示匹配来源和id
    if (_has_lidar) {
      text_label += "L";
      text_label += std::to_string(obj.match_ids(0));
    }
    if (_has_falcon_lidar) {
      text_label += "F";
      text_label += std::to_string(obj.match_ids(3));
    }
    if (_has_radar) {
      text_label += "R";
      text_label += std::to_string(obj.match_ids(2));
    }
    if (_has_obu) {
      text_label += "O";
      text_label += std::to_string(obj.match_ids(8));
    }
    if (_has_vidar) {
      text_label += "V";
      text_label += std::to_string(obj.match_ids(4));
    }
       //  STATE_DETECT = 0;  STATE_PREDICT = 1;
    if (obj.detect_state() == 0)
      text_label += "-DET";
    else
      text_label += "-PRE";
    drawText(pos, text_label, tmp_marker, 1.0, 1.0);

    // camera label info
    bool _has_camera = obj.has_camera_supplement();
    auto& camera_marker = params_.marker_list[i+params_.max_obj_size/2];
    std::string camera_label;
    if (_has_camera) {
      if (obj.match_ids(1) != -1) {
        camera_label += "C60_";
        camera_label += std::to_string(obj.match_ids(1));
      }
      if (obj.match_ids(5) != -1) {
        camera_label += "C30_";
        camera_label += std::to_string(obj.match_ids(5));
      }
      if (obj.match_ids(6) != -1) {
        camera_label += "C120r_";
        camera_label += std::to_string(obj.match_ids(6));
      }
      if (obj.match_ids(7) != -1) {
        camera_label += "C120f_";
        camera_label += std::to_string(obj.match_ids(7));
    }
    }
    pos.z() = center.z() + 1.2;
    drawText(pos, camera_label, camera_marker, 1.0, 1.0);
  }

  return params_.marker_list;
}
}  // namespace fusion
}  // namespace perception
