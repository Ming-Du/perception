#include "trajectory_marker_pub.h"

namespace perception {
namespace fusion {

std::vector<ROS_VISUALIZATION_MARKER>& TrajMarkerPub::display(const TrackedObjects& tracked_objects) {

  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < 2 * obj_vec.size()) {
    params_.max_obj_size = 2 * obj_vec.size();
    params_.marker_list.resize(params_.max_obj_size);
  } else {
    for (size_t i = 2 * obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    auto& tmp_marker = params_.marker_list[i];
    auto& cube_marker = params_.marker_list[i+obj_vec.size()];
    tmp_marker.ns = "trajectory";
    tmp_marker.id = i;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_STRIP;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.1;

    tmp_marker.color.r = 1.0;
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 0.0;
    tmp_marker.color.a = 0.5f;

    cube_marker.ns = "trajectory";
    cube_marker.id = i + obj_vec.size();
    cube_marker.header.frame_id = options_.frame_id;
    cube_marker.type = ROS_VISUALIZATION_MARKER::CUBE_LIST;
    cube_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    cube_marker.scale.x = 0.2;
    cube_marker.scale.y = 0.2;
    cube_marker.scale.z = 0.2;

    cube_marker.color.r = 1.0;
    cube_marker.color.g = 1.0;
    cube_marker.color.b = 0.0;
    cube_marker.color.a = 0.5f;
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& obj = tmp_obj.obj();

    auto& tmp_marker = params_.marker_list[i];
    size_t p_size = obj.polygon_size();
    tmp_marker.points.reserve(p_size);
    tmp_marker.points.clear();

    auto& cube_marker = params_.marker_list[i+obj_vec.size()];
    cube_marker.points.reserve(p_size);
    cube_marker.points.clear();

    for (size_t i = 0; i < p_size; ++i) {
      geometry_msgs::Point pts;
      pts.x = obj.polygon(i).x();
      pts.y = obj.polygon(i).y();
      pts.z = obj.polygon(i).z();
      tmp_marker.points.emplace_back(pts);
      cube_marker.points.emplace_back(pts);
    }

  }

  return params_.marker_list;
}

}  // namespace fusion
}  // namespace perception
