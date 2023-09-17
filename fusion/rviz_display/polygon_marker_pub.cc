#include "polygon_marker_pub.h"

namespace perception {
namespace fusion {
std::vector<ROS_VISUALIZATION_MARKER>& PolygonMarkerPub::display(
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
    const auto& tmp_obj = obj_vec[i];
    auto& obj = tmp_obj.obj();
    tmp_marker.ns = "polygon";
    tmp_marker.id = i;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.1;
    tmp_marker.scale.y = 0.1;
    tmp_marker.scale.z = 0.1;
    tmp_marker.color.a = 0.5f;
    if (obj.motion_state() == 0) {  // static-white
      tmp_marker.color.r = 1.0;
      tmp_marker.color.g = 1.0;
      tmp_marker.color.b = 1.0;
    } else {  // dynamic-green
      tmp_marker.color.r = 0.0;
      tmp_marker.color.g = 1.0;
      tmp_marker.color.b = 0.0;
    }

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);
  }

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    const auto& tmp_obj = obj_vec[i];
    auto& obj = tmp_obj.obj();

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

    auto& tmp_marker = params_.marker_list[i];
    tmp_marker.points.reserve(p_size * 6);
    tmp_marker.points.clear();

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

  return params_.marker_list;
}
}  // namespace fusion
}  // namespace perception