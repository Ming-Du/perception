#include "polygon_marker_pub.h"

namespace perception {
namespace radar {

void trackMarkerPolygonPub(const perception::RadarObjects& radar_fusion,
                           Params& param,
                           localization::Localization local_current) {
  param.marker_list.clear();
  param.marker_list.resize(param.max_obj_size);                              
  if (param.max_obj_size < radar_fusion.objs_size()) {
    param.max_obj_size = radar_fusion.objs_size();
    param.marker_list.resize(param.max_obj_size);
  } else {
    for (size_t i = radar_fusion.objs_size(); i < param.max_obj_size; ++i) {
      visualization_msgs::Marker& tmp_marker = param.marker_list[i];
      tmp_marker.action = visualization_msgs::Marker::DELETE;
    }
  }

  for (size_t i = 0; i < param.marker_list.size(); ++i) {
    visualization_msgs::Marker& tmp_marker = param.marker_list[i];
    tmp_marker.header.frame_id = "base_link";
    tmp_marker.ns = "polygon";
    tmp_marker.action = visualization_msgs::Marker::ADD;
    tmp_marker.id = i;
    tmp_marker.scale.x = 0.1;
    tmp_marker.scale.y = 0.1;
    tmp_marker.scale.z = 0.1;
    tmp_marker.color.r = 1.0;
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 0.0;
    tmp_marker.color.a = 0.5f;
    tmp_marker.pose.orientation.w = 1.0;
    tmp_marker.pose.position.x = 0;
    tmp_marker.pose.position.y = 0;
    tmp_marker.pose.position.z = 0;
    tmp_marker.type = visualization_msgs::Marker::LINE_LIST;
  }

  for (size_t i = 0; i < radar_fusion.objs_size(); ++i) {
    perception::Object object_ = radar_fusion.objs(i).obj();
    std::vector<geometry_msgs::Point> cub_points;
    size_t p_size = object_.contour_size();
    double p_low = 0;
    double p_high = object_.size().z();
    for (size_t p = 0; p < p_size; ++p) {
      geometry_msgs::Point pts;
      pts.x = object_.contour(p).x();
      pts.y = object_.contour(p).y();
      pts.z = p_low;
      cub_points.emplace_back(pts);
    }
    for (size_t p = 0; p < p_size; ++p) {
      geometry_msgs::Point pts;
      pts.x = object_.contour(p).x();
      pts.y = object_.contour(p).y();
      pts.z = p_high;           
      cub_points.emplace_back(pts);
    }

    visualization_msgs::Marker& tmp_marker = param.marker_list[i];
    tmp_marker.points.reserve(p_size * 6);
    tmp_marker.points.clear();

    for (size_t p = 0; p < p_size; ++p) {
      size_t next = p + 1;
      next = next < p_size ? next : 0;
      tmp_marker.points.emplace_back(cub_points[p]);
      tmp_marker.points.emplace_back(cub_points[next]);
    }
    for (size_t p = 0; p < p_size; ++p) {
      size_t next = p + 1;
      next = next < p_size ? next : 0;
      tmp_marker.points.emplace_back(cub_points[p + p_size]);
      tmp_marker.points.emplace_back(cub_points[next + p_size]);
    }

    for (size_t p = 0; p < p_size; ++p) {
      tmp_marker.points.emplace_back(cub_points[p]);
      tmp_marker.points.emplace_back(cub_points[p + p_size]);
    }
  }

}

}  // namespace radar
}  // namespace perception
