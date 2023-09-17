#include "lable_local_info_marker_pub.h"

namespace perception {
namespace radar {


void trackMarkerLocalPub(const perception::RadarObjects& radar_fusion,
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
    tmp_marker.ns = "track_info_local";
    tmp_marker.action = visualization_msgs::Marker::ADD;
    tmp_marker.id = i;
    tmp_marker.scale.z = 1.0;
    tmp_marker.color.r = 1.0;
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 0.0;
    tmp_marker.color.a = 1.0;
    tmp_marker.pose.orientation.w = 1.0;
    tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  }
  
  for (size_t i = 0; i < radar_fusion.objs_size(); ++i) {
    visualization_msgs::Marker& tmp_marker = param.marker_list[i];
    perception::Object object_ = radar_fusion.objs(i).obj();
    tmp_marker.pose.position.x = object_.center().x();
    tmp_marker.pose.position.y = object_.center().y();
    tmp_marker.pose.position.z = 0.7f;
    tmp_marker.text.clear();
    tmp_marker.text = NumToStr(object_.velocity().x() * 3.6, 1) + "+" 
                    + NumToStr(object_.velocity().y() * 3.6, 1) + "km/h - host_speed";
  }
}


}  // namespace radar
}  // namespace perception
