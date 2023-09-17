#include "rviz_display.h"


namespace perception {
namespace radar {

void RadarFusionDisplay(const perception::RadarObjects& radar_fusion,
                        ros::Publisher& publisher,
                        Params& param,
                        localization::Localization local_current) {

  param.radar_marker_array_ptr.markers.clear();

  trackMarkerLocalPub(radar_fusion, param, local_current);
  param.radar_marker_array_ptr.markers.reserve(param.radar_marker_array_ptr.markers.size() +
                                               param.marker_list.size());
  for (const auto& tmp_marker : param.marker_list) {
    param.radar_marker_array_ptr.markers.emplace_back(tmp_marker);
  }

  trackMarkerGlobalPub(radar_fusion, param, local_current);
  param.radar_marker_array_ptr.markers.reserve(param.radar_marker_array_ptr.markers.size() +
                                               param.marker_list.size());
  for (const auto& tmp_marker : param.marker_list) {
    param.radar_marker_array_ptr.markers.emplace_back(tmp_marker);
  }

  trackMarkerPolygonPub(radar_fusion, param, local_current);
  param.radar_marker_array_ptr.markers.reserve(param.radar_marker_array_ptr.markers.size() +
                                               param.marker_list.size());
  for (const auto& tmp_marker : param.marker_list) {
    param.radar_marker_array_ptr.markers.emplace_back(tmp_marker);
  }

  publisher.publish(param.radar_marker_array_ptr);
}


}  // namespace radar
}  // namespace perception
