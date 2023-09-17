#include "rviz_display.h"

namespace perception {
namespace fusion {
RvizDisplay::RvizDisplay(const ros::NodeHandlePtr& node_ptr) {
  // init markers pub
  MarkerPubOptions options;
  options.pre_fix = "";
  options.frame_id = "base_link";
  options.node_ptr = node_ptr;

  label_pub_.reset(new LabelInfosMarkerPub);
  label_pub_->init(options);

  polygon_pub_.reset(new PolygonMarkerPub);
  polygon_pub_->init(options);

  track_pub_.reset(new TrackInfoPubMarker);
  track_pub_->init(options);

  arrow_pub_.reset(new ArrowMarkerPub);
  arrow_pub_->init(options);

  vel_direc_pub_.reset(new ArrowMarkerPub);
  vel_direc_pub_->init(options);

  traj_pub_.reset(new TrajMarkerPub);
  traj_pub_->init(options);

  app_label_pub_.reset(new LabelInfosMarkerPub);
  app_label_pub_->init(options);

  app_polygon_pub_.reset(new PolygonMarkerPub);
  app_polygon_pub_->init(options);

  app_track_pub_.reset(new TrackInfoPubMarker);
  app_track_pub_->init(options);

  loc_track_pub_.reset(new TrackInfoPubMarker);
  loc_track_pub_->init(options);
#if USE_DEBUG_FUSION
  //display object for debug by duming
  object_pub_.reset(new ObjectMarkerPub);
  object_pub_->init(options);
#endif

  fusion_marker_array_ptr_.reset(new ROS_VISUALIZATION_MARKERARRAY);
  pub_perception_ =
      node_ptr->advertise<ROS_VISUALIZATION_MARKERARRAY>(options.pre_fix + "fusion_viz", 1, true);
}
void RvizDisplay::display(const TrackedObjects& tracked_objects_pnc,
                          const TrackedObjects& tracked_objects_app,
                          const localization::Localization localization) {
  if (tracked_objects_pnc.objs_size() == 0) {
    return;
  }
  fusion_marker_array_ptr_->markers.clear();

  /* pnc */
  // track
  auto marker_list_track = track_pub_->display(tracked_objects_pnc);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_track.size());
  for (const auto& tmp_marker : marker_list_track) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  auto marker_list_track_loc = loc_track_pub_->display_ego_text(localization);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_track_loc.size());
  for (const auto& tmp_marker : marker_list_track_loc) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  // label
  auto marker_list_label = label_pub_->display(tracked_objects_pnc);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_label.size());
  for (const auto& tmp_marker : marker_list_label) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  // polygon
  auto marker_list_polygon = polygon_pub_->display(tracked_objects_pnc);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_polygon.size());
  for (const auto& tmp_marker : marker_list_polygon) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  // arrow
  auto marker_list_arrow = arrow_pub_->display(tracked_objects_pnc);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_arrow.size());
  for (const auto& tmp_marker : marker_list_arrow) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  // vel direc
  auto marker_list_vel = vel_direc_pub_->display_app(tracked_objects_pnc, localization, "vel_direc", false);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_vel.size());
  for (const auto& tmp_marker : marker_list_vel) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

  // trajectory
  auto marker_list_traj = traj_pub_->display(tracked_objects_pnc);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                            marker_list_traj.size());
  for (const auto& tmp_marker : marker_list_traj) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }

#if USE_DEBUG_FUSION
  //objects one by one to publish
  auto marker_list_objects = object_pub_->display_app(tracked_objects_pnc, localization);
  fusion_marker_array_ptr_->markers.reserve(fusion_marker_array_ptr_->markers.size() +
                                                marker_list_objects.size());
  for (const auto& tmp_marker : marker_list_objects) {
    fusion_marker_array_ptr_->markers.emplace_back(tmp_marker);
  }
#endif
  pub_perception_.publish(*fusion_marker_array_ptr_);
}

}  // namespace fusion
}  // namespace perception
