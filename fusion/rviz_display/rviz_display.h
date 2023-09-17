#pragma once

#include <yaml-cpp/yaml.h>
#include "arrow_marker_pub.h"
#include "common/proto/localization.pb.h"
#include "lable_info_marker_pub.h"
#include "polygon_marker_pub.h"
#include "track_info_marker_pub.h"
#include "trajectory_marker_pub.h"
#include "object_marker_pub.h"

namespace perception {
namespace fusion {
class RvizDisplay {
 public:
  using Ptr = std::shared_ptr<RvizDisplay>;
  RvizDisplay(const ros::NodeHandlePtr& node_ptr);
  void display(const TrackedObjects& tracked_objects_pnc,
               const TrackedObjects& tracked_objects_app,
               const localization::Localization localization);

 private:
  std::string name() { return "RvizDisplay"; }

  ros::Publisher pub_perception_;
  ROS_VISUALIZATION_MARKERARRAY::Ptr fusion_marker_array_ptr_;
  LabelInfosMarkerPub::Ptr label_pub_, app_label_pub_;
  PolygonMarkerPub::Ptr polygon_pub_, app_polygon_pub_;
  TrackInfoPubMarker::Ptr track_pub_, app_track_pub_, loc_track_pub_;
  ArrowMarkerPub::Ptr arrow_pub_;
  ArrowMarkerPub::Ptr vel_direc_pub_;
  TrajMarkerPub::Ptr traj_pub_;
  ObjectMarkerPub::Ptr object_pub_;
};

}  // namespace fusion
}  // namespace perception
