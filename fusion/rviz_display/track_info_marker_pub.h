#pragma once
#include "base_marker_pub.h"
#include "common/proto/localization.pb.h"

namespace perception {
namespace fusion {
class TrackInfoPubMarker : public BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<TrackInfoPubMarker>;

  TrackInfoPubMarker() = default;

  virtual void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  virtual std::vector<ROS_VISUALIZATION_MARKER>& display(
      const TrackedObjects& tracked_objects) override;

  std::vector<ROS_VISUALIZATION_MARKER>& display_ego_text(
      const localization::Localization localization);

  virtual std::string name() override { return "TrackInfoPubMarker"; }
};

}  // namespace fusion
}  // namespace perception
