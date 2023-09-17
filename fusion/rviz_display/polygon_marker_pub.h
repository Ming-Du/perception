#pragma once
#include "base_marker_pub.h"

namespace perception {
namespace fusion {
class PolygonMarkerPub : public BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<PolygonMarkerPub>;

  PolygonMarkerPub() = default;

  virtual void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  virtual std::vector<ROS_VISUALIZATION_MARKER>& display(
      const TrackedObjects& tracked_objects) override;
  virtual std::string name() override { return "PolygonMarkerPub"; }
};

}  // namespace fusion
}  // namespace perception