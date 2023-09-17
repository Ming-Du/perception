#pragma once
#include "base_marker_pub.h"

namespace perception {
namespace fusion {
class TrajMarkerPub : public BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<TrajMarkerPub>;

  // TrajMarkerPub() = default;

  void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  virtual std::vector<ROS_VISUALIZATION_MARKER>& display(const TrackedObjects& tracked_objects) override;
  std::string name() override { return "TrajMarkerPub"; }
};

}  // namespace fusion
}  // namespace perception