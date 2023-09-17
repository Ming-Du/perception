#pragma once
#include "base_marker_pub.h"

namespace perception {
namespace fusion {
class LabelInfosMarkerPub : public BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<LabelInfosMarkerPub>;

  LabelInfosMarkerPub() = default;

  void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  std::vector<ROS_VISUALIZATION_MARKER>& display(const TrackedObjects& tracked_objects) override;
  std::string name() override { return "LabelInfosMarkerPub"; }
};

}  // namespace fusion
}  // namespace perception