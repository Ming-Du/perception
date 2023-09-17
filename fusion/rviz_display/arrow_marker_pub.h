#pragma once
#include "base_marker_pub.h"
#include "common/proto/localization.pb.h"

namespace perception {
namespace fusion {
class ArrowMarkerPub : public BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<ArrowMarkerPub>;

  ArrowMarkerPub() = default;

  void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  std::vector<ROS_VISUALIZATION_MARKER>& display(const TrackedObjects& tracked_objects) override;
  std::vector<ROS_VISUALIZATION_MARKER>& display_app(const TrackedObjects& tracked_objects, const localization::Localization localization, std::string ns = "app_arrow", bool is_arrow = true);
  std::string name() override { return "ArrowMarkerPub"; }

 private:
  void drawArrow(const TrackedObject& obj, ROS_VISUALIZATION_MARKER& marker, bool is_arrow, double alpha = 1.);
  Eigen::Quaterniond RPYToQuaterniond(const float& roll = 0.0,
                                      const float& pitch = 0.0,
                                      const float& yaw = 0.0);
  
  localization::Localization localization_;
};

}  // namespace fusion
}  // namespace perception