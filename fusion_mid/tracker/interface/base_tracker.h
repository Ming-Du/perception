#pragma once

#include <string>
#include <ros/ros.h>

#include "perception/fusion_mid/tracker/base/frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace mid_fusion {
struct TrackerOptions {};
class BaseTracker {
 public:
  BaseTracker() : name_("BaseTracker") {}
  virtual ~BaseTracker() = default;
  virtual bool Init() = 0;
  // @brief: tracking objects.
  // @param [in]: current object frame.
  // @param [in]: options.
  // @param [out]: current tracked objects frame.
  virtual bool Track(const mid_fusion::Frame &detected_frame,
                     const TrackerOptions &options,
                     mid_fusion::FramePtr tracked_frame) = 0;
  virtual std::string Name() { return name_; }

 protected:
  std::string name_;

};

PERCEPTION_REGISTER_REGISTERER(BaseTracker);
#define PERCEPTION_REGISTER_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTracker, name)
}  // namespace mid_fusion
}  // namespace perception
