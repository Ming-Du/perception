#pragma once

#include <memory>
#include <string>

#include "perception/fusion_mid/tracker/base/frame.h"
#include "perception/fusion_mid/tracker/interface/base_filter.h"

namespace perception {
namespace mid_fusion {

class MidFusionTrack {
 public:
  MidFusionTrack(const mid_fusion::ObjectPtr &obs, const double timestamp);
  ~MidFusionTrack() {}
  // update the object after association with a mid_fusion obervation
  void UpdataDetectedObs(const mid_fusion::ObjectPtr &obs_detected, const double timestamp);
  void SetObsNullptr();
  int GetObsId() const;
  mid_fusion::ObjectPtr GetDetectedObs();
  mid_fusion::ObjectPtr GetObs();
  double GetTimestamp();
  double GetTrackingTime();
  bool IsDead() { return is_dead_; }
  void SetDead() { is_dead_ = true; }
  bool ConfirmTrack() {
    ROS_DEBUG_STREAM("tracked_times: " << tracked_times_ << " s_tracked_times_threshold_: "
                                       << s_tracked_times_threshold_);
    return tracked_times_ > s_tracked_times_threshold_;
  }
  static void SetTrackedTimesThreshold(const int &threshold) {
    s_tracked_times_threshold_ = threshold;
  }
  static void SetChosenFilter(const std::string &chosen_filter) {
    s_chosen_filter_ = chosen_filter;
  }
  static void SetUseFilter(bool use_filter) { s_use_filter_ = use_filter; }

 private:
  double timestamp_ = 0.0;
  int obs_id_ = 0;
  int tracked_times_ = 0;
  double tracking_time_ = 0.0;
  bool is_dead_ = false;
  mid_fusion::ObjectPtr obs_detected_ = nullptr;  // observasion from mid_fusion
  mid_fusion::ObjectPtr obs_ = nullptr;        // track result after tracking
  std::shared_ptr<BaseFilter> filter_ = nullptr;

  static std::string s_chosen_filter_;
  static int s_current_idx_;
  static int s_tracked_times_threshold_;
  static bool s_use_filter_;

};

typedef std::shared_ptr<MidFusionTrack> MidFusionTrackPtr;

}  // namespace mid_fusion
}  // namespace perception
