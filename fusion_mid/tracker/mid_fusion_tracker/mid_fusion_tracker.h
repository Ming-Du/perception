#pragma once

#include <string>
#include <vector>

#include "perception/fusion_mid/tracker/interface/base_tracker.h"
#include "perception/fusion_mid/tracker/common/mid_fusion_track_manager.h"
#include "perception/fusion_mid/tracker/matcher/hm_matcher.h"
#include "perception/fusion_mid/proto/fusion_mid_tracker.pb.h"
#include "perception/base/perception_gflags.h"
#include "common/include/pb_utils.h"

namespace perception {
namespace mid_fusion {

class MidFusionTracker : public BaseTracker {
 public:
  using Ptr = std::shared_ptr<MidFusionTracker>;
  MidFusionTracker();
  virtual ~MidFusionTracker();
  bool Init() override;
  bool Track(const mid_fusion::Frame &detected_frame, const TrackerOptions &options,
             mid_fusion::FramePtr tracked_frame) override;

 private:
  std::string matcher_name_;
  mid_fusion::BaseMatcher *matcher_ = nullptr;
  MidFusionTrackManager *track_manager_ = nullptr;
  static double s_tracking_time_win_;
  void TrackObjects(const mid_fusion::Frame &detected_frame);
  void UpdateAssignedTracks(const mid_fusion::Frame &detected_frame,
                            std::vector<TrackObjectPair> assignments);
  void UpdateUnassignedTracks(const mid_fusion::Frame &detected_frame,
                              const std::vector<size_t> &unassigned_tracks);
  void DeleteLostTracks();
  void CreateNewTracks(const mid_fusion::Frame &detected_frame,
                       const std::vector<size_t> &unassigned_objects);
  void CollectTrackedFrame(mid_fusion::FramePtr tracked_frame);

};
}  // namespace mid_fusion
}  // namespace perception
