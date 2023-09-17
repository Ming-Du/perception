#pragma once

#include <vector>

#include "perception/fusion_mid/tracker/base/frame.h"
#include "perception/fusion_mid/tracker/common/mid_fusion_track.h"

namespace perception {
namespace mid_fusion {

class MidFusionTrackManager {
 public:
  MidFusionTrackManager() = default;
  ~MidFusionTrackManager() = default;

  inline std::vector<MidFusionTrackPtr> &mutable_tracks() { return tracks_; }
  inline const std::vector<MidFusionTrackPtr> &GetTracks() const { return tracks_; }

  void AddTrack(const MidFusionTrackPtr &track) { tracks_.push_back(track); }
  int RemoveLostTracks();
  void ClearTracks();

 protected:
  std::vector<MidFusionTrackPtr> tracks_;

};

}  // namespace mid_fusion
}  // namespace perception
