#include "perception/fusion_mid/tracker/common/mid_fusion_track_manager.h"

namespace perception {
namespace mid_fusion {

int MidFusionTrackManager::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      ++track_count;
    }
  }
  int removed_count = static_cast<int>(tracks_.size() - track_count);
  ROS_DEBUG_STREAM("Remove " << removed_count << " tracks");
  tracks_.resize(track_count);
  return static_cast<int>(track_count);
}

void MidFusionTrackManager::ClearTracks() { tracks_.clear(); }

}  // namespace mid_fusion
}  // namespace perception
