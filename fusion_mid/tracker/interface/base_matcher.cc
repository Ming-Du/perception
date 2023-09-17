#include "perception/fusion_mid/tracker/interface/base_matcher.h"
#include <numeric>

namespace perception {
namespace mid_fusion {

double BaseMatcher::s_max_match_distance_ = 2.5;
double BaseMatcher::s_bound_match_distance_ = 10.0;

void BaseMatcher::SetMaxMatchDistance(double dist) {
  s_max_match_distance_ = dist;
}

double BaseMatcher::GetMaxMatchDistance() { return s_max_match_distance_; }

void BaseMatcher::SetBoundMatchDistance(double dist) {
  s_bound_match_distance_ = dist;
}

double BaseMatcher::GetBoundMatchDistance() { return s_bound_match_distance_; }

void BaseMatcher::IDMatch(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                          const mid_fusion::Frame &detected_frame,
                          std::vector<TrackObjectPair> *assignments,
                          std::vector<size_t> *unassigned_tracks,
                          std::vector<size_t> *unassigned_objects) {
  size_t num_track = mid_fusion_tracks.size();
  const auto &objects = detected_frame.objects;
  double object_timestamp = detected_frame.timestamp;
  size_t num_obj = objects.size();
  if (num_track == 0 || num_obj == 0) {
    unassigned_tracks->resize(num_track);
    unassigned_objects->resize(num_obj);
    std::iota(unassigned_tracks->begin(), unassigned_tracks->end(), 0);
    std::iota(unassigned_objects->begin(), unassigned_objects->end(), 0);
    return;
  }
  std::vector<bool> track_used(num_track, false);
  std::vector<bool> object_used(num_obj, false);
  for (size_t i = 0; i < num_track; ++i) {
    const auto &track_object = mid_fusion_tracks[i]->GetDetectedObs();
    double track_timestamp = mid_fusion_tracks[i]->GetTimestamp();
    if (track_object.get() == nullptr) {
      ROS_ERROR_STREAM("track_object is not available");
      continue;
    }
    int track_object_track_id = track_object->track_id;
    for (size_t j = 0; j < num_obj; ++j) {
      int object_track_id = objects[j]->track_id;
      if (track_object_track_id == object_track_id &&
          RefinedTrack(track_object, track_timestamp, objects[j],
                       object_timestamp)) {
        assignments->push_back(std::pair<size_t, size_t>(i, j));
        track_used[i] = true;
        object_used[j] = true;
      }
    }
  }
  for (size_t i = 0; i < track_used.size(); ++i) {
    if (!track_used[i]) {
      unassigned_tracks->push_back(i);
    }
  }
  for (size_t i = 0; i < object_used.size(); ++i) {
    if (!object_used[i]) {
      unassigned_objects->push_back(i);
    }
  }
}

bool BaseMatcher::RefinedTrack(const mid_fusion::ObjectPtr &track_object,
                               double track_timestamp,
                               const mid_fusion::ObjectPtr &mid_fusion_object,
                               double mid_fusion_timestamp) {
  // This function is supposed to return true in the base class.
  // Specific actions can be overrided in derived classes.
  return true;
}

}  // namespace mid_fusion
}  // namespace perception
