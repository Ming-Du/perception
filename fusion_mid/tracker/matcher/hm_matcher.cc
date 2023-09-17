#include "perception/fusion_mid/tracker/matcher/hm_matcher.h"

#include <string>
#include <utility>

namespace perception {
namespace mid_fusion {


HMMatcher::HMMatcher() { name_ = "HMMatcher"; }

HMMatcher::~HMMatcher() {}

bool HMMatcher::Init() {
  std::string model_name = name_;
  ROS_INFO_STREAM("matcher name: " << name_);

  ROS_DEBUG_STREAM("Init: read config from: " << FLAGS_fusion_mid_config_manager_path);
  std::string config_file = FLAGS_fusion_mid_config_manager_path + "tracker_conf/hm_matcher_conf.pt";
  // get config params
  HmMatcherConf params;
  if (!::common::GetProtoFromFile(config_file, params)) {
    ROS_ERROR("Init: HMMatcher failed to load config file.");
    return false;
  }
  double max_match_distance = params.max_match_distance();
  double bound_match_distance = params.bound_match_distance();
  BaseMatcher::SetMaxMatchDistance(max_match_distance);
  BaseMatcher::SetBoundMatchDistance(bound_match_distance);
  return true;
}
// @brief match mid_fusion objects to tracks
// @params[IN] mid_fusion_tracks: global tracks
// @params[IN] mid_fusion_frame: current mid_fusion frame
// @params[IN] options: matcher options for future use
// @params[OUT] assignments: matched pair of tracks and measurements
// @params[OUT] unassigned_tracks: unmatched tracks
// @params[OUT] unassigned_objects: unmatched objects
// @return nothing
bool HMMatcher::Match(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                      const mid_fusion::Frame &detected_frame,
                      const TrackObjectMatcherOptions &options,
                      std::vector<TrackObjectPair> *assignments,
                      std::vector<size_t> *unassigned_tracks,
                      std::vector<size_t> *unassigned_objects) {
  IDMatch(mid_fusion_tracks, detected_frame, assignments, unassigned_tracks,
          unassigned_objects);
  TrackObjectPropertyMatch(mid_fusion_tracks, detected_frame, assignments,
                           unassigned_tracks, unassigned_objects);
  return true;
}

bool HMMatcher::RefinedTrack(const mid_fusion::ObjectPtr &track_object,
                             double track_timestamp,
                             const mid_fusion::ObjectPtr &mid_fusion_object,
                             double mid_fusion_timestamp) {
  double dist = 0.5 * DistanceBetweenObs(track_object, track_timestamp,
                                         mid_fusion_object, mid_fusion_timestamp) +
                0.5 * DistanceBetweenObs(mid_fusion_object, mid_fusion_timestamp,
                                         track_object, track_timestamp);

  return dist < BaseMatcher::GetMaxMatchDistance();
}

void HMMatcher::TrackObjectPropertyMatch(
    const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
    const mid_fusion::Frame &detected_frame, std::vector<TrackObjectPair> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  if (unassigned_tracks->empty() || unassigned_objects->empty()) {
    return;
  }
  std::vector<std::vector<double>> association_mat(unassigned_tracks->size());
  for (size_t i = 0; i < association_mat.size(); ++i) {
    association_mat[i].resize(unassigned_objects->size(), 0);
  }
  ComputeAssociationMat(mid_fusion_tracks, detected_frame, *unassigned_tracks,
                        *unassigned_objects, &association_mat);

  // from perception-common
  common::SecureMat<double> *global_costs =
      hungarian_matcher_.mutable_global_costs();
  global_costs->Resize(unassigned_tracks->size(), unassigned_objects->size());
  for (size_t i = 0; i < unassigned_tracks->size(); ++i) {
    for (size_t j = 0; j < unassigned_objects->size(); ++j) {
      (*global_costs)(i, j) = association_mat[i][j];
    }
  }
  std::vector<TrackObjectPair> property_assignments;
  std::vector<size_t> property_unassigned_tracks;
  std::vector<size_t> property_unassigned_objects;
  hungarian_matcher_.Match(
      BaseMatcher::GetMaxMatchDistance(), BaseMatcher::GetBoundMatchDistance(),
      common::GatedHungarianMatcher<double>::OptimizeFlag::OPTMIN,
      &property_assignments, &property_unassigned_tracks,
      &property_unassigned_objects);

  for (size_t i = 0; i < property_assignments.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_assignments[i].first);
    size_t go_idx = unassigned_objects->at(property_assignments[i].second);
    assignments->push_back(std::pair<size_t, size_t>(gt_idx, go_idx));
  }
  std::vector<size_t> temp_unassigned_tracks;
  std::vector<size_t> temp_unassigned_objects;
  for (size_t i = 0; i < property_unassigned_tracks.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_unassigned_tracks[i]);
    temp_unassigned_tracks.push_back(gt_idx);
  }
  for (size_t i = 0; i < property_unassigned_objects.size(); ++i) {
    size_t go_idx = unassigned_objects->at(property_unassigned_objects[i]);
    temp_unassigned_objects.push_back(go_idx);
  }
  *unassigned_tracks = temp_unassigned_tracks;
  *unassigned_objects = temp_unassigned_objects;
}
void HMMatcher::ComputeAssociationMat(
    const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
    const mid_fusion::Frame &detected_frame,
    const std::vector<size_t> &unassigned_tracks,
    const std::vector<size_t> &unassigned_objects,
    std::vector<std::vector<double>> *association_mat) {
  double frame_timestamp = detected_frame.timestamp;
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    for (size_t j = 0; j < unassigned_objects.size(); ++j) {
      const mid_fusion::ObjectPtr &track_object =
          mid_fusion_tracks[unassigned_tracks[i]]->GetObs();
      const mid_fusion::ObjectPtr &frame_object =
          detected_frame.objects[unassigned_objects[j]];
      double track_timestamp =
          mid_fusion_tracks[unassigned_tracks[i]]->GetTimestamp();
      double distance_forward = DistanceBetweenObs(
          track_object, track_timestamp, frame_object, frame_timestamp);
      double distance_backward = DistanceBetweenObs(
          frame_object, frame_timestamp, track_object, track_timestamp);
      association_mat->at(i).at(j) =
          0.5 * distance_forward + 0.5 * distance_backward;
    }
  }
}
double HMMatcher::DistanceBetweenObs(const mid_fusion::ObjectPtr &obs1,
                                     double timestamp1,
                                     const mid_fusion::ObjectPtr &obs2,
                                     double timestamp2) {
  double time_diff = timestamp2 - timestamp1;
  return (obs2->center - obs1->center -
          obs1->velocity.cast<double>() * time_diff)
      .head(2)
      .norm();
}

PERCEPTION_REGISTER_MATCHER(HMMatcher);

}  // namespace mid_fusion
}  // namespace perception
