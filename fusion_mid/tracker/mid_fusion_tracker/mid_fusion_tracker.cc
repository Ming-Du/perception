#include "perception/fusion_mid/tracker/mid_fusion_tracker/mid_fusion_tracker.h"

namespace perception {
namespace mid_fusion {

double MidFusionTracker::s_tracking_time_win_ = 0.06;
MidFusionTracker::MidFusionTracker()
    : BaseTracker(), matcher_(nullptr), track_manager_(nullptr) {
  name_ = "MidFusionTracker";
}

MidFusionTracker::~MidFusionTracker() {
  if (matcher_ != nullptr) {
    delete matcher_;
  }
  if (track_manager_ != nullptr) {
    delete track_manager_;
  }
}

bool MidFusionTracker::Init() {
  std::string model_name = name_;
  std::string chosen_filter;
  int tracked_times_threshold;
  bool use_filter;
  MidFusionTrackerConf params;
  ROS_DEBUG_STREAM("Init: read config from: " << FLAGS_fusion_mid_config_manager_path);
  std::string config_file = FLAGS_fusion_mid_config_manager_path + "tracker_conf/fusion_mid_tracker.pt";
  if (!::common::GetProtoFromFile(config_file, params)) {
    ROS_ERROR("Init: MidFusionTracker failed to load config file.");
    return false;
  }
  model_name = params.tracker_name();
  s_tracking_time_win_ = params.tracking_time_window();
  matcher_name_ = params.matcher_name();
  chosen_filter = params.chosen_filter();
  tracked_times_threshold = params.tracked_times_threshold();
  use_filter = params.use_filter();

  MidFusionTrack::SetChosenFilter(chosen_filter);
  MidFusionTrack::SetTrackedTimesThreshold(tracked_times_threshold);
  MidFusionTrack::SetUseFilter(use_filter);
  bool state = true;
  // Or use register class instead.
  if (matcher_name_ == "HMMatcher") {
    matcher_ = new HMMatcher();
    matcher_->Init();  //  use proto later
  } else {
    ROS_ERROR_STREAM("Not supported matcher : " << matcher_name_);
    state = false;
  }

  track_manager_ = new MidFusionTrackManager();
  if(!(track_manager_ != nullptr)) {
    ROS_WARN_STREAM("Failed to get MidFusionTrackManager instance.");
    state = false;
  }

  return state;
}

bool MidFusionTracker::Track(const mid_fusion::Frame &detected_frame,
                            const TrackerOptions &options,
                            mid_fusion::FramePtr tracked_frame) {
  TrackObjects(detected_frame);
  CollectTrackedFrame(tracked_frame);
  return true;
}

void MidFusionTracker::TrackObjects(const mid_fusion::Frame &detected_frame) {
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  TrackObjectMatcherOptions matcher_options;  
  const auto &mid_fusion_tracks = track_manager_->GetTracks();
  matcher_->Match(mid_fusion_tracks, detected_frame, matcher_options, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  UpdateAssignedTracks(detected_frame, assignments);
  UpdateUnassignedTracks(detected_frame, unassigned_tracks);
  DeleteLostTracks();
  CreateNewTracks(detected_frame, unassigned_objects);
}

void MidFusionTracker::UpdateAssignedTracks(
    const mid_fusion::Frame &detected_frame, std::vector<TrackObjectPair> assignments) {
  auto &mid_fusion_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < assignments.size(); ++i) {
    mid_fusion_tracks[assignments[i].first]->UpdataDetectedObs(
        detected_frame.objects[assignments[i].second], detected_frame.timestamp);
  }
}

void MidFusionTracker::UpdateUnassignedTracks(
    const mid_fusion::Frame &detected_frame,
    const std::vector<size_t> &unassigned_tracks) {
  double timestamp = detected_frame.timestamp;
  auto &mid_fusion_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    if (mid_fusion_tracks[unassigned_tracks[i]]->GetObs() != nullptr) {
      double mid_fusion_time = mid_fusion_tracks[unassigned_tracks[i]]->GetTimestamp();
      double time_diff = fabs(timestamp - mid_fusion_time);
      if (time_diff > s_tracking_time_win_) {
        mid_fusion_tracks[unassigned_tracks[i]]->SetDead();
      }
    } else {
      mid_fusion_tracks[unassigned_tracks[i]]->SetDead();
    }
  }
}

void MidFusionTracker::DeleteLostTracks() { track_manager_->RemoveLostTracks(); }

void MidFusionTracker::CreateNewTracks(
    const mid_fusion::Frame &detected_frame,
    const std::vector<size_t> &unassigned_objects) {
  for (size_t i = 0; i < unassigned_objects.size(); ++i) {
    MidFusionTrackPtr mid_fusion_track;
    mid_fusion_track.reset(new MidFusionTrack(detected_frame.objects[unassigned_objects[i]],
                                     detected_frame.timestamp));
    track_manager_->AddTrack(mid_fusion_track);
  }
}

void MidFusionTracker::CollectTrackedFrame(mid_fusion::FramePtr tracked_frame) {
  if (tracked_frame == nullptr) {
    ROS_ERROR_STREAM("tracked_frame is nullptr");
    return;
  }
  auto &objects = tracked_frame->objects;
  const auto &mid_fusion_tracks = track_manager_->GetTracks();
  ROS_DEBUG_STREAM("CollectTrackedFrame: tracks_size " << mid_fusion_tracks.size());
  for (size_t i = 0; i < mid_fusion_tracks.size(); ++i) {
    if (mid_fusion_tracks[i]->ConfirmTrack()) {
      mid_fusion::ObjectPtr object = mid_fusion::ObjectPtr(new mid_fusion::Object());
      const mid_fusion::ObjectPtr &track_object = mid_fusion_tracks[i]->GetObs();
      *object = *track_object;
      object->tracking_time = mid_fusion_tracks[i]->GetTrackingTime();
      object->track_id = mid_fusion_tracks[i]->GetObsId();
      object->latest_tracked_time = mid_fusion_tracks[i]->GetTimestamp();
      objects.push_back(object);
    }
  }
}

PERCEPTION_REGISTER_TRACKER(MidFusionTracker);

}  // namespace mid_fusion
}  // namespace perception
