#include "probabilistic_fusion.h"

#include <map>
#include <utility>

#include "base/base_init_options.h"
#include "base/configmanager.h"
#include "base/object_pool_types.h"
#include "base/track_pool_types.h"
#include "common/file.h"
#include "lib/data_association/hm_data_association/hm_tracks_objects_match.h"
#include "lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"
#include "lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "lib/data_fusion/type_fusion/bayes_type_fusion/bayes_type_fusion.h"
#include "lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"
#include "perception/common/include/perf_util.h"
#include "perception/common/include/string_util.h"
#include "perception/lib/config_manager/config_manager.h"
#include "common/proto/probabilistic_fusion_config.pb.h"

namespace perception {
namespace fusion {

using perception::fusion::GetAbsolutePath;

ProbabilisticFusion::ProbabilisticFusion() {}

ProbabilisticFusion::~ProbabilisticFusion() {}

bool ProbabilisticFusion::Init(const FusionInitOptions& init_options) {
  main_sensors_ = init_options.main_sensors;

  BaseInitOptions options;
  if (!GetFusionInitOptions("ProbabilisticFusion", &options)) {
    return false;
  }

  std::string work_root_config =
      GetAbsolutePath(lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(work_root_config, options.conf_file);
  ProbabilisticFusionConfig params;
  if (!perception::fusion::GetProtoFromFile(config, &params)) {
    ROS_ERROR_STREAM("Init: Read config failed: " << config);
    return false;
  }
  params_.use_lidar = params.use_lidar();
  params_.use_radar = params.use_radar();
  params_.use_camera = params.use_camera();
  // Modify @jiangnan add falcon lidar
  params_.use_falcon_lidar = params.use_falcon_lidar();
  // Modify(@liuxinyu): obu_test
  params_.use_obu = params.use_obu();
  params_.use_vidar = params.use_vidar();
  params_.tracker_method = params.tracker_method();
  params_.data_association_method = params.data_association_method();
  params_.gate_keeper_method = params.gate_keeper_method();
  for (int i = 0; i < params.prohibition_sensors_size(); ++i) {
    params_.prohibition_sensors.push_back(params.prohibition_sensors(i));
  }

  // static member initialization from PB config
  Track::SetMaxLidarInvisiblePeriod(params.max_lidar_invisible_period());
  Track::SetMaxRadarInvisiblePeriod(params.max_radar_invisible_period());
  Track::SetMaxCameraInvisiblePeriod(params.max_camera_invisible_period());
  // Modify @jiangnan add falcon lidar
  Track::SetMaxFalconLidarInvisiblePeriod(params.max_falcon_lidar_invisible_period());
  Track::SetMaxVidarInvisiblePeriod(params.max_vidar_invisible_period());
  // Modify(@liuxinyu): obu_test
  Track::SetMaxObuInvisiblePeriod(params.max_obu_invisible_period());
  Sensor::SetMaxCachedFrameNumber(params.max_cached_frame_num());
  //@lijian，加载自定义配置文件
  bool state = ConfigManager::Instance()->Init(FLAGS_work_root);

  scenes_.reset(new Scene());
  if (params_.data_association_method == "HMAssociation") {
    matcher_.reset(new HMTrackersObjectsAssociation());
  } else {
    ROS_ERROR_STREAM("Unknown association method: " << params_.data_association_method);
    return false;
  }
  if (!matcher_->Init()) {
    ROS_ERROR_STREAM("Failed to init matcher.");
    return false;
  }

  if (params_.gate_keeper_method == "PbfGatekeeper") {
    gate_keeper_.reset(new PbfGatekeeper());
  } else {
    ROS_ERROR_STREAM("Unknown gate keeper method: " << params_.gate_keeper_method);
    return false;
  }
  if (!gate_keeper_->Init()) {
    ROS_ERROR("Failed to init gatekeeper.");
    return false;
  }
  //  std::string cv_config_path = std::string(PROJECT_PATH) + std::string("/config/filter");
  std::string cv_config_path = FLAGS_work_root + std::string("/filter");
  state = DstTypeFusion::Init() && DstExistenceFusion::Init() && PbfTracker::InitParams() &&
               KalmanConfig::InitParams(cv_config_path.c_str()) &&
               UnscentedKalmanConfig::InitParams(cv_config_path.c_str()) &&
               ExtendedKalmanConfig::InitParams(cv_config_path.c_str()) &&
               BayesTypeFusion::Init();

  return state;
}

bool ProbabilisticFusion::Fuse(const FusionOptions& options,
                               const fusion::FrameConstPtr& sensor_frame,
                               std::vector<fusion::ObjectPtr>* fused_objects) {
  if (fused_objects == nullptr) {
    ROS_ERROR("Fuse: fusion error: fused_objects is nullptr!");
    return false;
  }

  auto* sensor_data_manager = SensorDataManager::Instance();

  std::lock_guard<std::mutex> data_lock(data_mutex_);
  if (!params_.use_lidar && sensor_data_manager->IsLidar(sensor_frame)) {
    return true;
  }
  if (!params_.use_radar && sensor_data_manager->IsRadar(sensor_frame)) {
    return true;
  }
  if (!params_.use_camera && sensor_data_manager->IsCamera(sensor_frame)) {
    return true;
  }
  if (!params_.use_obu && sensor_data_manager->IsObu(sensor_frame)) {
    return true;
  }
  if (!params_.use_vidar && sensor_data_manager->IsVidar(sensor_frame)) {
    return true;
  }
  if (!params_.use_falcon_lidar && sensor_data_manager->IsFalconLidar(sensor_frame)) {
    return true;
  }

  bool is_publish_sensor = this->IsPublishSensor(sensor_frame);

  if (is_publish_sensor)
    started_ = true;

  if (started_) {
    sensor_data_manager->AddSensorMeasurements(sensor_frame);
  }

  if (!is_publish_sensor)
    return true;

  // 2. query related sensor_frames for fusion
  std::lock_guard<std::mutex> fuse_lock(fuse_mutex_);
  double fusion_time = sensor_frame->timestamp;
  std::vector<SensorFramePtr> frames;
  sensor_data_manager->GetLatestFrames(fusion_time, &frames);
  ROS_DEBUG_STREAM("Fuse: Get " << frames.size() << " related frames for fusion");
  // 3. perform fusion on related frames
  for (const auto& frame : frames) {

    FuseFrame(frame);
  }



  // 4. collect fused objects
  CollectFusedObjects(fusion_time, fused_objects);

  return true;
}

std::string ProbabilisticFusion::Name() const {
  return "ProbabilisticFusion";
}

bool ProbabilisticFusion::IsPublishSensor(const fusion::FrameConstPtr& sensor_frame) const {
  for (auto main_sensor : main_sensors_)
    ROS_DEBUG_STREAM("IsPublishSensor: main_sensor=" << main_sensor);

  std::string sensor_id = sensor_frame->sensor_info.name();
  const auto& itr = std::find(main_sensors_.begin(), main_sensors_.end(), sensor_id);
  if (itr != main_sensors_.end())
    return true;
  else
    return false;
}

void ProbabilisticFusion::RemoveDevidedRadarTrack() {
  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
  if (foreground_tracks.size() == 0) {
    return;
  }
  Eigen::Vector3d diff_pos;
  Eigen::Vector3f diff_vel;
  for (int i = 0; i < foreground_tracks.size() - 1; i++) {
    TrackPtr track = foreground_tracks[i];
    // ROS_INFO_STREAM("00000000000 track_id:"<<track->GetTrackId());
    if (!track->IsAlive()) {
      continue;
    }
    int track_id = track->GetTrackId();
    auto fused_polygons = track->GetFusedObject()->GetBaseObject()->polygon_utm;
    // Modify by zhaokai
    if (fused_polygons.size() == 0) {
      continue;
    }
    auto fused_center = track->GetFusedObject()->GetBaseObject()->position;
    auto fused_velocity = track->GetFusedObject()->GetBaseObject()->velocity;
    bool is_only_radar_track = IsRearOnlyRadarOrCameraTrack(track);
    // ROS_INFO_STREAM("00000000000 track_id:"<<track->GetTrackId());
    for (int j = i + 1; j < foreground_tracks.size(); j++) {
      TrackPtr track2 = foreground_tracks[j];
      if (!track2->IsAlive()) {
        continue;
      }
      int track2_id = track2->GetTrackId();
      auto fused2_polygons = track2->GetFusedObject()->GetBaseObject()->polygon_utm;
      if (fused2_polygons.size() == 0) {
        continue;
      }
      auto fused_center2 = track2->GetFusedObject()->GetBaseObject()->position;
      auto fused_velocity2 = track2->GetFusedObject()->GetBaseObject()->velocity;
      bool is_only_radar_track2 = IsRearOnlyRadarOrCameraTrack(track2);

      diff_pos = fused_center2 - fused_center;
      float distance =
          static_cast<float>(std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
      if (distance > s_dis_overlap_filter_threshold_)
        continue;

      diff_vel = fused_velocity2 - fused_velocity;
      float velocity_diff = static_cast<float>(std::sqrt(diff_vel.head(2).cwiseProduct(diff_vel.head(2)).sum()));
      if (is_only_radar_track && !is_only_radar_track2) {
        if (track2->GetFusedObject()->GetBaseObject()->is_static && velocity_diff > 2) {
          continue;//防止radar的track跟路边静止车辆匹配上
        }
        distance = ComputeRadarTo3DboxDis(track, track2);
        if (distance < 1.5 ||(distance < 2.5 && velocity_diff < s_vel_overlap_filter_threshold_)) {
          track->SetAlive(false);
          // ROS_INFO_STREAM("111 track_id:"<<track->GetTrackId()<<" track_id2:"<<track2->GetTrackId()<<"distance:"<<distance<<" vel:"<<velocity_diff);
          break;
        }
      } else if (!is_only_radar_track && is_only_radar_track2) {
        if (track->GetFusedObject()->GetBaseObject()->is_static && velocity_diff > 2) {
          continue;
        }
        distance = ComputeRadarTo3DboxDis(track2, track);
        if (distance < 1.5 ||(distance < 2.5 && velocity_diff < s_vel_overlap_filter_threshold_)) {
          track2->SetAlive(false);//这里不break
          // ROS_INFO_STREAM("222 track_id:"<<track->GetTrackId()<<" track_id2:"<<track2->GetTrackId()<<"distance:"<<distance<<" vel:"<<velocity_diff);
        }
      }
    }
  }
}

float ProbabilisticFusion::ComputeRadarTo3DboxDis(TrackPtr track_radar, TrackPtr track_3Dbox) {
  float distance = s_dis_overlap_filter_threshold_;
  float distance_temp = s_dis_overlap_filter_threshold_;

  fusion::PointD PointO;
  PointO.x = track_radar->GetFusedObject()->GetBaseObject()->position[0];
  PointO.y = track_radar->GetFusedObject()->GetBaseObject()->position[1];
  PointO.z = 0.0;

  TrackObjectDistance track_obj_dis;
  fusion::ObjectPtr objPtr = track_3Dbox->GetFusedObject()->GetBaseObject();
  for (size_t idx = 0; idx < objPtr->polygon.size(); ++idx) {
    if (idx == objPtr->polygon_utm.size() - 1) {
      fusion::PointD PointA, PointB;
      PointA.x = objPtr->polygon_utm.at(idx).x;
      PointA.y = objPtr->polygon_utm.at(idx).y;
      PointB.x = objPtr->polygon_utm.at(0).x;
      PointB.y = objPtr->polygon_utm.at(0).y;
      distance_temp = track_obj_dis.ComputerPointLinesegmentDistance(PointA, PointB, PointO);
    } else {
      fusion::PointD PointA, PointB;
      PointA.x = objPtr->polygon_utm.at(idx).x;
      PointA.y = objPtr->polygon_utm.at(idx).y;
      PointB.x = objPtr->polygon_utm.at(idx + 1).x;
      PointB.y = objPtr->polygon_utm.at(idx + 1).y;
      distance_temp = track_obj_dis.ComputerPointLinesegmentDistance(PointA, PointB, PointO);
    }
    if (distance_temp < distance)
      distance = distance_temp;
  }
  return distance;
}

void ProbabilisticFusion::FuseFrame(const SensorFramePtr& frame) {
  ROS_DEBUG_STREAM("FuseFrame: Fusing frame: "
                   << frame->GetSensorId()
                   << ", foreground_object_number: " << frame->GetForegroundObjects().size()
                   << ", background_object_number: " << frame->GetBackgroundObjects().size()
                   << ", timestamp: " << FORMAT_TIMESTAMP(frame->GetTimestamp()));
  this->FuseForegroundTrack(frame);
//  this->RemoveOverlapTrack();   // need  to varify
  this->RemoveLostTrack();
}

void ProbabilisticFusion::FuseForegroundTrack(const SensorFramePtr& frame) {
  AssociationOptions options;
  AssociationResult association_result;
  matcher_->Associate(options, frame, scenes_, &association_result);

  const std::vector<TrackMeasurmentPair>& assignments = association_result.assignments;
  this->UpdateAssignedTracks(frame, assignments);
  //  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_assigned_track");

  const std::vector<size_t>& unassigned_track_inds = association_result.unassigned_tracks;
  this->UpdateUnassignedTracks(frame, unassigned_track_inds);
  //  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_unassigned_track");

  const std::vector<size_t>& unassigned_obj_inds = association_result.unassigned_measurements;
  this->CreateNewTracks(frame, unassigned_obj_inds);
  //  PERF_BLOCK_END_WITH_INDICATOR(indicator, "create_track");
}

void ProbabilisticFusion::UpdateAssignedTracks(
    const SensorFramePtr& frame,
    const std::vector<TrackMeasurmentPair>& assignments) {
  // Attention: match_distance should be used
  // in ExistenceFusion to calculate existence score.
  // We set match_distance to zero if track and object are matched,
  // which only has a small difference compared with actural match_distance
  TrackerOptions options;
  options.match_distance = 0;
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first;
    size_t obj_ind = assignments[i].second;
    trackers_[track_ind]->UpdateWithMeasurement(options, frame->GetForegroundObjects()[obj_ind],
                                                frame->GetTimestamp());
  }
}

void ProbabilisticFusion::UpdateUnassignedTracks(const SensorFramePtr& frame,
                                                 const std::vector<size_t>& unassigned_track_inds) {
  // Attention: match_distance(min_match_distance) should be used
  // in ExistenceFusion to calculate toic score.
  // Due to it hasn't been used(mainly for front radar object pub in
  // gatekeeper),
  // we do not set match_distance temporarily.
  TrackerOptions options;
  // options.match_distance = 0;    //org
  options.match_distance = 0.6;
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < unassigned_track_inds.size(); ++i) {
    size_t track_ind = unassigned_track_inds[i];
    trackers_[track_ind]->UpdateWithoutMeasurement(options, sensor_id, frame->GetTimestamp(),
                                                   frame->GetTimestamp());
  }
}

void ProbabilisticFusion::CreateNewTracks(const SensorFramePtr& frame,
                                          const std::vector<size_t>& unassigned_obj_inds) {
  for (size_t i = 0; i < unassigned_obj_inds.size(); ++i) {
    size_t obj_ind = unassigned_obj_inds[i];

    bool prohibition_sensor_flag = false;

    // Modify@ jiangnan:Disable camera from creating a tracker
    if (frame->GetSensorType() == perception::base::SENSING_30 ||
        frame->GetSensorType() == perception::base::SENSING_60 ||
        frame->GetSensorType() == perception::base::SENSING_120)
        prohibition_sensor_flag = true;

    std::shared_ptr<SensorObject> sensor_obj = frame->GetForegroundObjects()[obj_ind];

    // Modify@ jiangnan: Disable vidar from creating a tracker
    if (perception::base::SensorManager::Instance()->IsVidar(frame->GetSensorType()) && sensor_obj->GetBaseObject()->center_ego[0] < -30)
        prohibition_sensor_flag = true;

    std::for_each(params_.prohibition_sensors.begin(), params_.prohibition_sensors.end(),
                  [&](std::string sensor_name) {
                      if (sensor_name == frame->GetSensorId() &&
                          (!(sensor_obj->GetBaseObject()->radar_supplement.is_rear &&
                             sensor_obj->GetBaseObject()->center_ego[0] > -70)))
                          prohibition_sensor_flag = true;
                  });

    if (prohibition_sensor_flag) {
      continue;
    }
    TrackPtr track = TrackPool::Instance().Get();
    track->Initialize(frame->GetForegroundObjects()[obj_ind]);
    scenes_->AddForegroundTrack(track);

    ROS_DEBUG_STREAM("CreateNewTracks: object id: "
                     << frame->GetForegroundObjects()[obj_ind]->GetBaseObject()->track_id
                     << ", create new track: " << track->GetTrackId());

    if (params_.tracker_method == "PbfTracker") {
      std::shared_ptr<BaseTracker> tracker;
      tracker.reset(new PbfTracker());
      tracker->Init(track, frame->GetForegroundObjects()[obj_ind]);
      trackers_.emplace_back(tracker);
    }
  }
}

void ProbabilisticFusion::FusebackgroundTrack(const SensorFramePtr& frame) {
  // 1. association
  size_t track_size = scenes_->GetBackgroundTracks().size();
  size_t obj_size = frame->GetBackgroundObjects().size();
  std::map<int, size_t> local_id_2_track_ind_map;
  std::vector<bool> track_tag(track_size, false);
  std::vector<bool> object_tag(obj_size, false);
  std::vector<TrackMeasurmentPair> assignments;

  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < track_size; ++i) {
    const FusedObjectPtr& obj = background_tracks[i]->GetFusedObject();
    int local_id = obj->GetBaseObject()->track_id;
    local_id_2_track_ind_map[local_id] = i;
  }

  std::vector<SensorObjectPtr>& frame_objs = frame->GetBackgroundObjects();
  for (size_t i = 0; i < obj_size; ++i) {
    int local_id = frame_objs[i]->GetBaseObject()->track_id;
    const auto& it = local_id_2_track_ind_map.find(local_id);
    if (it != local_id_2_track_ind_map.end()) {
      size_t track_ind = it->second;
      assignments.push_back(std::make_pair(track_ind, i));  // track index to obj index
      track_tag[track_ind] = true;
      object_tag[i] = true;
      continue;
    }
  }

  // 2. update assigned track
  for (size_t i = 0; i < assignments.size(); ++i) {
    int track_ind = static_cast<int>(assignments[i].first);
    int obj_ind = static_cast<int>(assignments[i].second);
    background_tracks[track_ind]->UpdateWithSensorObject(frame_objs[obj_ind]);
  }

  // 3. update unassigned track
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < track_tag.size(); ++i) {
    if (!track_tag[i]) {
      background_tracks[i]->UpdateWithoutSensorObject(sensor_id, frame->GetTimestamp());
    }
  }

  // 4. create new track
  for (size_t i = 0; i < object_tag.size(); ++i) {
    if (!object_tag[i]) {
      TrackPtr track = TrackPool::Instance().Get();
      track->Initialize(frame->GetBackgroundObjects()[i], true);
      scenes_->AddBackgroundTrack(track);
    }
  }
}

void ProbabilisticFusion::RemoveLostTrack() {
  // need to remove tracker at the same time
  size_t foreground_track_count = 0;
  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    if (foreground_tracks[i]->IsAlive()) {
      if (i != foreground_track_count) {
        foreground_tracks[foreground_track_count] = foreground_tracks[i];
        trackers_[foreground_track_count] = trackers_[i];
      }
      foreground_track_count++;
    }
  }
  ROS_DEBUG_STREAM("RemoveLostTrack: Remove " << foreground_tracks.size() - foreground_track_count
                                              << " foreground tracks. " << foreground_track_count
                                              << " tracks left.");
  foreground_tracks.resize(foreground_track_count);
  trackers_.resize(foreground_track_count);

  // only need to remove frame track
  size_t background_track_count = 0;
  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < background_tracks.size(); ++i) {
    if (background_tracks[i]->IsAlive()) {
      if (i != background_track_count) {
        background_tracks[background_track_count] = background_tracks[i];
      }
      background_track_count++;
    }
  }
  ROS_DEBUG_STREAM("RemoveLostTrack: Remove " << background_tracks.size() - background_track_count
                                              << " background tracks");
  background_tracks.resize(background_track_count);
}

//void ProbabilisticFusion::RemoveOverlapTrack() {
//  IOU iou;
//  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
//  if (foreground_tracks.size() == 0) {
//    return;
//  }
//  Eigen::Vector3d diff_pos;
//  std::vector<ExtendTrack> processed_tracks;
//  int push_flag = 0;
//  for (int i = 0; i < foreground_tracks.size() - 1; i++) {
//    TrackPtr track = foreground_tracks[i];
//    ExtendTrack process_track;
//    process_track.Init();
//    process_track.track = track;
//    process_track.track->SetMultiOverlapTrack(false);
//    push_flag = 0;
//    int track_id = track->GetTrackId();
//    auto fused_polygons = track->GetFusedObject()->GetBaseObject()->polygon_utm;
//    // Modify by zhaokai
//    if (fused_polygons.size() == 0) {
//      continue;
//    }
//    auto fused1_center = track->GetFusedObject()->GetBaseObject()->position;
//    for (int j = i + 1; j < foreground_tracks.size(); j++) {
//      TrackPtr track2 = foreground_tracks[j];
//      int track2_id = track2->GetTrackId();
//      auto fused2_polygons = track2->GetFusedObject()->GetBaseObject()->polygon_utm;
//      if (fused2_polygons.size() == 0) continue;
//      auto fused2_center = track2->GetFusedObject()->GetBaseObject()->position;
//      diff_pos = fused2_center - fused1_center;
//      float distance =
//          std::move(std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
//      if (distance > s_iou_overlap_filter_threshold_)
//        continue;
//      double insection = iou.Compute(track_id, track2_id, fused_polygons, fused2_polygons);
//      //ROS_ERROR_STREAM("track1 id " << track_id << ", track2 id " << track2_id << ", insection " << insection);
//      if (insection > s_iou_overlap_area_threshold_) {
//        MergeOverlapTracker(track, track2, process_track);
//        push_flag = 1;
//      }
//    }//for j = i + 1
//    if (push_flag) {
//      process_track.Check();
//      processed_tracks.push_back(process_track);
//    }
//  }//for i = 0;
//  //update processed track
//  if (!processed_tracks.empty())
//    UpdateProcessedTrack(processed_tracks);
//}

/**
 *
 * @param track1
 * @param track2
 * @param processed_track
 * @tableofcontents
 * when track1 and track2 overlap 1 track1 and track is predicted or detected delete less track times, 1 delete track1 2 delete track2
 * if track1 is predicted track2 is detected, use track2 update track1 and delete track1, so merge = 3
 * if track2 is predicted track1 is detected, use track1 update track2 and delete track2 ,so merge = 4
 */
//void ProbabilisticFusion::MergeOverlapTracker(TrackPtr& track1, TrackPtr& track2, ExtendTrack &processed_track) {
//  //both detected or both predicted
//  processed_track.track = track1;
//  processed_track.relative_tracks.push_back(track2);
//  if ((!track1->IsPredicted() && !track2->IsPredicted())
//      || (track1->IsPredicted() && track2->IsPredicted())) {
//    //delete less track times
//    if (track1->GetTrackedTimes() > track2->GetTrackedTimes()) {
//      processed_track.merge = 2;//delete track2
//    }else {
//      processed_track.merge = 1; //delete own
//    }
//    return;
//  }//process both detected state
//  if (track1->IsPredicted()) {
//    //update track2 to track1
//    if (track1->GetTrackedTimes() > track2->GetTrackedTimes()) {
//      processed_track.merge = 3; //update other
//    }else {
//      processed_track.merge = 1;
//    }
//  }
//  if (track2->IsPredicted()) {
//    if (track2->GetTrackedTimes() > track1->GetTrackedTimes()) {
//      processed_track.merge = 4;//update other
//    } else {
//      processed_track.merge = 2;
//    }
//  }
//}
//
//void ProbabilisticFusion::UpdateProcessedTrack(std::vector<ExtendTrack> processed_tracks) {
//  std::map<int, int> multi_track;
//  for (auto process_track : processed_tracks) {
//    if (process_track.multi_overlap_track) {
//      multi_track[process_track.track->GetTrackId()] = 10;
//      process_track.track->SetMultiOverlapTrack(true);
//    }//process
//  }
//  //process not overlap bbox
//  for (auto process_track : processed_tracks) {
//    if (process_track.multi_overlap_track) continue;
//    if (multi_track.find(process_track.relative_tracks[0]->GetTrackId()) != multi_track.end()) continue;
//    MergeTracker(process_track);
//  }
//}
//
//void ProbabilisticFusion::MergeTracker(ExtendTrack &processed_track) {
//  switch (processed_track.merge) {
//    case 1:
//      processed_track.track->SetAlive(false);
//      break;
//    case 2:
//      processed_track.relative_tracks[0]->SetAlive(false);
//      break;
//    case 3:
//      processed_track.relative_tracks[0]->GetFusedObject()->GetBaseObject()->track_id = processed_track.track->GetTrackId();
//      processed_track.track->SetAlive(false);
//      break;
//    case 4:
//      processed_track.track->GetFusedObject()->GetBaseObject()->track_id = processed_track.relative_tracks[0]->GetTrackId();
//      processed_track.relative_tracks[0]->SetAlive(false);
//      break;
//    default:
//      break;
//  }
//}

void ProbabilisticFusion::CollectFusedObjects(double timestamp,
                                              std::vector<fusion::ObjectPtr> *fused_objects) {
  // fused_objects->clear();

  size_t fg_obj_num = 0;
  const std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    // 更新历史轨迹
    if (foreground_tracks[i]->GetFusedObject()->GetBaseObject()->trajectory.size() >= 50) {
      foreground_tracks[i]->GetFusedObject()->GetBaseObject()->trajectory.pop_front();
    }
    foreground_tracks[i]->GetFusedObject()->GetBaseObject()->trajectory.push_back(foreground_tracks[i]->GetFusedObject()->GetBaseObject()->position);

    //Modify@jiangnan :  collect  object filter
    if ((foreground_tracks[i]->GetFusedObject()->GetBaseObject()->is_lidar_rb || foreground_tracks[i]->GetFusedObject()->GetBaseObject()->type == perception::fusion::ObjectType::PEDESTRIAN) 
      && (foreground_tracks[i]->IsPredicted()))
      continue;
    if (gate_keeper_->AbleToPublish(foreground_tracks[i])) {
      this->CollectObjectsByTrack(timestamp, foreground_tracks[i], fused_objects);
      ++fg_obj_num;
    }
  }
}

void ProbabilisticFusion::CollectObjectsByTrack(double timestamp,
                                                const TrackPtr& track,
                                                std::vector<fusion::ObjectPtr>* fused_objects) {
  const FusedObjectPtr& fused_object = track->GetFusedObject();
  fusion::ObjectPtr obj = fusion::ObjectPool::Instance().Get();
  *obj = *(fused_object->GetBaseObject());
  const SensorId2ObjectMap& lidar_measurements = track->GetLidarObjects();
  const SensorId2ObjectMap& radar_measurements = track->GetRadarObjects();
  const SensorId2ObjectMap& camera_measurements = track->GetCameraObjects();
  const SensorId2ObjectMap& falcon_lidar_measurement = track->GetFalconLidarObjects();
  const SensorId2ObjectMap& obu_measurements = track->GetObuObjects();
  const SensorId2ObjectMap& vidar_measurements = track->GetVidarObjects();
  int num_measurements = static_cast<int>(
      lidar_measurements.size() + camera_measurements.size() + radar_measurements.size() +
      obu_measurements.size() + vidar_measurements.size() + falcon_lidar_measurement.size());
  obj->fusion_supplement.on_use = true;
  std::vector<fusion::SensorObjectMeasurement>& measurements = obj->fusion_supplement.measurements;
  measurements.resize(num_measurements);
  int m_id = 0;
  for (auto it = lidar_measurements.begin(); it != lidar_measurements.end(); ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    obj->match_id_lidar = it->second->GetBaseObject()->track_id;
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  for (auto it = camera_measurements.begin(); it != camera_measurements.end(); ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    if(base::SensorType::SENSING_60 == it->second->GetSensorType()){
        obj->match_id_camera_60f = it->second->GetBaseObject()->track_id;     
    }
    else if(base::SensorType::SENSING_30 == it->second->GetSensorType()){
        obj->match_id_camera_30f = it->second->GetBaseObject()->track_id;
    }
    else if(base::SensorType::SENSING_120 == it->second->GetSensorType()){
          perception::base::SensorInfo sinfo;
          perception::base::SensorManager::Instance()->GetSensorInfo(it->second->GetSensorId(),&sinfo);
          if(sinfo.orientation() == perception::base::SensorOrientation::FRONT){
            obj->match_id_camera_120f = it->second->GetBaseObject()->track_id; 
          }
          else if(sinfo.orientation() == perception::base::SensorOrientation::RIGHT){
            obj->match_id_camera_120r = it->second->GetBaseObject()->track_id; 
          } 
    }
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  for (auto it = radar_measurements.begin(); it != radar_measurements.end(); ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    obj->match_id_radar = it->second->GetBaseObject()->track_id;
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  // Modify(@liuxinyu): obu_test
  for (auto it = obu_measurements.begin(); it != obu_measurements.end(); ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    obj->match_id_obu = it->second->GetBaseObject()->track_id;
    CollectSensorMeasurementFromObject(it->second, measurement);
  }

  for (auto it = vidar_measurements.begin(); it != vidar_measurements.end(); ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    obj->match_id_vidar = it->second->GetBaseObject()->track_id;
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  // Modify@jiangnan: add falcom lidar
  for (auto it = falcon_lidar_measurement.begin(); it != falcon_lidar_measurement.end();
       ++it, m_id++) {
    fusion::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    obj->match_id_falcon = it->second->GetBaseObject()->track_id;
    CollectSensorMeasurementFromObject(it->second, measurement);
  }

  obj->track_id = track->GetTrackId();
  obj->latest_tracked_time = timestamp;
  // obj->tracking_time = track->GetTrackingPeriod();
  obj->tracking_time = track->GetTrackedTimes();
  obj->is_predicted = track->IsPredicted();
  fused_objects->emplace_back(obj);
}

void ProbabilisticFusion::CollectSensorMeasurementFromObject(
    const SensorObjectConstPtr& object,
    fusion::SensorObjectMeasurement* measurement) {
  measurement->sensor_id = object->GetSensorId();
  measurement->timestamp = object->GetTimestamp();
  measurement->track_id = object->GetBaseObject()->track_id;
  measurement->center = object->GetBaseObject()->center;
  measurement->theta = object->GetBaseObject()->theta;
  measurement->size = object->GetBaseObject()->size;
  measurement->velocity = object->GetBaseObject()->velocity;
  measurement->type = object->GetBaseObject()->type;
  if (IsCamera(object)) {
    measurement->box = object->GetBaseObject()->camera_supplement.box;
  }
}

FUSION_REGISTER_FUSIONSYSTEM(ProbabilisticFusion);

}  // namespace fusion
}  // namespace perception
