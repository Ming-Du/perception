#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/sensor_data_manager.h"
#include "common/extended_kalman_filter/extended_kalman_config.h"
#include "common/iou.h"
#include "common/standard_kalman_filter/kalman_config.h"
#include "common/unscented_kalman_filter/unscented_kalman_config.h"
#include "lib/interface/base_data_association.h"
#include "lib/interface/base_fusion_system.h"
#include "lib/interface/base_gatekeeper.h"
#include "lib/interface/base_tracker.h"
#include "base/track.h"

namespace perception {
namespace fusion {

struct FusionParams {
  bool use_lidar = true;
  bool use_radar = true;
  bool use_camera = true;
  bool use_falcon_lidar = true;  // Modify @jiangnan add falcon lidar
  bool use_obu = true;           // Modify(@liuxinyu):
  bool use_vidar = false;        // add by ming.du
  std::string tracker_method;
  std::string data_association_method;
  std::string gate_keeper_method;
  std::vector<std::string> prohibition_sensors;
};

class ProbabilisticFusion : public BaseFusionSystem {
 public:
  ProbabilisticFusion();
  ~ProbabilisticFusion();

  ProbabilisticFusion(const ProbabilisticFusion&) = delete;
  ProbabilisticFusion& operator=(const ProbabilisticFusion&) = delete;

  bool Init(const FusionInitOptions& init_options) override;

  bool Fuse(const FusionOptions& options,
            const fusion::FrameConstPtr& sensor_frame,
            std::vector<fusion::ObjectPtr>* fused_objects) override;

  std::string Name() const override;

 private:
  bool IsPublishSensor(const fusion::FrameConstPtr& sensor_frame) const;

  void FuseFrame(const SensorFramePtr& frame);

  void CollectFusedObjects(double timestamp, std::vector<fusion::ObjectPtr>* fused_objects);

  void FuseForegroundTrack(const SensorFramePtr& frame);

  void FusebackgroundTrack(const SensorFramePtr& frame);

  void RemoveLostTrack();

  // add by duming, for
  void RemoveOverlapTrack();
  /**
   * author duming
   * description: remove devided tracker about radar
   */
  void RemoveDevidedRadarTrack();
  float ComputeRadarTo3DboxDis(TrackPtr track_radar, TrackPtr track_3Dbox);

//  void MergeOverlapTracker(TrackPtr& track1, TrackPtr& track2, ExtendTrack &processed_track);
//
//  void UpdateProcessedTrack(std::vector<ExtendTrack> processed_tracks);
//  void MergeTracker(ExtendTrack &processed_track);

  void UpdateAssignedTracks(const SensorFramePtr& frame,
                            const std::vector<TrackMeasurmentPair>& assignments);

  void UpdateUnassignedTracks(const SensorFramePtr& frame,
                              const std::vector<size_t>& unassigned_track_inds);

  void CreateNewTracks(const SensorFramePtr& frame, const std::vector<size_t>& unassigned_obj_inds);

  void CollectObjectsByTrack(double timestamp,
                             const TrackPtr& track,
                             std::vector<fusion::ObjectPtr>* fused_objects);

  void CollectSensorMeasurementFromObject(const SensorObjectConstPtr& object,
                                          fusion::SensorObjectMeasurement* measurement);

 private:
  std::mutex data_mutex_;
  std::mutex fuse_mutex_;

  // add by duming used for remove overlap track
  double s_iou_overlap_filter_threshold_ = 10;
  double s_iou_overlap_area_threshold_ = 0.6;

  bool started_ = false;

  ScenePtr scenes_ = nullptr;
  std::vector<std::shared_ptr<BaseTracker>> trackers_;  // for foreground

  std::unique_ptr<BaseDataAssociation> matcher_;
  std::unique_ptr<BaseGatekeeper> gate_keeper_;
  FusionParams params_;
  KalmanConfig kalmanconfig_;
};

}  // namespace fusion
}  // namespace perception
