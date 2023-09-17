#pragma once

#include <string>

#include "lib/interface/base_gatekeeper.h"
#include "perception/base/sensor_manager/sensor_manager.h"  // Modify-guoxiaoxiao
#include "base/configmanager.h"

namespace perception {
namespace fusion {

struct PbfGatekeeperParams {
  bool publish_if_has_lidar = true;
  bool publish_if_has_radar = true;
  bool publish_if_has_camera = true;
  bool publish_if_has_falcon_lidar = true;
  bool use_camera_3d = true;

  double min_radar_confident_distance = 40;
  double max_radar_confident_angle = 20;
  double min_camera_publish_distance = 50;
  double invisible_period_threshold = 0.001;
  double toic_threshold = 0.8;
  double existence_threshold = 0.7;
  double radar_existence_threshold = 0.9;

  bool use_track_time_pub_strategy = true;
  int pub_track_time_thresh = 3;

  // Modify(@liuxinyu): obu_test
  bool publish_if_has_obu = true;
  double min_obu_publish_distance = 50;
  std::vector<float> frustum_front60;
};

class PbfGatekeeper : public BaseGatekeeper {
 public:
  PbfGatekeeper();
  ~PbfGatekeeper();

  PbfGatekeeper(const PbfGatekeeper&) = delete;
  PbfGatekeeper& operator=(const PbfGatekeeper&) = delete;

  bool Init() override;

  bool AbleToPublish(const TrackPtr& track) override;

  std::string Name() const override;

 private:
  bool LidarAbleToPublish(const TrackPtr& track);
  bool FalconLidarAbleToPublish(const TrackPtr& track);
  // Modify @jiangnan:add falcon lidar
  bool RadarAbleToPublish(const TrackPtr& track, bool is_night);
  bool CameraAbleToPublish(const TrackPtr& track, bool is_night);
  // Modify(@liuxinyu): obu_test
  bool ObuAbleToPublish(const TrackPtr& track, bool is_night);
  // ming.du
  bool VidarAbleToPublish(const TrackPtr& track);  // as vidar a special lidar

  bool CheckNoise(const TrackPtr& track);

  PbfGatekeeperParams params_;
  int vehicle_;
};

}  // namespace fusion
}  // namespace perception
