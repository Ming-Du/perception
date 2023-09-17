
#include "bayes_type_fusion.h"

#include <boost/format.hpp>
#include <limits>
#include <numeric>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "base/base_init_options.h"
#include "base/configmanager.h"
#include "base/sensor_data_manager.h"
#include "common/camera_util.h"
#include "common/file.h"
#include "perception/lib/config_manager/config_manager.h"
#include "common/proto/dst_type_fusion_config.pb.h"

namespace perception {
namespace fusion {

using perception::fusion::GetAbsolutePath;

template <typename Type>
std::string vector2string(const std::vector<Type> &values) {
  return absl::StrCat("(", absl::StrJoin(values, " "), ")");
}

std::string BayesTypeFusion::name_ = "BayesTypeFusion";  // NOLINT

BayesTypeFusion::BayesTypeFusion(TrackPtr track)
    : BaseTypeFusion(track) {

  SensorObjectConstPtr lidar_object = track->GetLatestLidarObject();
  SensorObjectConstPtr camera_object = track->GetLatestCameraObject();
  SensorObjectConstPtr radar_object = track->GetLatestRadarObject();
  SensorObjectConstPtr obu_object = track->GetLatestObuObject();
  SensorObjectConstPtr falcon_lidar_object = track->GetLatestFalconLidarObject();
  SensorObjectConstPtr vidar_object = track->GetLatestVidarObject();
  SensorObjectConstPtr sensor_obj = nullptr;

  if (camera_object != nullptr) {
    sensor_obj = camera_object;
  } else if (obu_object != nullptr) {
    sensor_obj = obu_object;
  } else if (vidar_object != nullptr) {
    sensor_obj = vidar_object;
  } else if (lidar_object != nullptr) {
    sensor_obj = lidar_object;
  } else if (falcon_lidar_object != nullptr) {
    sensor_obj = falcon_lidar_object;
  } else if (radar_object != nullptr) {
    sensor_obj = radar_object;
  }
  if (sensor_obj == nullptr) {
    ROS_ERROR("BayesTypeFusion: Track has no sensor_obj!");
    return;
  }
  bel_probs_.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 1.0 / static_cast<int>(ObjectType::MAX_OBJECT_TYPE));
}

bool BayesTypeFusion::Init() {
  // BaseInitOptions options;
  // if (!GetFusionInitOptions("BayesTypeFusion", &options)) {
  //   ROS_ERROR("Init: GetFusionInitOptions failed!");
  //   return false;
  // }

  // std::string woork_root_config = GetAbsolutePath(
  //     lib::ConfigManager::Instance()->work_root(), options.root_dir);

  // std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  // DstTypeFusionConfig params;

  // if (!perception::fusion::GetProtoFromFile(config, &params)) {
  //   ROS_ERROR_STREAM("Read config failed: " << config);
  //   return false;
  // }

  return true;
}

void BayesTypeFusion::UpdateWithMeasurement(const SensorObjectPtr measurement,
                                            double target_timestamp) {
  
  if (IsRadar(measurement) ||
      (IsLidar(measurement) && measurement->GetBaseObject()->is_lidar_rb) ||
      (measurement->GetBaseObject()->confidence < 0.3)) {
    return;
  }

  if (measurement->GetBaseObject()->type_probs.size() != bel_probs_.size()) {
    ROS_ERROR("UpdateWithMeasurement: Track has wrong size of type_probs!");
    return;
  }
  //bayes update                                        
  double total_prob = 0.0;
  for (int i = 0; i < measurement->GetBaseObject()->type_probs.size(); i++) {
    double prob = measurement->GetBaseObject()->type_probs[i];
    prob = std::min(std::max(prob, 0.0001), 0.9999);
    bel_probs_[i] *= prob;
    total_prob += bel_probs_[i];
  }
  //normalize
  for (double &prob : bel_probs_) {
    prob /= total_prob;
    if(prob > 0.9999) prob = 0.9999;
    if(prob < 0.0001) prob = 0.0001;
  }
  //find max
  auto iter = std::max_element(bel_probs_.begin(), bel_probs_.end());
  int index = static_cast<int>(iter - bel_probs_.begin());
  //update track type
  track_ref_->GetFusedObject()->GetBaseObject()->type = ObjectType(index);
}

void BayesTypeFusion::UpdateWithoutMeasurement(const std::string &sensor_id,
                                             double measurement_timestamp,
                                             double target_timestamp,
                                             double min_match_dist) {
}

std::string BayesTypeFusion::Name() const { return name_; }

}  // namespace fusion
}  // namespace perception
