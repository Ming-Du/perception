
#include "dst_type_fusion.h"

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
std::string vector2string(const std::vector<Type>& values) {
  return absl::StrCat("(", absl::StrJoin(values, " "), ")");
}

std::string DstTypeFusion::name_ = "DstTypeFusion";
DstMaps DstTypeFusion::dst_maps_;
DstTypeFusionOptions DstTypeFusion::options_;

DstTypeFusion::DstTypeFusion(TrackPtr track)
    : BaseTypeFusion(track), fused_dst_(name_) {
  Dst sensor_dst(name_);
  sensor_dst = TypeProbsToDst(track->GetFusedObject()->GetBaseObject()->type_probs);
  SensorObjectConstPtr lidar_object = track->GetLatestLidarObject();
  SensorObjectConstPtr camera_object = track->GetLatestCameraObject();
  SensorObjectConstPtr radar_object = track->GetLatestRadarObject();
  SensorObjectConstPtr obu_object = track->GetLatestObuObject();
  SensorObjectConstPtr falcon_lidar_object = track_ref_->GetLatestFalconLidarObject();
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
    ROS_ERROR("track has no sensor_obj!");
    return;
  }
  // change to get sensor type-syf
  auto type = fusion::kMogoSensorType2NameMap.find(sensor_obj->GetSensorType());
  if (type == fusion::kMogoSensorType2NameMap.end()) {
    sensor_reliability_ = 0.5;
    ROS_WARN("No name map from Mogo ObjectType!");
  } else {
    sensor_reliability_ = GetReliability(type->second);
  }
  fused_dst_ = fused_dst_ + sensor_dst * sensor_reliability_;
  UpdateTypeState();
}

bool DstTypeFusion::Init() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("DstTypeFusion", &options)) {
    ROS_ERROR("GetFusionInitOptions failed!");
    return false;
  }

  std::string woork_root_config =
      GetAbsolutePath(lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  DstTypeFusionConfig params;

  if (!perception::fusion::GetProtoFromFile(config, &params)) {
    ROS_ERROR_STREAM("Read config failed: " << config);
    return false;
  }

  for (auto camera_param : params.camera_params()) {
    std::string camera_id = camera_param.name();
    options_.camera_max_valid_dist_[camera_id] = camera_param.valid_dist();
    options_.sensor_reliability_[camera_id] = camera_param.reliability();
    options_.sensor_reliability_for_unknown_[camera_id] = camera_param.reliability_for_unknown();
    ROS_DEBUG_STREAM("dst type fusion params: " << camera_id
          << " max valid dist: " << options_.camera_max_valid_dist_[camera_id]
          << " reliability: " << options_.sensor_reliability_[camera_id]
          << " reliability for unknown: "
          << options_.sensor_reliability_for_unknown_[camera_id]);
  }

  for (auto lidar_param : params.lidar_params()) {
    std::string lidar_id = lidar_param.name();

    options_.sensor_reliability_[lidar_id] = lidar_param.reliability();
    options_.sensor_reliability_for_unknown_[lidar_id] = lidar_param.reliability_for_unknown();
    ROS_DEBUG_STREAM("dst type fusion params: " << lidar_id
          << " reliability: " << options_.sensor_reliability_[lidar_id]
          << " reliability for unknown: "
          << options_.sensor_reliability_for_unknown_[lidar_id]);
  }
  for (auto vidar_param : params.vidar_params()) {
    std::string vidar_id = vidar_param.name();
    options_.sensor_reliability_[vidar_id] = vidar_param.reliability();
    options_.sensor_reliability_for_unknown_[vidar_id] = vidar_param.reliability_for_unknown();
    ROS_DEBUG_STREAM("dst type fusion params: " << vidar_id
                                              << " reliability: " <<
                                              options_.sensor_reliability_[vidar_id]
                                              << " reliability for unknown: " <<
                                              options_.sensor_reliability_for_unknown_[vidar_id]);
  }
  for (auto v2x_param : params.v2x_params()) {
    std::string v2x_id = v2x_param.name();
    options_.sensor_reliability_[v2x_id] = v2x_param.reliability();
    options_.sensor_reliability_for_unknown_[v2x_id] = v2x_param.reliability_for_unknown();
    ROS_DEBUG_STREAM("dst type fusion params: " << v2x_id
                                              << " reliability: " <<
                                              options_.sensor_reliability_[v2x_id]
                                              << " reliability for unknown: " <<
                                              options_.sensor_reliability_for_unknown_[v2x_id]);
  }
  if (DstManager::Instance()->IsAppAdded(name_)) {
    return true;
  }
  DstManager::Instance()->AddApp(name_, dst_maps_.fod_subsets_, dst_maps_.subset_names_);
  return DstManager::Instance()->IsAppAdded(name_);
}

void DstTypeFusion::UpdateWithMeasurement(const SensorObjectPtr measurement,
                                          double target_timestamp) {
  if (IsRadar(measurement) || (measurement->GetBaseObject()->confidence < 0.2) ||
      (track_ref_->GetFusedObject()->GetBaseObject()->type == ObjectType::VEGETATION)) {
    return;
  }
  //  &&  (measurement->GetBaseObject()->type == ObjectType::UNKNOWN)

  Dst measurement_dst(name_);
  measurement_dst = TypeProbsToDst(measurement->GetBaseObject()->type_probs);
  ROS_DEBUG_STREAM("DstTypeFusion: type_probs: " << vector2string<float>(measurement->GetBaseObject()->type_probs));
  auto type = fusion::kMogoSensorType2NameMap.find(measurement->GetSensorType());
  if (type == fusion::kMogoSensorType2NameMap.end()) {
    sensor_reliability_ = 0.5;
    ROS_WARN("DstTypeFusion: sensor_id : %s No name map from Mogo ObjectType!", measurement->GetSensorId().c_str());
  } else {
    ROS_DEBUG_STREAM("DstTypeFusion: sensor_id : " << measurement->GetSensorId() << ", name : " << type->second);
    sensor_reliability_ = GetReliability(type->second);
  }
  fused_dst_ =
      fused_dst_ + measurement_dst * sensor_reliability_;
  ROS_DEBUG_STREAM("DstTypeFusion: reliability: " << GetReliability(type->second));
  // update subtype
  if (IsCamera(measurement)) {
    track_ref_->GetFusedObject()->GetBaseObject()->sub_type =
        measurement->GetBaseObject()->sub_type;
  }
  UpdateTypeState();
}

void DstTypeFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                             double measurement_timestamp,
                                             double target_timestamp,
                                             double min_match_dist) {
  /*perception::base::SensorManager *sensor_manager =
  perception::base::SensorManager::Instance(); if
  (sensor_manager->IsCamera(sensor_id)) {
    // add the evidence of OTHERS_UNMOVABLE
    double in_view_ratio = 0.0;
    // 1.get camera intrinsic and pose
    SensorDataManager *sensor_data_manager = SensorDataManager::Instance();
    base::BaseCameraModelPtr camera_model =
        sensor_data_manager->GetCameraIntrinsic(sensor_id);
    if(!(camera_model != nullptr))
    {
      ROS_ERROR_STREAM("Failed to get camera intrinsic for " << sensor_id);
    }

    Eigen::Affine3d sensor2world_pose;
    bool status = sensor_data_manager->GetPose(sensor_id, measurement_timestamp,
                                               &sensor2world_pose);

    //change from sensor_id to sensor_type-syf
    if (!sensor_manager->GetSensorInfo(sensor_id, &camera_sensor_info_))
    ROS_ERROR_STREAM("camera get sensor info failure : " << sensor_id);
          auto type =
  fusion::kMogoSensorType2NameMap.find(camera_sensor_info_.type()); if (type ==
  fusion::kMogoSensorType2NameMap.end()) { ROS_WARN("No name map from Mogo
  ObjectType!");
    }
    auto max_dist_it = options_.camera_max_valid_dist_.find(type->second);
    // auto max_dist_it = options_.camera_max_valid_dist_.find(sensor_id); //org
    if (max_dist_it == options_.camera_max_valid_dist_.end()) {
      ROS_WARN_STREAM(boost::format(
                   "There is no pre-defined max valid camera"
                   " dist for sensor type: %s") %
                   sensor_id);
    }
    if (status && max_dist_it != options_.camera_max_valid_dist_.end() && type
  != fusion::kMogoSensorType2NameMap.end()) { SensorObjectConstPtr lidar_object
  = track_ref_->GetLatestLidarObject(); SensorObjectConstPtr radar_object =
  track_ref_->GetLatestRadarObject(); double camera_max_dist =
  max_dist_it->second; if (lidar_object != nullptr) { in_view_ratio =
  ObjectInCameraView( lidar_object, camera_model, sensor2world_pose,
            measurement_timestamp, camera_max_dist, true, false);
      } else if (radar_object != nullptr) {
        in_view_ratio = ObjectInCameraView(
            radar_object, camera_model, sensor2world_pose,
            measurement_timestamp, camera_max_dist, false, false);
      }
    }

    // TODO(yuantingrong): need consider the camera object only
    // use the min_match_dist to decide the occlusion
    // TODO(yuantingrong): not reasonable, when sensor type is camera,
    // the input min_match_dist  is a probability([0, 1], 1 is better)
    // to measure how well the fused object locate
    // in a box detected by the camera.
    auto loss_fun = [](double dist_score) {
      CHECK_GE(dist_score, 0.0);
      static constexpr double th = 0.9;
      if (dist_score >= th) {
        return 0.0;
      }
      double res = 1 - (dist_score / th) * (dist_score / th);
      return res * res;
    };
    double occlusion_score = loss_fun(min_match_dist);
    std::map<uint64_t, double> fp_dst_map = {{DstMaps::OTHERS, 0.7},
                                             {DstMaps::UNKNOWN, 0.3}};
    Dst fp_dst(name_);
    fp_dst.SetBba(fp_dst_map);
    // fused_dst_ = fused_dst_ +
    //              (fp_dst * in_view_ratio * occlusion_score) *
    //                  GetReliabilityForUnKnown(sensor_id,
  measurement_timestamp);
    //change from sensor id to sensor type - syf
    fused_dst_ = fused_dst_ +
                 (fp_dst * in_view_ratio * occlusion_score) *
                     GetReliabilityForUnKnown(type->second,
  measurement_timestamp); UpdateTypeState();
  }*/
}

std::string DstTypeFusion::Name() const {
  return name_;
}

bool DstTypeFusion::TypToHyp(size_t object_type, uint64_t* hypothesis_type) const {
  auto find_res = dst_maps_.typ_to_hyp_map_.find(object_type);
  if (find_res == dst_maps_.typ_to_hyp_map_.end()) {
    return false;
  }
  *hypothesis_type = find_res->second;
  return true;
}

bool DstTypeFusion::HypToTyp(uint64_t hypothesis_type, size_t* object_type) const {
  auto find_res = dst_maps_.hyp_to_typ_map_.find(hypothesis_type);
  if (find_res == dst_maps_.hyp_to_typ_map_.end()) {
    return false;
  }
  *object_type = find_res->second;
  return true;
}

Dst DstTypeFusion::TypeProbsToDst(const std::vector<float>& type_probs) {
  Dst res_dst(name_);
  double type_probs_sum = std::accumulate(type_probs.begin(), type_probs.end(), 0.0);
  if (type_probs_sum < std::numeric_limits<double>::min()) {
    // ROS_WARN("the sum of types probability equal 0.0!");
    return res_dst;
  }
  // if (type_probs.size() > fusion::ObjectType::UNKNOWN_UNMOVABLE &&
  // type_probs[(int)fusion::ObjectType::UNKNOWN_UNMOVABLE] > 0.0f) {
  // ROS_DEBUG_STREAM("unknonw_unmovable prob = " <<
  //    type_probs[(int)fusion::ObjectType::UNKNOWN_UNMOVABLE] << " > 0.0f");
  //}
  std::map<uint64_t, double> res_bba_map;
  for (size_t i = 0; i < type_probs.size(); ++i) {
    size_t typ = i;
    uint64_t hyp = 0;
    if (!TypToHyp(typ, &hyp)) {
      continue;
    }
    res_bba_map[hyp] = type_probs[typ];
  }
  // for support camera probs detection prob
  // if (type_probs_sum < 1.0) {
  //     auto find_res = res_bba_map.find(DSTInitiator::OTHERS);
  //     if (find_res == res_bba_map.end()) {
  //         res_bba_map[DSTInitiator::OTHERS] = 1 - type_probs_sum;
  //     } else {
  //         find_res->second += (1 - type_probs_sum);
  //     }
  // }
  if (!(res_dst.SetBba(res_bba_map))) {
    ROS_ERROR("failed to res_dst.SetBba(res_bba_map)).");
  }
  return res_dst;
}

double DstTypeFusion::GetReliability(const std::string& sensor_id) const {
  auto find_res = options_.sensor_reliability_.find(sensor_id);
  if (find_res == options_.sensor_reliability_.end()) {
    ROS_DEBUG_STREAM("the sensor type: " << sensor_id << " is not supported by class fusion!");
    return 0.0;
  }
  return find_res->second;
}

double DstTypeFusion::GetReliabilityForUnKnown(const std::string& sensor_id,
                                               double measurement_timestamp) const {
  auto find_res = options_.sensor_reliability_for_unknown_.find(sensor_id);
  if (find_res == options_.sensor_reliability_for_unknown_.end()) {
    ROS_DEBUG_STREAM("the sensor type: " << sensor_id << " is not supported by class fusion!");
    return 0.0;
  }
  time_t rawtime = static_cast<time_t>(measurement_timestamp);
  struct tm timeinfo;
  localtime_r(&rawtime, &timeinfo);
  bool is_night = (timeinfo.tm_hour >= 17);
  double prob =
      (perception::base::SensorManager::Instance()->IsCamera(sensor_id) && is_night) ? 0.1 : 1.0;
  return find_res->second * prob;
}

void DstTypeFusion::UpdateTypeState() {
  const std::vector<double>& fused_dst_vec = fused_dst_.GetBbaVec();
  auto max_iter = std::max_element(fused_dst_vec.begin(), fused_dst_vec.end());
  size_t max_hyp_ind = max_iter - fused_dst_vec.begin();
  uint64_t max_hyp = DstManager::Instance()->IndToFodSubset(name_, max_hyp_ind);
  if (max_hyp == DstMaps::OTHERS) {
    ROS_DEBUG_STREAM("max hyp is UNKNOWN_MOVABLE" << fused_dst_.PrintBba());
  }

  // update type
  size_t object_type = 0;
  HypToTyp(max_hyp, &object_type);
  track_ref_->GetFusedObject()->GetBaseObject()->type =
      static_cast<fusion::ObjectType>(object_type);
  auto type =
      fusion::kSubType2TypeMap.find(track_ref_->GetFusedObject()->GetBaseObject()->sub_type);
  if (type != fusion::kSubType2TypeMap.end() &&
      type->second != track_ref_->GetFusedObject()->GetBaseObject()->type) {
    track_ref_->GetFusedObject()->GetBaseObject()->sub_type = fusion::ObjectSubType::UNKNOWN;
  }

  // if (type != fusion::kSubType2TypeMap.end() &&
  //     type->second != track_ref_->GetFusedObject()->GetBaseObject()->type) {
  //   track_ref_->GetFusedObject()->GetBaseObject()->sub_type =
  //       fusion::ObjectSubType::UNKNOWN;
  // }      //org

  // update type_probs
  std::vector<float> type_probs(static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE), 0);
  for (size_t i = 0; i < fused_dst_vec.size(); ++i) {
    size_t type = 0;
    uint64_t hyp = DstManager::Instance()->IndToFodSubset(name_, i);
    HypToTyp(hyp, &type);
    type_probs[type] += static_cast<float>(fused_dst_vec[i]);
  }
  track_ref_->GetFusedObject()->GetBaseObject()->type_probs = type_probs;
}

}  // namespace fusion
}  // namespace perception
