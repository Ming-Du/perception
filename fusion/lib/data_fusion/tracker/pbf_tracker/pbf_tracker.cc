#include "pbf_tracker.h"
#include "common/file.h"
#include "lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "lib/data_fusion/motion_fusion/kalman_motion_fusion/CV_kalman_motion_fusion.h"
#include "lib/data_fusion/motion_fusion/kalman_motion_fusion/ekf_motion_fusion.h"
#include "lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "lib/data_fusion/motion_fusion/kalman_motion_fusion/ukf_motion_fusion.h"
#include "lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "lib/data_fusion/type_fusion/bayes_type_fusion/bayes_type_fusion.h"
#include "perception/common/include/string_util.h"
#include "perception/lib/config_manager/config_manager.h"

namespace perception {
namespace fusion {

using perception::fusion::GetAbsolutePath;

// TODO(all) fix the static string lint issue
std::string PbfTracker::s_type_fusion_method_ = "DstTypeFusion";  // NOLINT
std::string PbfTracker::s_existence_fusion_method_ =              // NOLINT
    "DstExistenceFusion";
std::string PbfTracker::s_motion_fusion_method_ =  // NOLINT
    "KalmanMotionFusion";
std::string PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";  // NOLINT

PbfTracker::PbfTracker() {}

PbfTracker::~PbfTracker() {}

bool PbfTracker::InitParams() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfTracker", &options)) {
    return false;
  }

  std::string woork_root_config =
      GetAbsolutePath(lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  ROS_DEBUG_STREAM("InitParams: Config file : " << config);
  PbfTrackerConfig params;
  if (!perception::fusion::GetProtoFromFile(config, &params)) {
    ROS_ERROR_STREAM("InitParams: Read config failed: " << config);
    return false;
  }

  // ROS_ERROR_STREAM("Load PbfTrackerConfig: " << params.type_fusion_method() << ","
  //        << params.motion_fusion_method() << "," << params.shape_fusion_method()
  //        << "," << params.existence_fusion_method();
  s_type_fusion_method_ = params.type_fusion_method();
  s_motion_fusion_method_ = params.motion_fusion_method();
  s_existence_fusion_method_ = params.existence_fusion_method();
  s_shape_fusion_method_ = params.shape_fusion_method();
  return true;
}

bool PbfTracker::InitMethods() {
    if (s_type_fusion_method_ == "BayesTypeFusion") {
    type_fusion_.reset(new BayesTypeFusion(track_));
  } else if (s_type_fusion_method_ == "DstTypeFusion") {
    type_fusion_.reset(new DstTypeFusion(track_));
  } else {
    ROS_ERROR_STREAM("InitMethods: Unknown type fusion : " << s_type_fusion_method_);
    return false;
  }

  if (s_motion_fusion_method_ == "KalmanMotionFusion") {
    motion_fusion_.reset(new KalmanMotionFusion(track_));
  } else if (s_motion_fusion_method_ == "CVKalmanMotionFusion") {
    motion_fusion_.reset(new CVKalmanMotionFusion(track_));
  } else if (s_motion_fusion_method_ == "EKFMotionFusion") {
    motion_fusion_.reset(new EkfMotionFusion(track_));
  } else if (s_motion_fusion_method_ == "UkfMotionFusion") {
    motion_fusion_.reset(new UkfMotionFusion(track_));
  } else {
    ROS_ERROR_STREAM("InitMethods: Unknown motion fusion : " << s_motion_fusion_method_);
    return false;
  }

  if (s_existence_fusion_method_ == "DstExistenceFusion") {
    existence_fusion_.reset(new DstExistenceFusion(track_));
  } else {
    ROS_ERROR_STREAM("InitMethods: Unknown existence fusion : " << s_existence_fusion_method_);
    return false;
  }

  if (s_shape_fusion_method_ == "PbfShapeFusion") {
    shape_fusion_.reset(new PbfShapeFusion(track_));
  }else {
    ROS_ERROR_STREAM("InitMethods: Unknown shape fusion : " << s_shape_fusion_method_);
    return false;
  }

  tracker_optimizer_.reset(new tracker_optimizer(track_));
  return true;
}

bool PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  track_ = track;
  if (!InitMethods()) {
    return false;
  }
  shape_fusion_->Init();
  motion_fusion_->Init();
  tracker_optimizer_->Init();
  return true;
}

void PbfTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                       const SensorObjectPtr measurement,
                                       double target_timestamp) {
  // DstExistenceFusion
  if(!IsLidar(measurement)){
    track_->addMatchTimes();
  }
  std::string sensor_id = measurement->GetSensorId();

  if (IsLidar(measurement) || IsFalconLidar(measurement)) {
    tracker_optimizer_->StoreHistoryTrajectory(measurement->GetBaseObject()->position,
                                               measurement->GetBaseObject()->velocity);
    tracker_optimizer_->CheckMotionStateIsStatic();
  }

  existence_fusion_->UpdateWithMeasurement(measurement, target_timestamp, options.match_distance);
  if (IsCamera(measurement) /*|| IsVidar(measurement)*/) {
    // motion_fusion_->UpdateWithoutMeasurement(sensor_id, target_timestamp, target_timestamp);
  } else {
    motion_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  }
  //update shape fusion
  shape_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  // DstTypeFusion
  type_fusion_->UpdateWithMeasurement(measurement, target_timestamp);

  tracker_optimizer_->SetStaticMotion();  // need  to varify
  tracker_optimizer_->CheckYawIsReasonable();

  track_->UpdateWithSensorObject(measurement);

  if (IsVidar(measurement) || IsLidar(measurement) || IsFalconLidar(measurement) ||
      IsRadar(measurement) || IsObu(measurement)) {
    track_->AddTrackedTimes();
  }

  track_->SetPredictionState(false);
}

void PbfTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                          const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double target_timestamp) {
  existence_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp, target_timestamp,
                                              options.match_distance);
  motion_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp, target_timestamp);
  shape_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp, target_timestamp);
  type_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp, target_timestamp,
                                         options.match_distance);
  track_->UpdateWithoutSensorObject(sensor_id, measurement_timestamp);
  track_->SetPredictionState(true);//set prediction true
}

std::string PbfTracker::Name() const {
  return "PbfTracker";
}

}  // namespace fusion
}  // namespace perception

