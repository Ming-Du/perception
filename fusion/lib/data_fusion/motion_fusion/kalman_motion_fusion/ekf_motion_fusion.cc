
#include "ekf_motion_fusion.h"

#include <algorithm>
#include <iomanip>
#include <limits>

#include "perception/base/sensor_manager/sensor_manager.h"
#include "perception/common/geometry/basic.h"
#include "common/proto/sensor_meta.pb.h"

namespace {
constexpr double kDoubleEpsilon = std::numeric_limits<double>::epsilon();
}

namespace perception {
namespace fusion {

int EkfMotionFusion::s_eval_window_ = 3;
size_t EkfMotionFusion::s_history_size_maximum_ = 20;

const int INDEX_X = 0;
const int INDEX_Y = 1;
const int INDEX_V = 2;
const int INDEX_THETA = 3;
const int INDEX_OMEGA = 4;
const double K_EPS = 0.0001;

bool EkfMotionFusion::Init() {
  if (track_ref_ == nullptr) {
    return false;
  }
  if (track_ref_->GetLatestLidarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestLidarObject());
  } else if (track_ref_->GetLatestRadarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestRadarObject());
  }
  return true;
}

bool EkfMotionFusion::InitFilter(const SensorObjectConstPtr& sensor_object) {
  // std::string ekf_config_path = std::string(PROJECT_PATH) + std::string("/config/filter");
  //  std::string ekf_config_path = FLAGS_work_root + std::string("/filter");
  //  ekf_filter_config_.init(ekf_config_path.c_str());
  Eigen::VectorXd global_states;
  global_states.setZero(5, 1);
  global_states(INDEX_X) = sensor_object->GetBaseObject()->anchor_point(0);
  global_states(INDEX_Y) = sensor_object->GetBaseObject()->anchor_point(1);
  global_states(INDEX_V) = std::sqrt(std::pow(sensor_object->GetBaseObject()->velocity(0), 2) +
                                     std::pow(sensor_object->GetBaseObject()->velocity(1), 2));
  global_states(INDEX_THETA) = sensor_object->GetBaseObject()->theta;
  // global_states(INDEX_OMEGA) =

  ekf_filter_.init(&ekf_filter_config_, global_states);

  if (sensor_object->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(sensor_object->GetSensorType(),
                        sensor_object->GetBaseObject()->velocity.cast<double>(),
                        sensor_object->GetTimestamp());
  }
  return true;
}

void EkfMotionFusion::GetStates(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity) {
  *anchor_point = fused_anchor_point_;
  *velocity = fused_velocity_;
}

void EkfMotionFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                               double measurement_timestamp,
                                               double target_timestamp) {
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr camera_ptr = track_ref_->GetLatestCameraObject();
  bool is_alive = (lidar_ptr != nullptr || radar_ptr != nullptr || camera_ptr != nullptr);
  if (filter_init_ && is_alive) {
    double time_diff = measurement_timestamp - track_ref_->GetFusedObject()->GetTimestamp();
    MotionFusionWithoutMeasurement(time_diff);
    fused_state_ = ekf_filter_.State();
    fused_anchor_point_(0) = fused_state_[0];
    fused_anchor_point_(1) = fused_state_[1];
    fused_velocity_(0) = fused_state_[2];
    fused_velocity_(1) = fused_state_[3];
    fused_yaw_ = fused_state_[4];
    // fused_acceleration_(0) = ekf_filter_.GetStates()(4);
    // fused_acceleration_(1) = ekf_filter_.GetStates()(5);
  }
  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  // of camera improved, this step is not needed.
  UpdateMotionState();
}

void EkfMotionFusion::UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                                            double target_timestamp) {
  fused_anchor_point_ = measurement->GetBaseObject()->anchor_point.cast<double>();
  fused_velocity_ = measurement->GetBaseObject()->velocity.cast<double>();
  fused_acceleration_ = measurement->GetBaseObject()->acceleration.cast<double>();
  center_uncertainty_ = measurement->GetBaseObject()->center_uncertainty;
  velo_uncertainty_ = measurement->GetBaseObject()->velocity_uncertainty;
  acc_uncertainty_ = measurement->GetBaseObject()->acceleration_uncertainty;

  double time_diff = measurement->GetTimestamp() - track_ref_->GetFusedObject()->GetTimestamp();
  bool is_lidar = IsLidar(measurement);
  bool is_radar = IsRadar(measurement);
  bool is_camera = IsCamera(measurement);
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr camera_ptr = track_ref_->GetLatestCameraObject();
  // Motion fusion
  if (is_lidar || is_radar || is_camera) {
    if (filter_init_) {
      MotionFusionWithMeasurement(measurement, time_diff);
    } else {
      filter_init_ = InitFilter(measurement);
    }
    if (!filter_init_) {
      // No kalman result, no matter which sensortype
      // of measurement, use measurement's
      // anchor point and velocity
      return;
    }
  }
  // shape & location fusion
  if (is_lidar || is_radar) {
    if (is_lidar || (is_radar && lidar_ptr == nullptr)) {
      // 1) measurement is lidar: use lidar's anchor point
      //    , use fused velocity
      // 2) measurement is radar, and has no history lidar:
      // use radar's anchor point, use fused velocity
      fused_velocity_(0) = ekf_filter_.State()(2);  // ekf_filter_.GetStates()(2);
      fused_velocity_(1) = ekf_filter_.State()(3);  // ekf_filter_.GetStates()(3);
    }
    if (is_radar && lidar_ptr != nullptr) {
      // measurement is radar, has history lidar:
      // use fused achor point, use fused velocity
      fused_anchor_point_(0) = ekf_filter_.State()(0);  // ekf_filter_.GetStates()(0);
      fused_anchor_point_(1) = ekf_filter_.State()(1);  // ekf_filter_.GetStates()(1);
      fused_velocity_(0) = ekf_filter_.State()(2);      // ekf_filter_.GetStates()(2);
      fused_velocity_(1) = ekf_filter_.State()(3);      // ekf_filter_.GetStates()(3);
    }
  } else if (is_camera) {
    if (filter_init_) {
      if (lidar_ptr != nullptr) {
        // measurement is camera, has history lidar
        // use fused position, use fused velocity
        fused_anchor_point_(0) = ekf_filter_.State()(0);  // ekf_filter_.GetStates()(0);
        fused_anchor_point_(1) = ekf_filter_.State()(1);  // ekf_filter_.GetStates()(1);
        fused_velocity_(0) = ekf_filter_.State()(2);      // ekf_filter_.GetStates()(2);
        fused_velocity_(1) = ekf_filter_.State()(3);      // ekf_filter_.GetStates()(3);
      } else if (radar_ptr != nullptr) {
        // measurement is camera, has no history lidar, but
        // has history radar, use history radar's position
        // use fused velocity
        fused_anchor_point_ = radar_ptr->GetBaseObject()->anchor_point.cast<double>();
        fused_velocity_(0) = ekf_filter_.State()(2);  // ekf_filter_.GetStates()(2);
        fused_velocity_(1) = ekf_filter_.State()(3);  // ekf_filter_.GetStates()(3);
      } else {
        // measurement is camera, has no history lidar
        // or radar, use measurement's anchor point and
        // velocity, do nothing(at the beginning of this
        // function, fused velocity/anchor point has been
        // set to as same as measurement)
      }
    }
  }

  // fused_acceleration_(0) = ekf_filter_.GetStates()(4);
  // fused_acceleration_(1) = ekf_filter_.GetStates()(5);

  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  // of camera improved, this step is not needed.
  UpdateMotionState();
}

/*
void EkfMotionFusion::MotionFusionWithoutMeasurement(
    const double time_diff) {
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;
  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  env_uncertainty.setZero(6, 6);
  ekf_filter_.Predict(transform_matrix, env_uncertainty);
} */

void EkfMotionFusion::MotionFusionWithoutMeasurement(const double time_diff) {
  ekf_filter_.predict(time_diff);
}

/*
void EkfMotionFusion::MotionFusionWithMeasurement(
    const SensorObjectConstPtr& measurement, double time_diff) {
  // we use kalman filter to update our tracker.
  // The pipeline is detailed as follows:
  // 1) compute the time diff to predict the tracker
  //    (although we introduce the acceleration, we
  //    doesn't use it to update the position)    //pos_x = pos_x + vel_x *  time_diff (+ 0.5 *
acc_x * acc_x); not use acc_x - syf
  // 2) DeCorrelation the uncertainty matrix (we belief
  //    that the velocity won`t be affected by position)
  // 3) compute the acceleration of the measurement
  // 4) use the history radar or lidar(depend on which sensor
  //    type in current) to correct the observation
  // 5) set r_matrix according to converged or not
  // 6) use kalman to correct the predict before
  // 7) use correction breakdown to eliminate the unreasonable
  //    acceleration gain or velocity noise
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;

  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;

  env_uncertainty.setIdentity(6, 6);
  env_uncertainty *= 0.5;

  ekf_filter_.Predict(transform_matrix, env_uncertainty);

  Eigen::Vector3d measured_acceleration = Eigen::Vector3d::Zero();
  measured_acceleration = ComputeAccelerationMeasurement(
      measurement->GetSensorType(),
      measurement->GetBaseObject()->velocity.cast<double>(),
      measurement->GetTimestamp());

  Eigen::VectorXd observation;
  observation.setZero(6, 1);
  observation(0) = measurement->GetBaseObject()->center(0);
  observation(1) = measurement->GetBaseObject()->center(1);
  observation(2) = measurement->GetBaseObject()->velocity(0);
  observation(3) = measurement->GetBaseObject()->velocity(1);
  observation(4) = measured_acceleration(0);
  observation(5) = measured_acceleration(1);

  Eigen::MatrixXd r_matrix;
  r_matrix.setIdentity(6, 6);
  r_matrix.topLeftCorner(2, 2) = measurement->GetBaseObject()
                                     ->center_uncertainty.topLeftCorner(2, 2)
                                     .cast<double>();
  r_matrix.block<2, 2>(2, 2) = measurement->GetBaseObject()
                                   ->velocity_uncertainty.topLeftCorner(2, 2)
                                   .cast<double>();

  ADEBUG << "fusion_original_measurement@(" << std::setprecision(10)
         << observation(0) << "," << observation(1) << "," << observation(2)
         << "," << observation(3) << ")";
  ADEBUG << "fusion_original_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  // Compute pseudo measurement
  Eigen::Vector4d temp_observation = observation.head(4);
  Eigen::Vector4d pseudo_measurement =
      ComputePseudoMeasurement(temp_observation, measurement->GetSensorType());
  observation.head(4) = pseudo_measurement;

  //if converged, store in history data for acceleration calculation
  if (measurement->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(measurement->GetSensorType(),
                        measurement->GetBaseObject()->velocity.cast<double>(),
                        measurement->GetTimestamp());
  }

  // Adapt noise level to rewarding status. According if lidar and if velocity converged.
  RewardRMatrix(measurement->GetSensorType(),
                measurement->GetBaseObject()->velocity_converged, &r_matrix);

  ADEBUG << "fusion_pseudo_measurement@(" << std::setprecision(10)
         << observation(0) << "," << observation(1) << "," << observation(2)
         << "," << observation(3) << ")";
  ADEBUG << "fusion_pseudo_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  ekf_filter_.DeCorrelation(2, 0, 2, 2);
  ekf_filter_.Correct(observation, r_matrix);
  ekf_filter_.CorrectionBreakdown();

  ADEBUG << "fusion_filter_belief@(" << std::setprecision(10)
         << ekf_filter_.State()(0) << ","                     //  << ekf_filter_.GetStates()(0) <<
","
         << ekf_filter_.State()(1) << ","                     //  << ekf_filter_.GetStates()(1) <<
","
         << ekf_filter_.State()(2) << ","           //<< ekf_filter_.GetStates()(2) << ","
         << ekf_filter_.State()(3) << ")";        //<< ekf_filter_.GetStates()(3) << ")";

  // ADEBUG << "fusion_filter_belief_covariance@("
  //        << ekf_filter_.GetUncertainty()(0, 0) << ","
  //        << ekf_filter_.GetUncertainty()(0, 1) << ","
  //        << ekf_filter_.GetUncertainty()(1, 0) << ","
  //        << ekf_filter_.GetUncertainty()(1, 1) << ","
  //        << ekf_filter_.GetUncertainty()(2, 2) << ","
  //        << ekf_filter_.GetUncertainty()(2, 3) << ","
  //        << ekf_filter_.GetUncertainty()(3, 2) << ","
  //        << ekf_filter_.GetUncertainty()(3, 3) << ")";
}     */

void EkfMotionFusion::MotionFusionWithMeasurement(const SensorObjectConstPtr& measurement,
                                                  double time_diff) {
  // we use kalman filter to update our tracker.
  // The pipeline is detailed as follows:
  // 1) compute the time diff to predict the tracker
  //    (although we introduce the acceleration, we
  //    doesn't use it to update the position)    //pos_x = pos_x + vel_x *  time_diff (+ 0.5 *
  //    acc_x * acc_x); not use acc_x - syf
  // 2) DeCorrelation the uncertainty matrix (we belief
  //    that the velocity won`t be affected by position)
  // 3) compute the acceleration of the measurement
  // 4) use the history radar or lidar(depend on which sensor
  //    type in current) to correct the observation
  // 5) set r_matrix according to converged or not
  // 6) use kalman to correct the predict before
  // 7) use correction breakdown to eliminate the unreasonable
  //    acceleration gain or velocity noise
  /*
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;

  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;

  env_uncertainty.setIdentity(6, 6);
  env_uncertainty *= 0.5;

  ekf_filter_.Predict(transform_matrix, env_uncertainty); */

  ekf_filter_.predict(time_diff);

  // if converged, store in history data for acceleration calculation
  if (measurement->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(measurement->GetSensorType(),
                        measurement->GetBaseObject()->velocity.cast<double>(),
                        measurement->GetTimestamp());
  }

  Eigen::VectorXd global_states;
  global_states.setZero(5, 1);
  global_states(INDEX_X) = measurement->GetBaseObject()->center(0);
  global_states(INDEX_Y) = measurement->GetBaseObject()->center(1);
  global_states(INDEX_V) = std::sqrt(std::pow(measurement->GetBaseObject()->velocity(0), 2) +
                                     std::pow(measurement->GetBaseObject()->velocity(1), 2));
  global_states(INDEX_THETA) = measurement->GetBaseObject()->theta;

  ekf_filter_.update(time_diff, global_states);

  /*
    Eigen::Vector3d measured_acceleration = Eigen::Vector3d::Zero();
    measured_acceleration = ComputeAccelerationMeasurement(
        measurement->GetSensorType(),
        measurement->GetBaseObject()->velocity.cast<double>(),
        measurement->GetTimestamp());

    Eigen::VectorXd observation;
    observation.setZero(6, 1);
    observation(0) = measurement->GetBaseObject()->center(0);
    observation(1) = measurement->GetBaseObject()->center(1);
    observation(2) = measurement->GetBaseObject()->velocity(0);
    observation(3) = measurement->GetBaseObject()->velocity(1);
    observation(4) = measured_acceleration(0);
    observation(5) = measured_acceleration(1);

    Eigen::MatrixXd r_matrix;
    r_matrix.setIdentity(6, 6);
    r_matrix.topLeftCorner(2, 2) = measurement->GetBaseObject()
                                       ->center_uncertainty.topLeftCorner(2, 2)
                                       .cast<double>();
    r_matrix.block<2, 2>(2, 2) = measurement->GetBaseObject()
                                     ->velocity_uncertainty.topLeftCorner(2, 2)
                                     .cast<double>();

    ADEBUG << "fusion_original_measurement@(" << std::setprecision(10)
           << observation(0) << "," << observation(1) << "," << observation(2)
           << "," << observation(3) << ")";
    ADEBUG << "fusion_original_measurement_covariance@(" << r_matrix(0, 0) << ","
           << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
           << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
           << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

    // Compute pseudo measurement
    Eigen::Vector4d temp_observation = observation.head(4);
    Eigen::Vector4d pseudo_measurement =
        ComputePseudoMeasurement(temp_observation, measurement->GetSensorType());
    observation.head(4) = pseudo_measurement;

    //if converged, store in history data for acceleration calculation
    if (measurement->GetBaseObject()->velocity_converged) {
      UpdateSensorHistory(measurement->GetSensorType(),
                          measurement->GetBaseObject()->velocity.cast<double>(),
                          measurement->GetTimestamp());
    }

    // Adapt noise level to rewarding status. According if lidar and if velocity converged.
    RewardRMatrix(measurement->GetSensorType(),
                  measurement->GetBaseObject()->velocity_converged, &r_matrix);

    ADEBUG << "fusion_pseudo_measurement@(" << std::setprecision(10)
           << observation(0) << "," << observation(1) << "," << observation(2)
           << "," << observation(3) << ")";
    ADEBUG << "fusion_pseudo_measurement_covariance@(" << r_matrix(0, 0) << ","
           << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
           << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
           << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

    ekf_filter_.DeCorrelation(2, 0, 2, 2);
    ekf_filter_.Correct(observation, r_matrix);
    ekf_filter_.CorrectionBreakdown();      */

  ROS_DEBUG_STREAM( "fusion_filter_belief@(" << std::setprecision(10) << ekf_filter_.State()(0)
         << ","                             //  << ekf_filter_.GetStates()(0) << ","
         << ekf_filter_.State()(1) << ","   //  << ekf_filter_.GetStates()(1) << ","
         << ekf_filter_.State()(2) << ","   //<< ekf_filter_.GetStates()(2) << ","
         << ekf_filter_.State()(3) << ")");  //<< ekf_filter_.GetStates()(3) << ")";

  // ADEBUG << "fusion_filter_belief_covariance@("
  //        << ekf_filter_.GetUncertainty()(0, 0) << ","
  //        << ekf_filter_.GetUncertainty()(0, 1) << ","
  //        << ekf_filter_.GetUncertainty()(1, 0) << ","
  //        << ekf_filter_.GetUncertainty()(1, 1) << ","
  //        << ekf_filter_.GetUncertainty()(2, 2) << ","
  //        << ekf_filter_.GetUncertainty()(2, 3) << ","
  //        << ekf_filter_.GetUncertainty()(3, 2) << ","
  //        << ekf_filter_.GetUncertainty()(3, 3) << ")";
}

void EkfMotionFusion::UpdateMotionState() {
  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();
  obj->anchor_point = fused_anchor_point_.cast<double>();
  // it seems that it is the only place to update the FusedObject's `center`
  // who will be used in CollectFusedObjects
  obj->center = obj->anchor_point;
  obj->velocity = fused_velocity_.cast<float>();
  obj->theta = fused_yaw_;
  // obj->acceleration = fused_acceleration_.cast<float>();     //reserve now.

  // Previously, obj velocity uncertainty would be updated according to the
  // uncertainty within kalman filter. however, rewarding strategy within
  // motion fusion could change kalman filter's uncertainty, which could
  // not reflect the reality of cov.
  // TEMPORARYLY, we keep obj's velocity covariance as it computed in
  // single sensor. THIS IS AS SAME AS 2-1-19-1.
  // a more general probabilisitic method would be try in this week, and
  // codes would be updated after benchmarking.
  obj->center_uncertainty = center_uncertainty_;
  obj->velocity_uncertainty = velo_uncertainty_;
  obj->acceleration_uncertainty = acc_uncertainty_;
}

Eigen::VectorXd EkfMotionFusion::ComputeAccelerationMeasurement(const base::SensorType& sensor_type,
                                                                const Eigen::Vector3d& velocity,
                                                                const double& timestamp) {
  Eigen::Vector3d acceleration_measurement = Eigen::Vector3d::Zero();
  if (perception::base::SensorManager::Instance()->IsCamera(sensor_type)) {
    // acceleration_measurement(0) = ekf_filter_.GetStates()(4);
    // acceleration_measurement(1) = ekf_filter_.GetStates()(5);
    return acceleration_measurement;
  }
  if (GetSensorHistoryLength(sensor_type) >= s_eval_window_) {
    size_t history_index = GetSensorHistoryIndex(sensor_type, s_eval_window_);
    if (history_index >= history_velocity_.size()) {
      AERROR << "illegal history index";
      return Eigen::Vector3d::Zero();
    }
    acceleration_measurement = velocity - history_velocity_[history_index];
    acceleration_measurement /= (timestamp - history_timestamp_[history_index]);
  }
  return acceleration_measurement;
}

void EkfMotionFusion::RewardRMatrix(const base::SensorType& sensor_type,
                                    const bool& converged,
                                    Eigen::MatrixXd* r_matrix) {
  perception::base::SensorManager* sensor_manager = perception::base::SensorManager::Instance();
  const float converged_scale = 0.01f;
  const float unconverged_scale = 1000.0f;
  if (sensor_manager->IsLidar(sensor_type)) {
    if (converged) {
      r_matrix->setIdentity();
      r_matrix->block<4, 4>(0, 0) *= converged_scale;
    } else {
      r_matrix->setIdentity();
      r_matrix->block<2, 2>(0, 0) *= converged_scale;
      r_matrix->block<2, 2>(2, 2) *= unconverged_scale;
    }
  } else if (sensor_manager->IsRadar(sensor_type) || sensor_manager->IsCamera(sensor_type)) {
    r_matrix->block<4, 4>(0, 0) *= 2.0;
    int lidar_history_length = GetSensorHistoryLength(base::SensorType::VELODYNE_64);
    if (lidar_history_length > 0) {
      r_matrix->block<2, 2>(2, 2).setIdentity();
      r_matrix->block<2, 2>(2, 2) *= unconverged_scale;
    }
  }
  r_matrix->block<2, 2>(4, 4) *= 0.5;
}

Eigen::Vector4d EkfMotionFusion::ComputePseudoMeasurement(const Eigen::Vector4d& measurement,
                                                          const base::SensorType& sensor_type) {
  // What is a pseudo-lidar estimation? if given lidar estimation could trace
  // a good radar estimation within a short window, then project radar
  // estimation on given lidar one. otherwise, use original lidar estimation.
  // what is a pseudo-radar estimation? if given radar estimation is good,
  // project it on its last good lidar estimation within a short window.
  // otherwise, use current belief
  perception::base::SensorManager* sensor_manager = perception::base::SensorManager::Instance();
  if (sensor_manager->IsLidar(sensor_type)) {
    return ComputePseudoLidarMeasurement(measurement);
  }
  if (sensor_manager->IsRadar(sensor_type)) {
    return ComputePseudoRadarMeasurement(measurement);
  }
  if (sensor_manager->IsCamera(sensor_type)) {
    return ComputePseudoCameraMeasurement(measurement);
  }
  AINFO << "unsupport sensor type for pseudo measurement computation!";
  Eigen::Vector4d pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d EkfMotionFusion::ComputePseudoLidarMeasurement(const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int trace_count = 0;
  const float velocity_angle_change_thresh_ = static_cast<float>(M_PI / 20.0);
  const float acceleration_angle_change_thresh_ = static_cast<float>(M_PI / 3.0);
  Eigen::Vector4d pseudo_measurement = measurement;
  Eigen::Vector3d lidar_velocity = Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  // fused_acceleration(0) = ekf_filter_.GetStates()(4);
  // fused_acceleration(1) = ekf_filter_.GetStates()(5);
  // Return if lidar velocity is already small enough
  double lidar_velocity_norm = lidar_velocity.norm();
  if (lidar_velocity_norm < kDoubleEpsilon) {
    return pseudo_measurement;
  }
  // Trace back radar velocity history, try to find good radar measurement
  // which could help lidar velocity get a more accurate pseudo measurement
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (perception::base::SensorManager::Instance()->IsRadar(history_type)) {
      trace_count++;
      Eigen::Vector3d radar_velocity = history_velocity_[history_index];
      double radar_velocity_norm = radar_velocity.norm();
      // Abandon radar history, if their velocity lengths are too different
      if (fabs(radar_velocity_norm - lidar_velocity_norm) > velocity_length_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its velocity angle change is too large
      double velocity_angle_change = common::CalculateTheta2DXY(radar_velocity, lidar_velocity);
      if (fabs(velocity_angle_change) > velocity_angle_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its acceleration angle change is too large
      Eigen::Vector3d radar_velocity_project_on_lidar_velocity =
          common::Calculate2DXYProjectVector(radar_velocity, lidar_velocity);
      Eigen::Vector3d radar_velocity_project_on_lidar_velocity_gain =
          radar_velocity_project_on_lidar_velocity - lidar_velocity;
      double acceleration_angle_change = common::CalculateTheta2DXY(
          fused_acceleration, radar_velocity_project_on_lidar_velocity_gain);
      if (fabs(acceleration_angle_change) > acceleration_angle_change_thresh_) {
        continue;
      }
      // Compute normalized velocity gain
      double normalized_radar_velocity_project_on_lidar_velocity_gain =
          radar_velocity_project_on_lidar_velocity_gain.head(2).norm() /
          std::max(radar_velocity_project_on_lidar_velocity.head(2).norm(),
                   lidar_velocity.head(2).norm());
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / velocity_angle_change_thresh_;
      // Abandon radar history, if normalized diff is too large
      if (normalized_radar_velocity_project_on_lidar_velocity_gain *
              normalized_velocity_angle_change >
          0.5) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_lidar_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_lidar_velocity(1);
      return pseudo_measurement;
    }
    if (trace_count == s_eval_window_) {
      pseudo_measurement = measurement;
      return pseudo_measurement;
    }
  }
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d EkfMotionFusion::ComputePseudoCameraMeasurement(
    const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int trace_count = 0;
  const float velocity_angle_change_thresh_ = static_cast<float>(M_PI / 10.0);
  const float acceleration_angle_change_thresh_ = static_cast<float>(M_PI / 3.0);
  Eigen::Vector4d pseudo_measurement = measurement;
  Eigen::Vector3d camera_velocity = Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  // fused_acceleration(0) = ekf_filter_.GetStates()(4);
  // fused_acceleration(1) = ekf_filter_.GetStates()(5);
  // Return if camera velocity is already small enough
  double camera_velocity_norm = camera_velocity.norm();
  if (camera_velocity_norm < kDoubleEpsilon) {
    return pseudo_measurement;
  }
  // Trace back radar velocity history, try to find good radar measurement
  // which could help camera velocity get a more accurate pseudo measurement
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (perception::base::SensorManager::Instance()->IsRadar(history_type)) {
      trace_count++;
      Eigen::Vector3d radar_velocity = history_velocity_[history_index];
      double radar_velocity_norm = radar_velocity.norm();
      // Abandon radar history, if their velocity lengths are too different
      if (fabs(radar_velocity_norm - camera_velocity_norm) > velocity_length_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its velocity angle change is too large
      double velocity_angle_change = common::CalculateTheta2DXY(radar_velocity, camera_velocity);
      if (fabs(velocity_angle_change) > velocity_angle_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its acceleration angle change is too large
      Eigen::Vector3d radar_velocity_project_on_camera_velocity =
          common::Calculate2DXYProjectVector(radar_velocity, camera_velocity);
      Eigen::Vector3d radar_velocity_project_on_camera_velocity_gain =
          radar_velocity_project_on_camera_velocity - camera_velocity;
      double acceleration_angle_change = common::CalculateTheta2DXY(
          fused_acceleration, radar_velocity_project_on_camera_velocity_gain);
      if (fabs(acceleration_angle_change) > acceleration_angle_change_thresh_) {
        continue;
      }
      // Compute normalized velocity gain
      double normalized_radar_velocity_project_on_camera_velocity_gain =
          radar_velocity_project_on_camera_velocity_gain.head(2).norm() /
          std::max(radar_velocity_project_on_camera_velocity.head(2).norm(),
                   camera_velocity.head(2).norm());
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / velocity_angle_change_thresh_;
      // Abandon radar history, if normalized diff is too large
      if (normalized_radar_velocity_project_on_camera_velocity_gain *
              normalized_velocity_angle_change >
          0.3) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_camera_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_camera_velocity(1);
      return pseudo_measurement;
    }
    if (trace_count == s_eval_window_) {
      pseudo_measurement = measurement;
      return pseudo_measurement;
    }
  }
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d EkfMotionFusion::ComputePseudoRadarMeasurement(const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int lidar_trace_count = 0;
  int camera_trace_count = 0;
  Eigen::Vector4d pseudo_measurement = measurement;
  // int lidar_camera_history_length =
  //     GetSensorHistoryLength(base::SensorType::VELODYNE_64) +
  //     GetSensorHistoryLength(base::SensorType::CAMERA_6MM);		//org

  // Modify-guoxiaoxiao
  int lidar_history_length = 0, camera_history_length = 0;
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr camera_ptr = track_ref_->GetLatestCameraObject();
  if (lidar_ptr != nullptr) {
    lidar_history_length = GetSensorHistoryLength(lidar_ptr->GetSensorType());
  }
  if (camera_ptr != nullptr) {
    camera_history_length = GetSensorHistoryLength(camera_ptr->GetSensorType());
  }
  int lidar_camera_history_length = lidar_history_length + camera_history_length;

  // Keep motion if lidar & camera history is empty
  if (lidar_camera_history_length == 0) {
    pseudo_measurement(2) = ekf_filter_.State()(2);  // ekf_filter_.GetStates()(2);
    pseudo_measurement(3) = ekf_filter_.State()(3);  // ekf_filter_.GetStates()(3);
    return pseudo_measurement;
  }
  Eigen::Vector3d radar_velocity = Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  // fused_acceleration(0) = ekf_filter_.GetStates()(4);
  // fused_acceleration(1) = ekf_filter_.GetStates()(5);
  perception::base::SensorManager* sensor_manager = perception::base::SensorManager::Instance();
  // Trace back lidar and camera history, try to find good lidar/camera
  // measurement which could help radar velocity get a more robust pseudo
  // measurement.
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (!sensor_manager->IsLidar(history_type) && !sensor_manager->IsCamera(history_type)) {
      continue;
    }
    // Keep motion, if no good history been found
    if (lidar_trace_count == s_eval_window_ || camera_trace_count == s_eval_window_) {
      pseudo_measurement(2) = ekf_filter_.State()(2);  // ekf_filter_.GetStates()(2);
      pseudo_measurement(3) = ekf_filter_.State()(3);  // ekf_filter_.GetStates()(3);
      return pseudo_measurement;
    }
    Eigen::Vector3d history_velocity = history_velocity_[history_index];
    // Abandon history measurement, if its speed is too small
    if (history_velocity.norm() < kDoubleEpsilon) {
      pseudo_measurement(2) = history_velocity(0);
      pseudo_measurement(3) = history_velocity(1);
      return pseudo_measurement;
    }
    // Compute velocity angle change and acceleration angle change
    double velocity_angle_change = common::CalculateTheta2DXY(radar_velocity, history_velocity);
    Eigen::Vector3d radar_velocity_project_on_history_velocity =
        common::Calculate2DXYProjectVector(radar_velocity, history_velocity);
    Eigen::Vector3d radar_velocity_project_on_history_velocity_gain =
        radar_velocity_project_on_history_velocity - history_velocity;
    double acceleration_angle_change = common::CalculateTheta2DXY(
        fused_acceleration, radar_velocity_project_on_history_velocity_gain);
    // Compute normalized velocity gain
    double normalized_radar_velocity_project_on_history_velocity_gain =
        radar_velocity_project_on_history_velocity_gain.head(2).norm() /
        std::max(radar_velocity_project_on_history_velocity.head(2).norm(),
                 history_velocity.head(2).norm());
    // Handle lidar history
    if (sensor_manager->IsLidar(history_type)) {
      lidar_trace_count++;
      // Abandon lidar measurement, if its velocity angle change is too big
      if (fabs(velocity_angle_change) > M_PI / 20) {
        continue;
      }
      // Abandon lidar measurement, if its acceleration angle change is too
      // big
      if (fabs(acceleration_angle_change) > M_PI / 3) {
        continue;
      }
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change = fabs(velocity_angle_change) / (M_PI / 20);
      // Abandon lidar measurement, if normalized diff is too big
      if (normalized_radar_velocity_project_on_history_velocity_gain *
              normalized_velocity_angle_change >
          0.5) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_history_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_history_velocity(1);
      return pseudo_measurement;
    } else {
      ++camera_trace_count;
      // Abandon camera measurement, if its velocity angle change is too big
      if (fabs(velocity_angle_change) > M_PI / 10) {
        continue;
      }
      // Abandon camera measurement, if its acceleration angle change is too
      // big
      if (fabs(acceleration_angle_change) > M_PI / 3) {
        continue;
      }
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change = fabs(velocity_angle_change) / (M_PI / 10);
      // Abandon lidar measurement, if normalized diff is too big
      if (normalized_radar_velocity_project_on_history_velocity_gain *
              normalized_velocity_angle_change >
          0.3) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_history_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_history_velocity(1);
      return pseudo_measurement;
    }
  }
  // Use original measurement, if history is not enough
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

void EkfMotionFusion::UpdateSensorHistory(const base::SensorType& sensor_type,
                                          const Eigen::Vector3d& velocity,
                                          const double& timestamp) {
  // int lidar_history_length =
  //     GetSensorHistoryLength(base::SensorType::VELODYNE_64);
  // int radar_history_length =
  //     GetSensorHistoryLength(base::SensorType::CT_RADAR);		//org

  // Modify-guoxiaoxiao
  int lidar_history_length = 0, radar_history_length = 0;
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  if (lidar_ptr != nullptr) {
    lidar_history_length = GetSensorHistoryLength(lidar_ptr->GetSensorType());
  }
  if (radar_ptr != nullptr) {
    radar_history_length = GetSensorHistoryLength(radar_ptr->GetSensorType());
  }

  if ((lidar_history_length >= s_eval_window_ && radar_history_length >= s_eval_window_) ||
      history_velocity_.size() > s_history_size_maximum_) {
    history_velocity_.pop_front();
    history_timestamp_.pop_front();
    history_sensor_type_.pop_front();
  }
  history_velocity_.push_back(velocity);
  history_timestamp_.push_back(timestamp);
  history_sensor_type_.push_back(sensor_type);
}

int EkfMotionFusion::GetSensorHistoryLength(const base::SensorType& sensor_type) {
  int sensor_history_length = 0;
  for (size_t i = 0; i < history_sensor_type_.size(); ++i) {
    if (history_sensor_type_[i] == sensor_type) {
      ++sensor_history_length;
    }
  }
  return sensor_history_length;
}

int EkfMotionFusion::GetSensorHistoryIndex(const base::SensorType& sensor_type,
                                           const int& trace_length) {
  int history_index = 0;
  int history_count = 0;
  for (size_t i = 1; i <= history_sensor_type_.size(); ++i) {
    history_index = static_cast<int>(history_sensor_type_.size() - i);
    if (history_sensor_type_[history_index] == sensor_type) {
      ++history_count;
    }
    if (history_count == trace_length) {
      return history_index;
    }
  }

  return -1;
}

}  // namespace fusion
}  // namespace perception
