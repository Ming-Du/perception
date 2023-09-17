#include "CV_kalman_motion_fusion.h"

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

int CVKalmanMotionFusion::s_eval_window_ = 3;
float CVKalmanMotionFusion::host_yaw_ = 0.0;
Eigen::Vector3d CVKalmanMotionFusion::host_position_ = Eigen::Vector3d(0, 0, 0);

size_t CVKalmanMotionFusion::s_history_size_maximum_ = 20;

bool CVKalmanMotionFusion::Init() {
  if (track_ref_ == nullptr) {
    return false;
  }

  if (track_ref_->GetLatestFalconLidarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestFalconLidarObject());
  } else if (track_ref_->GetLatestLidarObject() != nullptr &&
             !(track_ref_->GetLatestLidarObject() ->GetBaseObject() ->is_lidar_rb)) {
    filter_init_ = InitFilter(track_ref_->GetLatestLidarObject());
  } else if (track_ref_->GetLatestRadarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestRadarObject());
  } else if (track_ref_->GetLatestVidarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestVidarObject());
  } else if(track_ref_->GetLatestLidarObject() != nullptr){
    filter_init_ = InitFilter(track_ref_->GetLatestLidarObject());
  } else if(track_ref_->GetLatestObuObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestObuObject());
  }

  return true;
}

bool CVKalmanMotionFusion::InitFilter(const SensorObjectConstPtr& sensor_object) {
  // modify by jiangnan
  const std::vector<bool> gain_break_down = {0, 0, 0, 0};
  const std::vector<bool> value_break_down = {0, 0, 1, 1};
  const float gain_break_down_threshold = 2.0f;
  const float value_break_down_threshold = 0.05f;
  Eigen::MatrixXd global_uncertainty;
  Eigen::VectorXd global_states;
  // global_states: center(2), velocity(2), delete acceleration(2) by jiangnan
  global_uncertainty.setIdentity(4, 4);
  global_states.setZero(4, 1);

//Modify@jiangnan： set P0
  global_uncertainty(2, 2) = 25;
  global_uncertainty(3, 3) = 25;

  // Modify @jiangnan: set state of measurement value
  noise_state_ = -1;
  is_lidar_rb_ = false;

//Modify @jiangnan : update tracked_time  
  last_tracked_time_ = sensor_object->GetTimestamp();

  // Modify @jiangnan : reset host_yaw
  host_yaw_ = sensor_object->GetBaseObject()->host_yaw;
  host_position_ = sensor_object->GetBaseObject()->host_position;
  fused_position_ = sensor_object->GetBaseObject()->position.cast<double>();
  fused_velocity_ = sensor_object->GetBaseObject()->velocity.cast<double>();
  fused_acceleration_ = Eigen::Vector3d(0, 0, 0);
  fused_acceleration_ego_ = Eigen::Vector3d(0, 0, 0);
  measured_acceleration_ = Eigen::Vector3d(0, 0, 0);
  global_states(0) = sensor_object->GetBaseObject()->position(0);
  global_states(1) = sensor_object->GetBaseObject()->position(1);
  global_states(2) = sensor_object->GetBaseObject()->velocity(0);
  global_states(3) = sensor_object->GetBaseObject()->velocity(1);
  // global_uncertainty.topLeftCorner(2, 2) =
  //     sensor_object->GetBaseObject()->center_uncertainty.topLeftCorner(2, 2).cast<double>();
  // global_uncertainty.block<2, 2>(2, 2) =
  //     sensor_object->GetBaseObject()->velocity_uncertainty.topLeftCorner(2, 2).cast<double>();
  if (sensor_object->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(sensor_object->GetSensorType(),
                        sensor_object->GetBaseObject()->velocity.cast<double>(),
                        sensor_object->GetTimestamp());
  }
  // modify by jiangnan
  if (!kalman_filter_.Init(global_states, global_uncertainty)) {
    return false;
  }
  if (!kalman_filter_.SetGainBreakdownThresh(gain_break_down, gain_break_down_threshold) ||
      !kalman_filter_.SetValueBreakdownThresh(value_break_down, value_break_down_threshold)) {
    return false;
  }
  return true;
}

void CVKalmanMotionFusion::GetStates(Eigen::Vector3d* position, Eigen::Vector3d* velocity) {
  *position = fused_position_;
  *velocity = fused_velocity_;
}

void CVKalmanMotionFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                                    double measurement_timestamp,
                                                    double target_timestamp) {


  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr falcon_lidar_ptr = track_ref_->GetLatestFalconLidarObject();
  SensorObjectConstPtr obu_ptr = track_ref_->GetLatestObuObject();
  SensorObjectConstPtr vidar_ptr = track_ref_->GetLatestVidarObject();

  bool is_alive =
      (lidar_ptr != nullptr || radar_ptr != nullptr || obu_ptr != nullptr ||
       vidar_ptr != nullptr || falcon_lidar_ptr != nullptr);

  if (filter_init_ && is_alive) {
    // double time_diff = measurement_timestamp - track_ref_->GetFusedObject()->GetTimestamp();
    double time_diff = measurement_timestamp - last_tracked_time_;

    if (track_ref_->GetLatestRadarObject() != nullptr) {
        // now use radar acceration to update fused_acceration
        fused_acceleration_ego_ = track_ref_->GetLatestRadarObject()->GetBaseObject()->acceleration_ego.cast<double>();
        fused_acceleration_ = track_ref_->GetLatestRadarObject()->GetBaseObject()->acceleration.cast<double>();
    } else {
        // when there is no radar, set the acceleration to 0
        fused_acceleration_ego_ = Eigen::Vector3d(0, 0, 0);
        fused_acceleration_ = Eigen::Vector3d(0, 0, 0);
    }

    MotionFusionWithoutMeasurement(time_diff);
    last_tracked_time_ = measurement_timestamp;

    fused_position_(0) = kalman_filter_.GetStates()(0);
    fused_position_(1) = kalman_filter_.GetStates()(1);
    fused_velocity_(0) = kalman_filter_.GetStates()(2);
    fused_velocity_(1) = kalman_filter_.GetStates()(3);
    // fused_acceleration_ = measured_acceleration_;
  }
  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  has_measurement_ = false;
  UpdateMotionState();
}


void CVKalmanMotionFusion::UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                                                 double target_timestamp) {

  // double time_diff = measurement->GetTimestamp() - track_ref_->GetFusedObject()->GetTimestamp();
  double time_diff = measurement->GetTimestamp() - last_tracked_time_;

  bool is_lidar = IsLidar(measurement);
  bool is_radar = IsRadar(measurement);
  bool is_obu = IsObu(measurement);
  bool is_vidar = IsVidar(measurement);
  bool is_falcon_lidar = IsFalconLidar(measurement);
  SensorObjectConstPtr obu_ptr = track_ref_->GetLatestObuObject();
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr vidar_ptr = track_ref_->GetLatestVidarObject();

  SensorObjectConstPtr falcon_lidar_ptr = track_ref_->GetLatestFalconLidarObject();

  if (measurement->GetBaseObject()->is_lidar_rb) {
    is_lidar_rb_ = true;
  } else {
    is_lidar_rb_ = false;
  }

  // Motion fusion
  if (is_lidar || is_radar || is_obu || is_vidar || is_falcon_lidar) {
    if (filter_init_) {
      MotionFusionWithMeasurement(measurement, time_diff);
      last_tracked_time_ = measurement->GetTimestamp();
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

  fused_position_(0) = kalman_filter_.GetStates()(0);
  fused_position_(1) = kalman_filter_.GetStates()(1);
  fused_velocity_(0) = kalman_filter_.GetStates()(2);
  fused_velocity_(1) = kalman_filter_.GetStates()(3);

  // fused_acceleration_ = measured_acceleration_;
  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  has_measurement_ = true;
  UpdateMotionState();
}

// Modify by jiangnan
void CVKalmanMotionFusion::MotionFusionWithoutMeasurement(const double time_diff) {
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;
  transform_matrix.setIdentity(4, 4);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  env_uncertainty.setZero(4, 4);
  kalman_filter_.Predict(transform_matrix, env_uncertainty);
}

void CVKalmanMotionFusion::MotionFusionWithMeasurement(const SensorObjectConstPtr& measurement,
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
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;

  transform_matrix.setIdentity(4, 4); 
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  env_uncertainty.setIdentity(4, 4);
//  env_uncertainty *= 0.7;

  // Modify@jiangnan： set Q
  env_uncertainty(0, 0) = 0.6;
  env_uncertainty(1, 1) = 0.6;
  env_uncertainty(2, 2) = 0.8;
  env_uncertainty(3, 3) = 0.8;

  // add by jiangnan
  kalman_filter_.Predict(transform_matrix, env_uncertainty);

  // measured_acceleration_ = ComputeAccelerationMeasurement(
  //     measurement->GetSensorType(), measurement->GetBaseObject()->velocity.cast<double>(),
  //     measurement->GetTimestamp());

  // Modify @jiangnan:use radar acceleration
  if (IsRadar(measurement)) {
    fused_acceleration_ego_ = measurement->GetBaseObject()->acceleration_ego.cast<double>();
    fused_acceleration_ = measurement->GetBaseObject()->acceleration.cast<double>();
  } else if (track_ref_->GetLatestRadarObject() != nullptr) {
    fused_acceleration_ego_ = track_ref_->GetLatestRadarObject()->GetBaseObject()->acceleration_ego.cast<double>();
    fused_acceleration_ = track_ref_->GetLatestRadarObject()->GetBaseObject()->acceleration.cast<double>();
  } else { 
    // when there is no radar, set the acceleration to 0
    fused_acceleration_ego_ = Eigen::Vector3d(0, 0, 0);
    fused_acceleration_ = Eigen::Vector3d(0, 0, 0);
  }

  Eigen::Vector4d observation;
  // observation.setZero(4, 1);
  observation(0) = measurement->GetBaseObject()->position(0);
  observation(1) = measurement->GetBaseObject()->position(1);
  observation(2) = measurement->GetBaseObject()->velocity(0);
  observation(3) = measurement->GetBaseObject()->velocity(1);

  Eigen::MatrixXd r_matrix;
  r_matrix.setIdentity(4, 4);

  // r_matrix.topLeftCorner(2, 2) =
  //     measurement->GetBaseObject()->center_uncertainty.topLeftCorner(2, 2).cast<double>();
  // r_matrix.block<2, 2>(2, 2) =
  //     measurement->GetBaseObject()->velocity_uncertainty.topLeftCorner(2, 2).cast<double>();

   
  // if converged, store in history data for acceleration calculation
  if (measurement->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(measurement->GetSensorType(),
                        measurement->GetBaseObject()->velocity.cast<double>(),
                        measurement->GetTimestamp());
  }
  // Adapt noise level to rewarding status. According if lidar and if velocity converged.
  noise_state_ = CheckMeasurementNoiseState(measurement);
  RewardRMatrix( &r_matrix);

  kalman_filter_.CV_DeCorrelation(2, 0, 2, 2);
  kalman_filter_.Correct(observation, r_matrix);
  // add by jiangnan
  kalman_filter_.CorrectionBreakdown();
}

void CVKalmanMotionFusion::UpdateMotionState() {
  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();
  // obj->anchor_point = fused_anchor_point_.cast<double>();
  // it seems that it is the only place to update the FusedObject's `center`
  // who will be used in CollectFusedObjects

  obj->position = fused_position_;
  // Modify @jiangnan : use measurement position
  // object is static and measurement is not lidar_rb
  if (has_measurement_ && obj->is_static && !(is_lidar_rb_)) {
      obj->position[0] = kalman_filter_.GetObservation()(0);
      obj->position[1] = kalman_filter_.GetObservation()(1);
  }
  obj->velocity = fused_velocity_.cast<float>();
  obj->acceleration = fused_acceleration_.cast<float>();
  obj->acceleration_ego = fused_acceleration_ego_.cast<float>();
  obj->center[0] = fused_position_[0] - host_position_[0];
  obj->center[1] = fused_position_[1] - host_position_[1];

  obj->position_uncertainty = position_uncertainty_;
  obj->velocity_uncertainty = velo_uncertainty_;
  obj->acceleration_uncertainty = acc_uncertainty_;
}

Eigen::VectorXd CVKalmanMotionFusion::ComputeAccelerationMeasurement(
    const base::SensorType& sensor_type,
    const Eigen::Vector3d& velocity,
    const double& timestamp) {
  Eigen::Vector3d acceleration_measurement = Eigen::Vector3d::Zero();
  if (GetSensorHistoryLength(sensor_type) >= s_eval_window_) {
    size_t history_index = GetSensorHistoryIndex(sensor_type, s_eval_window_);
    if (history_index >= history_velocity_.size()) {
      ROS_ERROR("ComputeAccelerationMeasurement: illegal history index!");
      return Eigen::Vector3d::Zero();
    }
    acceleration_measurement = velocity - history_velocity_[history_index];
    acceleration_measurement /= (timestamp - history_timestamp_[history_index]);
  }
  return acceleration_measurement;
}

  int CVKalmanMotionFusion::CheckMeasurementNoiseState(const SensorObjectConstPtr &measurement){

  const bool &converged = measurement->GetBaseObject()->velocity_converged;
  const perception::fusion::ObjectSource &source =
      measurement->GetBaseObject()->source;

  double predict_px = kalman_filter_.GetStates()(0);
  double predict_py = kalman_filter_.GetStates()(1);
  double predict_vx = kalman_filter_.GetStates()(2);
  double predict_vy = kalman_filter_.GetStates()(3);

  double measure_px = measurement->GetBaseObject()->position[0];
  double measure_py = measurement->GetBaseObject()->position[1];
  double measure_vx = measurement->GetBaseObject()->velocity[0];
  double measure_vy = measurement->GetBaseObject()->velocity[1];

// calculate residual error of position and velocity
  double position_diff = std::sqrt(std::pow(predict_px - measure_px, 2) +
                                    std::pow(predict_py - measure_py, 2));
  double velocity_diff = std::sqrt(std::pow(predict_vx - measure_vx, 2) +
                                    std::pow(predict_vy - measure_vy, 2));

  int state = -1;

  if (track_ref_->GetTrackedTimes() < 6) {
    // case 0 :  use  the measurement value when the tracking times are too small
    state = 0;
    return state;
  }

  // measurement is falcon_lidar
  if (IsFalconLidar(measurement)) {
    if (velocity_diff < 3) {
      // case 1 : use falcon lidar velocity and position
      state = 1;
    } else {
      // case 2: falcon lidar object's velocity  is  anomaly
      state = 2;
    }
    return state;
  }

  // measurement is lidar
  if (IsLidar(measurement)) {
    // measurement is lidar_ai
    if (!(measurement->GetBaseObject()->is_lidar_rb)) {
      if (velocity_diff < 3) {
        // case 3: use lidar_ai velocity and position
        state = 3;
      } else {
        // case 4: lidar object's velocity  is  anomaly
        state = 4;
      }
      return state;
    } else {
      // measurement is lidar_rb
      if (track_ref_->GetFusedObject()->GetBaseObject()->is_lidar_rb) {
        // case 5 : when fusion object is rb, use lidar_rb position and velocity
        state = 5;
        return state;
      } else {
        // case 6 :  when fusion object is not rb,not use lidar_rb position
        state = 6;
        return state;
      }
    }
  }

  // measurement is radar
  if (IsRadar(measurement)) {
    if (track_ref_->GetLatestLidarObject() == nullptr && track_ref_->GetLatestFalconLidarObject() == nullptr) {
      // use radar position
      state = 7;
    } else {
      // not use radar position
      state = 8;
    }
    return state;
  }

  // measurement is vidar
  if (IsVidar(measurement)) {
    if (track_ref_->GetLatestLidarObject() == nullptr && track_ref_->GetLatestFalconLidarObject() == nullptr
         && track_ref_->GetLatestRadarObject() == nullptr) {
      state = 9;
    } else {
      state = 10;
    }
    return state;
  }

  if (source == perception::fusion::ObjectSource::V2V_BSM) {
    state = 15;
    return state;
  }
  if (source == perception::fusion::ObjectSource::V2N_RSM) {
    state = 16;
    return state;
  }
  if (source == perception::fusion::ObjectSource::V2I_SSM) {
    state = 17;
    return state;
  }
  if (source == perception::fusion::ObjectSource::V2N_RSI) {
    state = 18;
    return state;
  }
  return state;
  }


void CVKalmanMotionFusion::RewardRMatrix(Eigen::MatrixXd *r_matrix) {
  
  // Modify @ jiangnan(202212):Setting observation noise by case
  float local_px_error = 0.0, local_py_error = 0.0, local_vx_error = 0.0,
        local_vy_error = 0.0;
  float utm_px_error = 0.0, utm_py_error = 0.0, utm_vx_error = 0.0,
        utm_vy_error = 0.0;

  switch (noise_state_) {
  case 0:
    local_px_error = kalman_config_.converged_scale;
    local_py_error = kalman_config_.converged_scale;
    local_vx_error = kalman_config_.converged_scale;
    local_vy_error = kalman_config_.converged_scale;
    break;
  case 1:
    local_px_error = kalman_config_.converged_scale_falcon_px;
    local_py_error = kalman_config_.converged_scale_falcon_py;
    local_vx_error = kalman_config_.converged_scale_falcon_vx;
    local_vy_error = kalman_config_.converged_scale_falcon_vy;
    break;
  case 2:
    local_px_error = kalman_config_.converged_scale_falcon_px;
    local_py_error = kalman_config_.converged_scale_falcon_py;
    local_vx_error = kalman_config_.converged_scale_falcon_vx * 6.0;
    local_vy_error = kalman_config_.converged_scale_falcon_vy * 6.0;
    break;
  case 3:
    local_px_error = kalman_config_.converged_scale_lidar_px;
    local_py_error = kalman_config_.converged_scale_lidar_py;
    local_vx_error = kalman_config_.converged_scale_lidar_vx;
    local_vy_error = kalman_config_.converged_scale_lidar_vy;
    break;
  case 4:
    local_px_error = kalman_config_.converged_scale_lidar_px;
    local_py_error = kalman_config_.converged_scale_lidar_py;
    local_vx_error = kalman_config_.converged_scale_lidar_vx * 6.0;
    local_vy_error = kalman_config_.converged_scale_lidar_vy * 6.0;
    break;
  case 5:
    local_px_error = kalman_config_.converged_scale_lidar_px * 1.6;
    local_py_error = kalman_config_.converged_scale_lidar_py * 1.6;
    local_vx_error = kalman_config_.converged_scale_lidar_vx * 2.0;
    local_vy_error = kalman_config_.converged_scale_lidar_vy * 2.0;
    break;
  case 6:
    local_px_error = kalman_config_.converged_scale_lidar_px * 6.0;
    local_py_error = kalman_config_.converged_scale_lidar_py * 6.0;
    local_vx_error = kalman_config_.converged_scale_lidar_vx * 2.0;
    local_vy_error = kalman_config_.converged_scale_lidar_vx * 2.0;
    break;
  case 7:
    local_px_error = kalman_config_.converged_scale;
    local_py_error = kalman_config_.converged_scale;
    local_vx_error = kalman_config_.converged_scale;
    local_vy_error = kalman_config_.converged_scale;
    break;
  case 8:
    local_px_error = kalman_config_.converged_scale_radar_px;
    local_py_error = kalman_config_.converged_scale_radar_py;
    local_vx_error = kalman_config_.converged_scale_radar_vx;
    local_vy_error = kalman_config_.converged_scale_radar_vy;
    break;
  case 9:
    local_px_error = kalman_config_.converged_scale;
    local_py_error = kalman_config_.converged_scale;
    local_vx_error = kalman_config_.converged_scale;
    local_vy_error = kalman_config_.converged_scale;
    break;
  case 10:
    local_px_error = kalman_config_.converged_scale_vidar_px;
    local_py_error = kalman_config_.converged_scale_vidar_py;
    local_vx_error = kalman_config_.converged_scale_vidar_vx;
    local_vy_error = kalman_config_.converged_scale_vidar_vy;
    break;
  case 15:
    local_px_error = kalman_config_.converged_scale_bsm_p;
    local_py_error = kalman_config_.converged_scale_bsm_p;
    local_vx_error = kalman_config_.converged_scale_bsm_v;
    local_vy_error = kalman_config_.converged_scale_bsm_v;
    break;
  case 16:
    local_px_error = kalman_config_.converged_scale_v2n_rsm_p;
    local_py_error = kalman_config_.converged_scale_v2n_rsm_p;
    local_vx_error = kalman_config_.converged_scale_v2n_rsm_v;
    local_vy_error = kalman_config_.converged_scale_v2n_rsm_v;
    break;
  case 17:
    local_px_error = kalman_config_.converged_scale_v2i_ssm_p;
    local_py_error = kalman_config_.converged_scale_v2i_ssm_p;
    local_vx_error = kalman_config_.converged_scale_v2i_ssm_v;
    local_vy_error = kalman_config_.converged_scale_v2i_ssm_v;
    break;
  case 18:
    local_px_error = kalman_config_.converged_scale_v2n_rsi_p;
    local_py_error = kalman_config_.converged_scale_v2n_rsi_p;
    local_vx_error = kalman_config_.converged_scale_v2n_rsi_v;
    local_vy_error = kalman_config_.converged_scale_v2n_rsi_v;
    break;
  default:
    local_px_error = kalman_config_.unconverged_scale;
    local_py_error = kalman_config_.unconverged_scale;
    local_vx_error = kalman_config_.unconverged_scale;
    local_vy_error = kalman_config_.unconverged_scale;
    break;
  }
  ObservationErrorToGlobal(utm_px_error, utm_py_error, local_px_error,
                           local_py_error);
  ObservationErrorToGlobal(utm_vx_error, utm_vy_error, local_vx_error,
                           local_vy_error);

  r_matrix->block<1, 1>(0, 0) *= utm_px_error;
  r_matrix->block<1, 1>(1, 1) *= utm_py_error;
  r_matrix->block<1, 1>(2, 2) *= utm_vx_error;
  r_matrix->block<1, 1>(3, 3) *= utm_vy_error;
}
void CVKalmanMotionFusion::UpdateSensorHistory(const base::SensorType& sensor_type,
                                               const Eigen::Vector3d& velocity,
                                               const double& timestamp) {
  // int lidar_history_length =
  //     GetSensorHistoryLength(base::SensorType::VELODYNE_64);
  // int radar_history_length =
  //     GetSensorHistoryLength(base::SensorType::CT_RADAR);		//org

  // Modify-guoxiaoxiao
  int lidar_history_length = 0, radar_history_length = 0, vidar_history_length = 0,
      falcon_lidar_history_length = 0;
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr vidar_ptr = track_ref_->GetLatestVidarObject();
  SensorObjectConstPtr falcon_lidar_ptr = track_ref_->GetLatestFalconLidarObject();
  if (lidar_ptr != nullptr) {
    lidar_history_length = GetSensorHistoryLength(lidar_ptr->GetSensorType());
  }
  if (radar_ptr != nullptr) {
    radar_history_length = GetSensorHistoryLength(radar_ptr->GetSensorType());
  }
  if (vidar_ptr != nullptr) {
    vidar_history_length = GetSensorHistoryLength(vidar_ptr->GetSensorType());
  }
  if (falcon_lidar_ptr != nullptr) {
    falcon_lidar_history_length = GetSensorHistoryLength(falcon_lidar_ptr->GetSensorType());
  }

  if ((lidar_history_length >= s_eval_window_ && radar_history_length >= s_eval_window_ &&
       vidar_history_length >= s_eval_window_ && falcon_lidar_history_length >= s_eval_window_) ||
      history_velocity_.size() > s_history_size_maximum_) {
    history_velocity_.pop_front();
    history_timestamp_.pop_front();
    history_sensor_type_.pop_front();
  }
  history_velocity_.push_back(velocity);
  history_timestamp_.push_back(timestamp);
  history_sensor_type_.push_back(sensor_type);
}

int CVKalmanMotionFusion::GetSensorHistoryLength(const base::SensorType& sensor_type) {
  int sensor_history_length = 0;
  for (size_t i = 0; i < history_sensor_type_.size(); ++i) {
    if (history_sensor_type_[i] == sensor_type) {
      ++sensor_history_length;
    }
  }
  return sensor_history_length;
}

int CVKalmanMotionFusion::GetSensorHistoryIndex(const base::SensorType& sensor_type,
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

// Modify @jiangnan
void CVKalmanMotionFusion::ObservationErrorToGlobal(float &use_error_x,
                                                    float &use_error_y,
                                                    const float local_error_x,
                                                    const float local_error_y) {
  double cos_host = std::cos(host_yaw_);
  double sin_host = std::sin(host_yaw_);
  // use_error_x = abs(local_error_x * cos_host - local_error_y * sin_host);
  // use_error_y = abs(local_error_x * sin_host + local_error_y * cos_host);
  // Modify @jiangnan : Appropriate amplification of observation noise
  use_error_x = abs(local_error_x * cos_host) + abs(local_error_y * sin_host);
  use_error_y = abs(local_error_x * sin_host) + abs(local_error_y * cos_host);
}

} // namespace fusion
} // namespace perception
