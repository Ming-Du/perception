#pragma once

#include <deque>
#include <string>
#include <vector>

#include "common/unscented_kalman_filter/unscented_kalman_filter.h"
#include "lib/interface/base_motion_fusion.h"

namespace perception {
namespace fusion {

class UkfMotionFusion : public BaseMotionFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit UkfMotionFusion(TrackPtr track) : BaseMotionFusion(track) {}
  ~UkfMotionFusion() = default;

  UkfMotionFusion(const UkfMotionFusion&) = delete;
  UkfMotionFusion& operator=(const UkfMotionFusion&) = delete;

  // @brief init kalman filter and some magic number
  bool Init() override;

  // @brief update the tracker with current measurement
  // @params[IN] measurement: sensor results
  // @params[IN] target_timestamp: tracker timestamp
  void UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                             double target_timestamp) override;

  // @brief update the tracker only use time diff
  // @params[IN] sensor_id
  // @params[IN] measurement_timestamp
  // @params[IN] target_timestamp
  void UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override { return "UkfMotionFusion"; }

  void GetStates(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity);

 private:
  bool InitFilter(const SensorObjectConstPtr& sensor_object);

  void MotionFusionWithoutMeasurement(const double time_diff);
  void MotionFusionWithMeasurement(const SensorObjectConstPtr& measurement, double time_diff);

  // Update state
  void UpdateMotionState();

  // @brief We use the history sensor information
  //         to compute the acceleration.
  // @params[IN] sensor_type: which sensor type we
  //            need in history queue
  // @params[IN] velocity: the current velocity
  // @params[IN] timestamp: the current timestamp
  // @return result acceleration
  Eigen::VectorXd ComputeAccelerationMeasurement(const base::SensorType& sensor_type,
                                                 const Eigen::Vector3d& velocity,
                                                 const double& timestamp);

  void UpdateSensorHistory(const base::SensorType& sensor_type,
                           const Eigen::Vector3d& velocity,
                           const double& timestamp);
  // @brief reward r matrix according coming sensor type and converge status
  // @params[IN] sensor_type: sensor type of coming measurement
  // @params[IN] converged: converge status of coming measurement
  void RewardRMatrix(const base::SensorType& sensor_type,
                     const bool& converged,
                     Eigen::MatrixXd* r_matrix);
  // @brief compute pseudo measurement
  // @params[IN] measurement: original measurement
  // @params[IN] sensor_type: sensor type of coming measurement
  // @params[OUT] has_good_radar: whether good radar has been found or not
  // @return pseudo measurement of coming measurement
  Eigen::Vector4d ComputePseudoMeasurement(const Eigen::Vector4d& measurement,
                                           const base::SensorType& sensor_type);

  // @brief compute pseudo lidar measurement
  // @params[IN] measurement: original lidar measurement
  // @return pseudo measurement of coming lidar measurement
  Eigen::Vector4d ComputePseudoLidarMeasurement(const Eigen::Vector4d& measurement);
  // @brief compute pseudo camera measurement
  // @params[IN] measurement: original camera measurement
  // @return pseudo measurement of coming camera measurement
  Eigen::Vector4d ComputePseudoCameraMeasurement(const Eigen::Vector4d& measurement);
  // @brief compute pseudo radar measurement
  // @params[IN] measurement: original radar measurement
  // @return pseudo measurement of coming radar measurement
  Eigen::Vector4d ComputePseudoRadarMeasurement(const Eigen::Vector4d& measurement);

  int GetSensorHistoryLength(const base::SensorType& sensor_type);
  int GetSensorHistoryIndex(const base::SensorType& sensor_type, const int& trace_length);

  void CorrectionGainBreakdown();

 private:
  bool filter_init_ = false;
  std::deque<Eigen::Vector3d> history_velocity_;
  std::deque<double> history_timestamp_;
  std::deque<base::SensorType> history_sensor_type_;
  UnscentendKalmanFilter ukf_filter_;
  UnscentedKalmanConfig ukf_filter_config_;
  Eigen::Vector3d fused_anchor_point_;
  Eigen::Vector3d fused_velocity_;
  Eigen::Vector3d fused_acceleration_;
  Eigen::Vector3d prior_measured_acceleration_;

  Eigen::Matrix3f center_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f velo_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f acc_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Vector3d measured_acceleration_ = Eigen::Vector3d::Zero();

  static int s_eval_window_;
  static size_t s_history_size_maximum_;
  double velocity_length_change_thresh_ = 5.0f;  // diff < 5 m/s

  float gain_break_down_threshold_ = 2.0f;
};

}  // namespace fusion
}  // namespace perception
