#pragma once

#include <deque>
#include <string>
#include <vector>

#include "common/standard_kalman_filter/kalman_config.h"
#include "common/standard_kalman_filter/kalman_filter.h"
#include "lib/interface/base_motion_fusion.h"

namespace perception {
namespace fusion {
// CV model：constant velocity. Modify with jiangnan
class CVKalmanMotionFusion : public BaseMotionFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit CVKalmanMotionFusion(TrackPtr track) : BaseMotionFusion(track) {}
  ~CVKalmanMotionFusion() = default;

  CVKalmanMotionFusion(const CVKalmanMotionFusion&) = delete;
  CVKalmanMotionFusion& operator=(const CVKalmanMotionFusion&) = delete;

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

  std::string Name() const override { return "KalmamnMotionFusion"; }

  void GetStates(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity);
  void InitKalmanConfig(KalmanConfig kalmanconfig);

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

  void RewardRMatrix(Eigen::MatrixXd* r_matrix);

  int GetSensorHistoryLength(const base::SensorType& sensor_type);
  int GetSensorHistoryIndex(const base::SensorType& sensor_type, const int& trace_length);
  // Modify @jiangnan:
  void ObservationErrorToGlobal(float& use_error_x,
                                float& use_error_y,
                                const float local_error_x,
                                const float local_error_y);


  int CheckMeasurementNoiseState(const SensorObjectConstPtr &measurement);

 private:
  bool filter_init_ = false;
  bool has_measurement_ = false;
  bool is_lidar_rb_ = false;
  int noise_state_ = -1;
  double last_tracked_time_ = 0.0;
  std::deque<Eigen::Vector3d> history_velocity_;
  std::deque<double> history_timestamp_;
  std::deque<base::SensorType> history_sensor_type_;
  KalmanFilter kalman_filter_;
  KalmanConfig kalman_config_;
  Eigen::Vector3d fused_position_;
  Eigen::Vector3d fused_velocity_;
  Eigen::Vector3d fused_acceleration_;
  Eigen::Vector3d fused_acceleration_ego_;
  Eigen::Vector3d measured_acceleration_;

  //   Eigen::Matrix3f center_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f position_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f velo_uncertainty_ = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f acc_uncertainty_ = Eigen::Matrix3f::Zero();

  static float host_yaw_;
  static Eigen::Vector3d host_position_;
  static int s_eval_window_;
  static size_t s_history_size_maximum_;
  double velocity_length_change_thresh_ = 5.0f;  // diff < 5 m/s
};

}  // namespace fusion
}  // namespace perception
