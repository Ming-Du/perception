#pragma once

#include <vector>

#include <ros/ros.h>
#include "common/base_filter.h"
#include "unscented_kalman_config.h"

namespace perception {
namespace fusion {

class UnscentendKalmanFilter : public BaseFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  UnscentendKalmanFilter();

  bool Init(const Eigen::VectorXd& initial_belief_states,
            const Eigen::MatrixXd& initial_uncertainty) {
    return true;
  }

  bool Init(UnscentedKalmanConfig* config,
            const Eigen::VectorXd& initial_belief_states,
            const Eigen::MatrixXd& initial_uncertainty);

  // @brief predict the current state and uncertainty of system
  // @params[IN] transform_matrix: transform the state from the
  //             pre moment to current moment
  // @params[IN] env_uncertainty_matrix: the uncertainty brought by
  //             the environment when predict.
  bool Predict(const Eigen::MatrixXd& transform_matrix,
               const Eigen::MatrixXd& env_uncertainty_matrix);

  /**
  @brief: predicts sigma points, the state, and the state covariance
  @params[IN]: time_diff the change in time (in seconds) between the last
  @author:liuxinyu
  **/
  bool Predict(const double time_diff);

  // @brief: Calculate augmented sigma points: global_states_sig_aug
  // @author:liuxinyu
  void AugmentedSigmaPoints();

  // @brief: Predict the sigma points: states_sig_pred_
  // @author:liuxinyu
  void SigmaPointPrediction(const double time_diff);

  // @brief: Predict Mean and Covariance of the predicted state: global_states_ and
  // global_uncertainty_
  // @author:liuxinyu
  void PredictMeanAndCovariance();

  // @brief use the current observation to correct the predict
  // @params[IN] cur_observation: the observationin in current time
  // @params[IN] cur_observation_uncertainty: the uncertainty of
  //             the observation in current time.
  bool Correct(const Eigen::VectorXd& cur_observation,
               const Eigen::MatrixXd& cur_observation_uncertainty);

  // @brief: Predict either lidar or radar or camera measurement with given Sigma predictions
  // @author:liuxinyu
  void computeMeanAndCovariance(const Eigen::MatrixXd& cur_observation_uncertainty);

  // @brief: Updates the state with either lidar or radar or camera measurement
  // @author:liuxinyu
  void updateState(const Eigen::VectorXd& cur_observation);

  // @brief set the control matrix
  bool SetControlMatrix(const Eigen::MatrixXd& control_matrix);

  // @brief get the system states
  Eigen::VectorXd GetStates() const;

  // @brief get the belief uncertainty
  Eigen::MatrixXd GetUncertainty() const;

  bool SetValueBreakdownThresh(const std::vector<bool>& break_down, const float threshold = 0.05f);

  // bool DeCorrelation(int x, int y, int x_len, int y_len);
  void CorrectionValueBreakdown();

 private:
  Eigen::VectorXd global_states;
  Eigen::MatrixXd global_uncertainty;
  // @brief kalman gain
  Eigen::VectorXd prior_global_states_;
  Eigen::VectorXd value_break_down_;
  Eigen::MatrixXd kalman_gain_;

  // create augmented mean vector
  Eigen::VectorXd global_states_aug_;
  // create augmented state covariance
  Eigen::MatrixXd global_uncertainty_aug_;
  // create sigma point matrix
  Eigen::MatrixXd global_states_sig_aug;
  // predicted sigma points matrix
  Eigen::MatrixXd states_sig_pred_;
  // Weights of sigma points
  Eigen::VectorXd weights_;

  Eigen::MatrixXd observation_sig;
  // mean predicted measurement
  Eigen::VectorXd observation_pred;
  // measurement covariance matrix S
  Eigen::MatrixXd s_matrix;
  // create matrix for cross correlation Tc
  Eigen::MatrixXd t_matrix;

  // Augmented state dimension
  int n_aug_ = 7;
  // Sigma point spreading parameter
  double lambda_;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  double uncertainty_std_a_;  // debug-guoxiaoxiao  org:2.0   1.4 = 1.2
  // Process noise standard deviation yaw acceleration in rad/s^2
  double uncertainty_std_yawdd_;  // debug-guoxiaoxiao  org:1.0  0.7  = 0.6

  float value_break_down_threshold_ = 999.0f;
};

}  // namespace fusion
}  // namespace perception
