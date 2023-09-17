#pragma once

#include <vector>

#include "Eigen/Dense"

namespace perception {
namespace common {

class AdaptiveKalmanFilter {
 public:
  AdaptiveKalmanFilter();

  bool Init(const Eigen::VectorXd &initial_belief_states,
            const Eigen::MatrixXd &initial_uncertainty);

  // @brief predict the current state and uncertainty of system
  // @params[IN] transform_matrix: transform the state from the
  //             pre moment to current moment
  // @params[IN] env_uncertainty_matrix: the uncertainty brought by
  //             the environment when predict.
  bool Predict(const Eigen::MatrixXd &transform_matrix,
               const Eigen::MatrixXd &env_uncertainty_matrix);

  // @brief use the current observation to correct the predict
  // @params[IN] cur_observation: the observationin in current time
  // @params[IN] cur_observation_uncertainty: the uncertainty of
  //             the observation in current time.
  bool Correct(const Eigen::VectorXd &cur_observation,
               const Eigen::MatrixXd &cur_observation_uncertainty);

  // @brief set the control matrix
  bool SetControlMatrix(const Eigen::MatrixXd &control_matrix);

  // @brief get the system states
  Eigen::VectorXd GetStates() const;

  // @brief get the belief uncertainty
  Eigen::MatrixXd GetUncertainty() const;
  bool DeCorrelation(int x, int y, int x_len, int y_len);
  void CorrectionBreakdown();
  bool SetGainBreakdownThresh(const std::vector<bool> &break_down,
                              const float threshold = 2.0f);
  bool SetValueBreakdownThresh(const std::vector<bool> &break_down,
                               const float threshold = 0.05f);

 private:
  bool init_;

  // @brief the name of the filter
  std::string name_;

  // @brief the number of the system states
  int states_num_;

  Eigen::MatrixXd transform_matrix_;
  Eigen::VectorXd global_states_;
  Eigen::MatrixXd global_uncertainty_;
  Eigen::MatrixXd env_uncertainty_;
  Eigen::MatrixXd cur_observation_;
  Eigen::MatrixXd cur_observation_uncertainty_;
  Eigen::MatrixXd c_matrix_;

  // @brief kalman gain
  Eigen::VectorXd prior_global_states_;
  Eigen::VectorXd gain_break_down_;
  Eigen::VectorXd value_break_down_;
  Eigen::MatrixXd kalman_gain_;

  float value_break_down_threshold_ = 999.0f;
  float gain_break_down_threshold_ = 0.0f;
};

}  // namespace common
}  // namespace perception
