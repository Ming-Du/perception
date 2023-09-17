#include "kalman_filter.h"
#include "common/include/log.h"

namespace perception {
namespace fusion {

KalmanFilter::KalmanFilter() : BaseFilter("KalmanFilter") {}

bool KalmanFilter::Init(const Eigen::VectorXd& initial_belief_states,
                        const Eigen::MatrixXd& initial_uncertainty) {
  if (initial_uncertainty.rows() != initial_uncertainty.cols()) {
    ROS_ERROR("Init: the cols and rows of uncertainty martix should be equal!");
    return false;
  }
  states_num_ = static_cast<int>(initial_uncertainty.rows());

  if (states_num_ <= 0) {
    ROS_ERROR("Init: state_num should be greater than zero!");
    return false;
  }

  if (states_num_ != initial_belief_states.rows()) {
    ROS_ERROR("Init: the rows of state should be equal to state_num!");
    return false;
  }

  global_states_ = initial_belief_states;
  global_uncertainty_ = initial_uncertainty;
  prior_global_states_ = global_states_;

  transform_matrix_.setIdentity(states_num_, states_num_);
  cur_observation_.setZero(states_num_, 1);
  cur_observation_uncertainty_.setIdentity(states_num_, states_num_);

  c_matrix_.setIdentity(states_num_, states_num_);
  env_uncertainty_.setZero(states_num_, states_num_);

  gain_break_down_.setZero(states_num_, 1);
  value_break_down_.setZero(states_num_, 1);

  kalman_gain_.setZero(states_num_, states_num_);
  init_ = true;
  return true;
}

bool KalmanFilter::Predict(const Eigen::MatrixXd& transform_matrix,
                           const Eigen::MatrixXd& env_uncertainty_matrix) {
  if (!init_) {
    ROS_ERROR("Predict: Kalman Filter initialize not successfully!");
    return false;
  }
  if (transform_matrix.rows() != states_num_) {
    ROS_ERROR("Predict: the rows of transform matrix should be equal to state_num!");
    return false;
  }
  if (transform_matrix.cols() != states_num_) {
    ROS_ERROR("Predict: the cols of transform matrix should be equal to state_num!");
    return false;
  }
  if (env_uncertainty_matrix.rows() != states_num_) {
    ROS_ERROR("Predict: the rows of env uncertainty should be equal to state_num!");
    return false;
  }
  if (env_uncertainty_matrix.cols() != states_num_) {
    ROS_ERROR("Predict: the cols of env uncertainty should be equal to state_num!");
    return false;
  }
  transform_matrix_ = transform_matrix;
  env_uncertainty_ = env_uncertainty_matrix;
  // X_ = F * X
  global_states_ = transform_matrix_ * global_states_;
  // P_ = F * P * F_t + Q
  global_uncertainty_ =
      transform_matrix_ * global_uncertainty_ * transform_matrix_.transpose() + env_uncertainty_;
  return true;
}

bool KalmanFilter::Correct(const Eigen::VectorXd& cur_observation,
                           const Eigen::MatrixXd& cur_observation_uncertainty) {
  if (!init_) {
    ROS_ERROR("Correct: Kalman Filter initialize not successfully!");
    return false;
  }
  if (cur_observation.rows() != states_num_) {
    ROS_ERROR("Correct: the rows of current observation should be equal to state_num!");
    return false;
  }
  if (cur_observation_uncertainty.rows() != states_num_) {
    ROS_ERROR(
        "Correct: the rows of current observation uncertainty "
        "should be equal to state_num!");
    return false;
  }
  if (cur_observation_uncertainty.cols() != states_num_) {
    ROS_ERROR(
        "Correct: the cols of current observation uncertainty "
        "should be equal to state_num!");
    return false;
  }

  cur_observation_ = cur_observation;
  cur_observation_uncertainty_ = cur_observation_uncertainty;
  // K = P_ * H_t * ( H * P_ * H_t + R )^-1
  kalman_gain_ =
      global_uncertainty_ * c_matrix_.transpose() *
      (c_matrix_ * global_uncertainty_ * c_matrix_.transpose() + cur_observation_uncertainty_)
          .inverse();
  // X = X_ + K * ( Z - H * X_ )
  global_states_ = global_states_ + kalman_gain_ * (cur_observation_ - c_matrix_ * global_states_);
  Eigen::MatrixXd tmp_identity;
  tmp_identity.setIdentity(states_num_, states_num_);
  // P = (I - K * H) * P_ * (I - K * H)_t + K * R * K_t
  global_uncertainty_ = (tmp_identity - kalman_gain_ * c_matrix_) * global_uncertainty_ *
                            (tmp_identity - kalman_gain_ * c_matrix_).transpose() +
                        kalman_gain_ * cur_observation_uncertainty_ * kalman_gain_.transpose();
  return true;
}

bool KalmanFilter::SetControlMatrix(const Eigen::MatrixXd& control_matrix) {
  if (!init_) {
    ROS_ERROR("SetControlMatrix: Kalman Filter initialize not successfully!");
    return false;
  }
  if (control_matrix.rows() != states_num_ || control_matrix.cols() != states_num_) {
    ROS_ERROR("SetControlMatrix: the rows/cols of control matrix should be equal to state_num!");
    return false;
  }
  c_matrix_ = control_matrix;
  return true;
}

Eigen::VectorXd KalmanFilter::GetStates() const {
  return global_states_;
}

Eigen::MatrixXd KalmanFilter::GetUncertainty() const {
  return global_uncertainty_;
}

bool KalmanFilter::SetGainBreakdownThresh(const std::vector<bool>& break_down,
                                          const float threshold) {
  if (static_cast<int>(break_down.size()) != states_num_) {
    return false;
  }
  for (int i = 0; i < states_num_; i++) {
    if (break_down[i]) {
      gain_break_down_(i) = 1;
    }
  }
  gain_break_down_threshold_ = threshold;
  return true;
}

bool KalmanFilter::SetValueBreakdownThresh(const std::vector<bool>& break_down,
                                           const float threshold) {
  if (static_cast<int>(break_down.size()) != states_num_) {
    return false;
  }
  for (int i = 0; i < states_num_; i++) {
    if (break_down[i]) {
      value_break_down_(i) = 1;
    }
  }
  value_break_down_threshold_ = threshold;
  return true;
}
void KalmanFilter::CorrectionBreakdown() {
  Eigen::VectorXd states_gain = global_states_ - prior_global_states_;
  Eigen::VectorXd breakdown_diff = states_gain.cwiseProduct(gain_break_down_);
  // acceleration gain greater than 2, then normalize to 2.
  global_states_ -= breakdown_diff;
  if (breakdown_diff.norm() > gain_break_down_threshold_) {
    breakdown_diff.normalize();
    breakdown_diff *= gain_break_down_threshold_;
  }
  global_states_ += breakdown_diff;

  Eigen::VectorXd temp;
  temp.setOnes(states_num_, 1);
  // if velocity less than 0.05, set velocity 0.
  if ((global_states_.cwiseProduct(value_break_down_)).norm() < value_break_down_threshold_) {
    global_states_ = global_states_.cwiseProduct(temp - value_break_down_);
  }
  prior_global_states_ = global_states_;
}

bool KalmanFilter::DeCorrelation(int x, int y, int x_len, int y_len) {
  if (x >= states_num_ || y >= states_num_ || x + x_len >= states_num_ ||
      y + y_len >= states_num_) {
    return false;
  }
  for (int i = 0; i < x_len; i++) {
    for (int j = 0; j < y_len; j++) {
      global_uncertainty_(x + i, y + j) = 0;
    }
  }
  return true;
}

bool KalmanFilter::CV_DeCorrelation(int x, int y, int x_len, int y_len) {
  // add by jiangnan : Apply to  CV model  motion fusion
  for (int i = 0; i < x_len; i++) {
    for (int j = 0; j < y_len; j++) {
      global_uncertainty_(x + i, y + j) = 0;
    }
  }
  return true;
}

Eigen::VectorXd KalmanFilter::GetObservation() const {
  return cur_observation_;
};

}  // namespace fusion
}  // namespace perception
