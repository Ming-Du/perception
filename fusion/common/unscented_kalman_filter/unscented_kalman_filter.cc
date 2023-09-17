
#include "unscented_kalman_filter.h"
#include "common/include/log.h"

namespace perception {
namespace fusion {

UnscentendKalmanFilter::UnscentendKalmanFilter() : BaseFilter("UnscentendKalmanFilter") {}

bool UnscentendKalmanFilter::Init(UnscentedKalmanConfig* config,
                                  const Eigen::VectorXd& initial_belief_states,
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
  // Modify(@liuxinyu):
  UnscentedKalmanConfig* conf = (UnscentedKalmanConfig*)config;
  uncertainty_std_a_ = conf->std_a;
  uncertainty_std_yawdd_ = conf->std_yawdd;

  value_break_down_.setZero(states_num_, 1);

  // Define spreading parameter
  lambda_ = 3 - n_aug_;

  global_states_ = initial_belief_states;
  global_uncertainty_ = initial_uncertainty;
  prior_global_states_ = global_states_;

  global_states.setZero(states_num_, 1);
  global_uncertainty.setZero(states_num_, states_num_);

  global_states_aug_.setZero(n_aug_, 1);
  // create augmented state covariance
  global_uncertainty_aug_.setIdentity(n_aug_, n_aug_);
  // create sigma point matrix
  global_states_sig_aug.setZero(n_aug_, 2 * n_aug_ + 1);

  // Matrix to hold sigma points
  states_sig_pred_ = Eigen::MatrixXd(states_num_, 2 * n_aug_ + 1);

  // Vector for weights
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (size_t i = 1; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // create matrix for sigma points in measurement space
  observation_sig.setZero(states_num_, 2 * n_aug_ + 1);

  observation_pred.setZero(states_num_, 1);

  // innovation covariance matrix S
  s_matrix.setZero(states_num_, states_num_);

  t_matrix.setZero(states_num_, states_num_);

  kalman_gain_.setZero(states_num_, states_num_);

  init_ = true;
  return true;
}

bool UnscentendKalmanFilter::Predict(const Eigen::MatrixXd& transform_matrix,
                                     const Eigen::MatrixXd& env_uncertainty_matrix) {
  return true;
}

void UnscentendKalmanFilter::AugmentedSigmaPoints() {
  // create augmented mean state
  global_states_aug_.fill(0.0);
  global_states_aug_.head(states_num_) = global_states_;

  // create augmented covariance matrix
  global_uncertainty_aug_.fill(0.0);
  global_uncertainty_aug_.topLeftCorner(states_num_, states_num_) = global_uncertainty_;
  global_uncertainty_aug_(5, 5) = std::pow(uncertainty_std_a_, 2);
  global_uncertainty_aug_(6, 6) = std::pow(uncertainty_std_yawdd_, 2);

  // create square root matrix
  Eigen::MatrixXd cholesky_L_ = global_uncertainty_aug_.llt().matrixL();

  // create augmented sigma points
  global_states_sig_aug.fill(0.0);
  global_states_sig_aug.col(0) = global_states_aug_;
  for (size_t i = 0; i < n_aug_; ++i) {
    global_states_sig_aug.col(i + 1) =
        global_states_aug_ + std::sqrt(lambda_ + n_aug_) * cholesky_L_.col(i);
    global_states_sig_aug.col(i + 1 + n_aug_) =
        global_states_aug_ - std::sqrt(lambda_ + n_aug_) * cholesky_L_.col(i);
  }
}

void UnscentendKalmanFilter::SigmaPointPrediction(const double time_diff) {
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {
    // extract values for better readability
    double state_px = global_states_sig_aug(0, i);
    double state_py = global_states_sig_aug(1, i);
    double state_v = global_states_sig_aug(2, i);
    double state_yaw = global_states_sig_aug(3, i);  // (-PI, PI)
    double state_yawd = global_states_sig_aug(4, i);
    double state_a = global_states_sig_aug(5, i);
    double state_yawdd = global_states_sig_aug(6, i);

    // predicted state values
    double state_px_p, state_py_p;

    // avoid division by zero
    if (std::fabs(state_yawd) > 0.001) {
      state_px_p =
          state_px + state_v / state_yawd *
                         (std::sin(state_yaw + state_yawd * time_diff) - std::sin(state_yaw));
      state_py_p =
          state_py + state_v / state_yawd *
                         (std::cos(state_yaw) - std::cos(state_yaw + state_yawd * time_diff));
    } else {
      state_px_p = state_px + state_v * time_diff * std::cos(state_yaw);
      state_py_p = state_py + state_v * time_diff * std::sin(state_yaw);
    }

    double state_v_p = state_v;
    double state_yaw_p = state_yaw + state_yawd * time_diff;
    double state_yawd_p = state_yawd;

    // add noise
    state_px_p = state_px_p + 0.5 * state_a * std::pow(time_diff, 2) * std::cos(state_yaw);
    state_py_p = state_py_p + 0.5 * state_a * std::pow(time_diff, 2) * std::sin(state_yaw);
    state_v_p = state_v_p + state_a * time_diff;

    state_yaw_p = state_yaw_p + 0.5 * state_yawdd * std::pow(time_diff, 2);
    state_yawd_p = state_yawd_p + state_yawdd * time_diff;

    // write predicted sigma point into right column
    states_sig_pred_(0, i) = state_px_p;
    states_sig_pred_(1, i) = state_py_p;
    states_sig_pred_(2, i) = state_v_p;
    states_sig_pred_(3, i) = state_yaw_p;
    states_sig_pred_(4, i) = state_yawd_p;
  }
}

void UnscentendKalmanFilter::PredictMeanAndCovariance() {
  // predicted state mean
  global_states.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    global_states = global_states + weights_(i) * states_sig_pred_.col(i);
  }

  // predicted state covariance matrix
  global_uncertainty.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    Eigen::VectorXd x_diff = states_sig_pred_.col(i) - global_states;
    global_uncertainty = global_uncertainty + weights_(i) * x_diff * x_diff.transpose();
  }
  global_states_ = global_states;
  global_uncertainty_ = global_uncertainty;
}

/**
 *  Estimate the object's location. Modify the state
 *  vector, x_. Predict sigma points, the state, and the state covariance matrix.
 *  1.generate augmented sigma point
 *  2.predict sigma points
 *  3.Predicted Mean and Covariance
 */
bool UnscentendKalmanFilter::Predict(const double time_diff) {
  if (!init_) {
    ROS_ERROR("Predict: Unscented Kalman Filter initialize not successfully!");
    return false;
  }

  AugmentedSigmaPoints();
  SigmaPointPrediction(time_diff);
  PredictMeanAndCovariance();

  return true;
}

void UnscentendKalmanFilter::computeMeanAndCovariance(
    const Eigen::MatrixXd& cur_observation_uncertainty) {
  //// ######## 1、transform sigma points into measurement space #########

  // transform sigma points into measurement space
  observation_sig.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // measurement model
    for (size_t j = 0; j < states_num_; ++j) {
      observation_sig(j, i) = states_sig_pred_(j, i);
    }
  }

  //// ######## 2、mean and covariance #########
  // mean predicted measurement
  observation_pred.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {
    observation_pred += weights_(i) * observation_sig.col(i);
  }

  // innovation covariance matrix S
  s_matrix.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = observation_sig.col(i) - observation_pred;
    s_matrix += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  s_matrix += cur_observation_uncertainty;
}

void UnscentendKalmanFilter::updateState(const Eigen::VectorXd& cur_observation) {
  //// ######## 3、update #########

  // calculate cross correlation matrix
  t_matrix.fill(0.0);
  for (size_t i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = observation_sig.col(i) - observation_pred;

    // state difference
    Eigen::VectorXd x_diff = states_sig_pred_.col(i) - global_states_;

    t_matrix += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  kalman_gain_ = t_matrix * s_matrix.inverse();

  // residual
  Eigen::VectorXd z_diff_ = cur_observation - observation_pred;

  // update state mean and covariance matrix
  global_states_ += kalman_gain_ * z_diff_;
  global_uncertainty_ -= kalman_gain_ * s_matrix * kalman_gain_.transpose();
}

bool UnscentendKalmanFilter::Correct(const Eigen::VectorXd& cur_observation,
                                     const Eigen::MatrixXd& cur_observation_uncertainty) {
  if (!init_) {
    ROS_ERROR("Correct: Unscented Kalman Filter initialize not successfully!");
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

  computeMeanAndCovariance(cur_observation_uncertainty);
  updateState(cur_observation);

  return true;
}

bool UnscentendKalmanFilter::SetControlMatrix(const Eigen::MatrixXd& control_matrix) {
  if (!init_) {
    ROS_ERROR("SetControlMatrix: Unscentend Kalman Filter initialize not successfully!");
    return false;
  }
  if (control_matrix.rows() != states_num_ || control_matrix.cols() != states_num_) {
    ROS_ERROR("SetControlMatrix: the rows/cols of control matrix should be equal to state_num!");
    return false;
  }
  c_matrix_ = control_matrix;
  return true;
}

Eigen::VectorXd UnscentendKalmanFilter::GetStates() const {
  return global_states_;
}

Eigen::MatrixXd UnscentendKalmanFilter::GetUncertainty() const {
  return global_uncertainty_;
}

bool UnscentendKalmanFilter::SetValueBreakdownThresh(const std::vector<bool>& break_down,
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

void UnscentendKalmanFilter::CorrectionValueBreakdown() {
  Eigen::VectorXd temp;
  temp.setOnes(states_num_, 1);
  // if velocity less than 0.05, set velocity 0.
  if ((global_states_.cwiseProduct(value_break_down_)).norm() < value_break_down_threshold_) {
    global_states_ = global_states_.cwiseProduct(temp - value_break_down_);
  }

  prior_global_states_ = global_states_;
}

// bool UnscentendKalmanFilter::DeCorrelation(int x, int y, int x_len, int y_len) {
//   if (x >= states_num_ || y >= states_num_ || x + x_len >= states_num_ ||
//       y + y_len >= states_num_) {
//     return false;
//   }
//   for (int i = 0; i < x_len; i++) {
//     for (int j = 0; j < y_len; j++) {
//       global_uncertainty_(x + i, y + j) = 0;
//     }
//   }
//   return true;
// }

}  // namespace fusion
}  // namespace perception
