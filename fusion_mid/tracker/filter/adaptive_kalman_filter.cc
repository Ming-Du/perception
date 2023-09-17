#include "perception/fusion_mid/tracker/filter/adaptive_kalman_filter.h"

namespace perception {
namespace mid_fusion {

double AdaptiveKalmanFilter::s_q_matrix_ratio_ = 0.074;

AdaptiveKalmanFilter::AdaptiveKalmanFilter() { name_ = "AdaptiveKalmanFilter"; }
AdaptiveKalmanFilter::~AdaptiveKalmanFilter() {}
void AdaptiveKalmanFilter::Init(const mid_fusion::Object& object) {
  belief_anchor_point_ = object.center;
  belief_velocity_ = object.velocity.cast<double>();
  a_matrix_.setIdentity();
  // initialize states to the states of the detected obstacle
  posteriori_state_(0) = belief_anchor_point_(0);
  posteriori_state_(1) = belief_anchor_point_(1);
  posteriori_state_(2) = belief_velocity_(0);
  posteriori_state_(3) = belief_velocity_(1);
  priori_state_ = posteriori_state_;
  q_matrix_.setIdentity();
  q_matrix_ *= s_q_matrix_ratio_;
  r_matrix_.setIdentity();
  r_matrix_.topLeftCorner(2, 2) =
      object.center_uncertainty.topLeftCorner(2, 2).cast<double>();
  r_matrix_.block<2, 2>(2, 2) =
      object.velocity_uncertainty.topLeftCorner(2, 2).cast<double>();
  p_matrix_.setIdentity();
  p_matrix_.topLeftCorner(2, 2) =
      object.center_uncertainty.topLeftCorner(2, 2).cast<double>();
  p_matrix_.block<2, 2>(2, 2) =
      object.velocity_uncertainty.topLeftCorner(2, 2).cast<double>();
  c_matrix_.setIdentity();
}
Eigen::VectorXd AdaptiveKalmanFilter::Predict(const double time_diff) {
  Eigen::VectorXd state;
  state.resize(4);
  state[0] = belief_anchor_point_[0] + belief_velocity_[0] * time_diff;
  state[1] = belief_anchor_point_[1] + belief_velocity_[1] * time_diff;
  state[2] = belief_velocity_[0];
  state[3] = belief_velocity_[1];
  return state;
}
Eigen::VectorXd AdaptiveKalmanFilter::UpdateWithObject(
    const mid_fusion::Object& new_object, double time_diff) {
  // predict and then correct
  a_matrix_.setIdentity();
  a_matrix_(0, 2) = time_diff;
  a_matrix_(1, 3) = time_diff;
  priori_state_ = a_matrix_ * posteriori_state_;
  // std::cout << "priori_state_:" << "\n" << priori_state_.matrix() << std::endl;
  p_matrix_ = ((a_matrix_ * p_matrix_) * a_matrix_.transpose()) + q_matrix_;
  belief_anchor_point_ = new_object.center;
  belief_velocity_ = new_object.velocity.cast<double>();
  Eigen::Vector4d measurement;
  measurement(0) = belief_anchor_point_(0);
  measurement(1) = belief_anchor_point_(1);
  measurement(2) = belief_velocity_(0);
  measurement(3) = belief_velocity_(1);
  r_matrix_.setIdentity();
  // std::cout << "r_matrix_:" << "\n" << r_matrix_.matrix() << std::endl;
  r_matrix_.topLeftCorner(2, 2) =
      new_object.center_uncertainty.topLeftCorner(2, 2).cast<double>();
  r_matrix_.block<2, 2>(2, 2) =
      new_object.velocity_uncertainty.topLeftCorner(2, 2).cast<double>();
  // std::cout << "r_matrix_:" << "\n" << r_matrix_.matrix() << std::endl;
  k_matrix_ =
      p_matrix_ * c_matrix_.transpose() *
      (c_matrix_ * p_matrix_ * c_matrix_.transpose() + r_matrix_).inverse();
  Eigen::Vector4d predict_measurement(priori_state_(0), priori_state_(1),
                                      priori_state_(2), priori_state_(3));
  // std::cout << "c_matrix_:" << "\n" << c_matrix_.matrix() << std::endl;
  // std::cout << "k_matrix_:" << "\n" << r_matrix_.matrix() << std::endl;
  posteriori_state_ =
      priori_state_ + k_matrix_ * (measurement - predict_measurement);
  p_matrix_ =
      (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_) * p_matrix_ *
          (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_).transpose() +
      k_matrix_ * r_matrix_ * k_matrix_.transpose();
  // std::cout << "p_matrix_:" << "\n" << p_matrix_.matrix() << std::endl;
  // std::cout << "posteriori_state_:" << "\n" << posteriori_state_.matrix() << std::endl;
  belief_anchor_point_(0) = posteriori_state_(0);
  belief_anchor_point_(1) = posteriori_state_(1);
  belief_velocity_(0) = posteriori_state_(2);
  belief_velocity_(1) = posteriori_state_(3);
  Eigen::VectorXd state;
  state.resize(4);
  state[0] = belief_anchor_point_[0];
  state[1] = belief_anchor_point_[1];
  state[2] = belief_velocity_[0];
  state[3] = belief_velocity_[1];
  return state;
}

void AdaptiveKalmanFilter::GetState(Eigen::Vector3d* anchor_point,
                                    Eigen::Vector3d* velocity) {
  if (anchor_point == nullptr) {
    ROS_ERROR_STREAM("anchor_point is not available");
    return;
  }
  if (velocity == nullptr) {
    ROS_ERROR_STREAM("velocity is not available");
    return;
  }
  *anchor_point = belief_anchor_point_;
  *velocity = belief_velocity_;
}

}  // namespace mid_fusion
}  // namespace perception
