#pragma once
#include <Eigen/Dense>
#include "extended_kalman_config.h"

namespace perception {
namespace fusion {

class ExtendedKalmanFilter
// : public KalmanFilter
{
 public:
  ExtendedKalmanFilter();
  ~ExtendedKalmanFilter();
  void init(ExtendedKalmanConfig* config, Eigen::VectorXd& z);
  void predict(double dt);
  void update(double dt, Eigen::VectorXd& z);
  Eigen::VectorXd X();
  Eigen::VectorXd State();
  Eigen::VectorXd fx(double dt);
  Eigen::VectorXd hx(double dt);

 private:
  void initializeX(Eigen::VectorXd& z);
  double control_theta(double theta);

 private:
  bool is_initialized_;
  bool is_v_initialized_;
  bool is_omega_initialized_;
  double dt_;
  Eigen::VectorXd x2_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  double omega_threshold_;
  double mu_omega_d_threshold_;

  double mu_a_;
  double mu_omega_d_;
  double sigma_a_;
  double sigma_omega_d_;
  double M_2_PI_;
};

}  // namespace fusion
}  // namespace perception