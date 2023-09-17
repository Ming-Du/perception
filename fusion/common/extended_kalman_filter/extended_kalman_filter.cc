#include "extended_kalman_filter.h"
#include <math.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include "extended_kalman_config.h"
// #include "common/log.hpp"

namespace perception {
namespace fusion {

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_V 2
#define INDEX_THETA 3
#define INDEX_OMEGA 4
#define K_EPS 0.0001

ExtendedKalmanFilter::ExtendedKalmanFilter() : is_initialized_(false) {
  omega_threshold_ = 0.5 * M_PI / 2.0;  //= 0.785             //10.0 * M_PI / 16 ;
  mu_omega_d_threshold_ = omega_threshold_ / 0.1;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::init(ExtendedKalmanConfig* config, Eigen::VectorXd& z) {
  M_2_PI_ = 2 * M_PI;
  // CTRV x,y,v,theta,omega,
  x_ = Eigen::VectorXd(5);
  F_ = Eigen::MatrixXd::Identity(5, 5);
  P_ = Eigen::MatrixXd::Identity(5, 5);
  Q_ = Eigen::MatrixXd::Identity(5, 5);
  H_ = Eigen::MatrixXd::Identity(5, 5);
  R_ = Eigen::MatrixXd::Identity(5, 5);
  ExtendedKalmanConfig* conf = (ExtendedKalmanConfig*)config;
  mu_a_ = conf->mu_a;
  mu_omega_d_ = conf->mu_omega_d;
  sigma_a_ = conf->sigma_a;
  sigma_omega_d_ = conf->sigma_omega_d;

  R_(0, 0) = conf->std_px * conf->std_px;
  R_(1, 1) = conf->std_py * conf->std_py;
  R_(2, 2) = conf->std_pv * conf->std_pv;
  R_(3, 3) = conf->std_ptheta * conf->std_ptheta;
  R_(4, 4) = conf->std_pomega * conf->std_pomega;

  P_(0, 0) = conf->px;
  P_(1, 1) = conf->py;
  P_(2, 2) = conf->pv;
  P_(3, 3) = conf->ptheta;
  P_(4, 4) = conf->pomega;

  initializeX(z);
}

void ExtendedKalmanFilter::initializeX(Eigen::VectorXd& z) {
  if (!is_initialized_) {
    x_[0] = z[0];
    x_[1] = z[1];
    x_[2] = z[2];  // 0.0;
    x_[3] = z[3];  // 0.0;
    x_[4] = 0.0;
    is_initialized_ = true;

    is_v_initialized_ = true;
  }
}

void ExtendedKalmanFilter::predict(double dt) {
  if (!is_v_initialized_) {
    return;
  }
  x_ = fx(dt);
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

Eigen::VectorXd ExtendedKalmanFilter::X() {
  return x_;
}

Eigen::VectorXd ExtendedKalmanFilter::State() {
  Eigen::VectorXd x(5);
  x[0] = x_[0];
  x[1] = x_[1];
  double v = x_[2];
  double theta = x_[3];
  x[2] = v * cos(theta);
  x[3] = v * sin(theta);
  x[4] = theta;
  return x;
}

double ExtendedKalmanFilter::control_theta(double theta) {
  if ((theta > M_PI) || (theta < -M_PI)) {
    // theta = fmod(theta, M_2_PI_);
    // if (theta > M_PI)
    //   theta -= M_2_PI_;
    // if (theta < -M_PI)
    //   theta += M_2_PI_;
    while (theta > M_PI)
      theta -= M_2_PI_;
    while (theta < -M_PI)
      theta += M_2_PI_;
  }
  return theta;
}

void ExtendedKalmanFilter::update(double dt, Eigen::VectorXd& z) {
  if (!is_v_initialized_) {
    double dx = z(INDEX_X) - x_(INDEX_X);
    double dy = z(INDEX_Y) - x_(INDEX_Y);
    // x_(INDEX_V) = sqrt(dx * dx + dy * dy) / dt;
    x_(INDEX_V) = z(INDEX_V);          // 0.0;
    x_(INDEX_THETA) = z(INDEX_THETA);  // control_theta(atan2(dy, dx));
    x_(INDEX_OMEGA) = 0.0;
    x_(INDEX_X) = z(INDEX_X);
    x_(INDEX_Y) = z(INDEX_Y);
    is_v_initialized_ = true;
    return;
  }
  double omega_last = x_(INDEX_OMEGA);
  x_(INDEX_OMEGA) = (dt > 0) ? ((z(INDEX_THETA) - x_(INDEX_THETA)) / dt) : omega_threshold_;

  // printf("\033[31m 111 x_(INDEX_OMEGA) is %lf, omega_threshold_ is %lf.\033[0m\n",
  // x_(INDEX_OMEGA), omega_threshold_);

  // x_(INDEX_THETA) = control_theta(x_(INDEX_THETA));
  // std::cout << "x_:\n" << x_ << std::endl;
  if (x_(INDEX_OMEGA) > omega_threshold_) {
    x_(INDEX_OMEGA) = omega_threshold_;
  } else if (x_(INDEX_OMEGA) < -omega_threshold_) {
    x_(INDEX_OMEGA) = -omega_threshold_;
  }

  // printf("\033[31m 222 x_(INDEX_OMEGA) is %lf.\033[0m\n", x_(INDEX_OMEGA));
  ///////syf
  mu_a_ = (dt > 0) ? ((z(INDEX_V) - x_(INDEX_V)) / dt) : mu_a_;
  mu_omega_d_ = (dt > 0) ? ((x_(INDEX_OMEGA) - omega_last) / dt) : mu_omega_d_;
  if (mu_omega_d_ > mu_omega_d_threshold_) {
    mu_omega_d_ = mu_omega_d_threshold_;
  } else if (mu_omega_d_ < -mu_omega_d_threshold_) {
    mu_omega_d_ = -mu_omega_d_threshold_;
  }
  ///////syf

  Eigen::VectorXd z_pred = hx(dt);
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd S = H_ * PHt + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = PHt * Si;

  x_ = x_ + K * y;

  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  // std::cout << "P_:\n" << P_ << std::endl;
  // return true;
}

Eigen::VectorXd ExtendedKalmanFilter::fx(double dt) {
  Eigen::VectorXd xp(5);
  double sin_theta = sin(x_(INDEX_THETA));
  double cos_theta = cos(x_(INDEX_THETA));
  double dt_2 = dt * dt;

  if (x_(INDEX_OMEGA) > omega_threshold_) {
    x_(INDEX_OMEGA) = omega_threshold_;
  } else if (x_(INDEX_OMEGA) < -omega_threshold_) {
    x_(INDEX_OMEGA) = -omega_threshold_;
  }
  // double theta_p = control_theta(x_(INDEX_OMEGA) * dt + x_(INDEX_THETA));
  double theta_p = x_(INDEX_OMEGA) * dt + x_(INDEX_THETA);

  // printf("\033[31m x_(INDEX_THETA) is %lf, theta_p is %lf, x_(INDEX_OMEGA) is %lf.\033[0m\n",
  //       x_(INDEX_THETA), theta_p, x_(INDEX_OMEGA));

  // double dtheta = x_(INDEX_OMEGA) * dt;
  // double theta_p = dtheta + x_(INDEX_THETA);
  // f(x)
  if (abs(x_(INDEX_OMEGA)) < K_EPS) {
    // omg == 0
    // compute state variables
    xp(INDEX_X) = x_(INDEX_V) * cos_theta * dt + x_(INDEX_X) + 0.5 * dt_2 * mu_a_ * cos_theta;
    xp(INDEX_Y) = x_(INDEX_V) * sin_theta * dt + x_(INDEX_Y) + 0.5 * dt_2 * mu_a_ * sin_theta;
    xp(INDEX_V) = x_(INDEX_V) + dt * mu_a_;
    xp(INDEX_THETA) = theta_p;
    // + 0.5 * dt_2 * mu_omega_d_;
    xp(INDEX_OMEGA) = x_(INDEX_OMEGA);
    // + dt * mu_omega_d_;

    // compute JA (F_) Matrix
    F_(0, 2) = dt * cos_theta;
    F_(0, 3) = -dt * x_(INDEX_V) * sin_theta;
    F_(0, 4) = 0.0;
    F_(1, 2) = dt * sin_theta;
    F_(1, 3) = dt * x_(INDEX_V) * cos_theta;
    F_(1, 4) = 0.0;
    F_(3, 4) = dt;
  } else {
    // omg != 0
    // compute state variables
    double _1_omega = 1.0 / x_(INDEX_OMEGA);
    double v_omega = x_(INDEX_V) * _1_omega;
    double v_omega_2 = v_omega * _1_omega;
    double dt_v_omega = dt * v_omega;

    double sin_theta_p = sin(theta_p);
    double cos_theta_p = cos(theta_p);

    xp(INDEX_X) =
        v_omega * (sin_theta_p - sin_theta) + x_(INDEX_X) + 0.5 * dt_2 * mu_a_ * cos_theta;
    xp(INDEX_Y) =
        v_omega * (cos_theta - cos_theta_p) + x_(INDEX_Y) + 0.5 * dt_2 * mu_a_ * sin_theta;
    xp(INDEX_V) = x_(INDEX_V) + dt * mu_a_;
    xp(INDEX_THETA) = theta_p + 0.5 * dt_2 * mu_omega_d_;
    xp(INDEX_OMEGA) = x_(INDEX_OMEGA) + dt * mu_omega_d_;

    // compute JA (F_) Matrix
    F_(0, 2) = _1_omega * (sin_theta_p - sin_theta);
    F_(0, 3) = _1_omega * (cos_theta_p - cos_theta);
    F_(0, 4) = dt_v_omega * cos_theta_p - v_omega_2 * (sin_theta_p - sin_theta);
    F_(1, 2) = _1_omega * (cos_theta - cos_theta_p);
    F_(1, 3) = v_omega * (sin_theta_p - sin_theta);
    F_(1, 4) = dt_v_omega * sin_theta_p - v_omega_2 * (cos_theta - cos_theta_p);
    F_(3, 4) = dt;
  }

  // compute Q
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double sigma_a_2 = sigma_a_ * sigma_a_;
  double sigma_omega_d_2 = sigma_omega_d_ * sigma_omega_d_;

  Q_(0, 0) = 0.25 * dt_4 * sigma_a_2 * cos_theta * cos_theta;
  Q_(0, 1) = 0.25 * dt_4 * sigma_a_2 * sin_theta * cos_theta;
  Q_(0, 2) = 0.5 * dt_3 * sigma_a_2 * cos_theta;
  Q_(1, 0) = Q_(0, 0);
  Q_(1, 1) = 0.25 * dt_4 * sigma_a_2 * sin_theta * sin_theta;
  Q_(1, 2) = 0.5 * dt_3 * sigma_a_2 * sin_theta;
  Q_(2, 0) = Q_(0, 2);
  Q_(2, 1) = Q_(1, 2);
  Q_(2, 2) = dt_2 * sigma_a_2;
  Q_(3, 3) = 0.25 * dt_4 * sigma_omega_d_2;
  Q_(3, 4) = 0.5 * dt_3 * sigma_omega_d_2;
  Q_(4, 3) = Q_(3, 4);
  Q_(4, 4) = dt_2 * sigma_omega_d_2;
  // xp(INDEX_THETA) = control_theta(xp(INDEX_THETA));
  return xp;
}

Eigen::VectorXd ExtendedKalmanFilter::hx(double dt) {
  Eigen::VectorXd zp(5);
  zp(INDEX_X) = x_(INDEX_X);
  zp(INDEX_Y) = x_(INDEX_Y);
  zp(INDEX_V) = x_(INDEX_V);
  zp(INDEX_THETA) = x_(INDEX_THETA);
  zp(INDEX_OMEGA) = x_(INDEX_OMEGA);
  return zp;
}

}  // namespace fusion
}  // namespace perception