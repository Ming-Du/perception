#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

namespace perception {
namespace fusion {

class ExtendedKalmanConfig
// : public KalmanConfig
{
 public:
  ExtendedKalmanConfig();
  ~ExtendedKalmanConfig();
  static bool InitParams(const char* config);

 public:
  static double mu_a;
  static double mu_omega_d;
  static double sigma_a;
  static double sigma_omega_d;
  static double std_px;
  static double std_py;
  static double std_pv;
  static double std_ptheta;
  static double std_pomega;
  static double px;
  static double py;
  static double pv;
  static double ptheta;
  static double pomega;
};

}  // namespace fusion
}  // namespace perception