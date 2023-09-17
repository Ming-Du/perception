#pragma once
#include <Eigen/Dense>

namespace perception {
namespace fusion {

class UnscentedKalmanConfig {
 public:
  UnscentedKalmanConfig();
  ~UnscentedKalmanConfig();
  static bool InitParams(const char* config);

 public:
  // Process noise standard deviation longitudinal acceleration in m/s^2
  static double std_a;
  // Process noise standard deviation yaw acceleration in rad/s^2
  static double std_yawdd;
  // Laser measurement noise standard deviation position1 in m
  static double std_laspx;
  // Laser measurement noise standard deviation position2 in m
  static double std_laspy;
  // Modify(@liuxinyu): laser measurement noise standard deviation v in m/s
  static double std_lasv;
  // Modify(@liuxinyu): laser measurement noise standard deviation yaw in rad
  static double std_lasyaw;
  // radar measurement noise standard deviation position1 in m
  static double std_rdpx;
  // radar measurement noise standard deviation position2 in m
  static double std_rdpy;
  // Modify(@liuxinyu): laser measurement noise standard deviation v in m/s
  static double std_rdv;
  // Modify(@liuxinyu): laser measurement noise standard deviation yaw in rad
  static double std_rdyaw;
  static double px;
  static double py;
  static double pv;
  static double ptheta;
  static double pomega;
};

}  // namespace fusion
}  // namespace perception