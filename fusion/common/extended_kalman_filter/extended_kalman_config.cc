#include "extended_kalman_config.h"
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
// #include "common/log.hpp"

namespace perception {
namespace fusion {

#define INIT_PARAMETER(node, name, type, default_val)                            \
  if (node[#name].IsDefined()) {                                                 \
    name = node[#name].as<type>();                                               \
  } else {                                                                       \
    name = (default_val);                                                        \
    std::cout << "there is no key <" << #name << "> in yaml file." << std::endl; \
  }

// ExtendedKalmanConfig::ExtendedKalmanConfig() {
// }
double ExtendedKalmanConfig::mu_a = 0.0;
double ExtendedKalmanConfig::mu_omega_d = 0.0;
double ExtendedKalmanConfig::sigma_a = 10.8;
double ExtendedKalmanConfig::sigma_omega_d = 10.1;
double ExtendedKalmanConfig::std_px = 3.1;
double ExtendedKalmanConfig::std_py = 3.1;
double ExtendedKalmanConfig::std_pv = 3.1;
double ExtendedKalmanConfig::std_ptheta = 3.1;
double ExtendedKalmanConfig::std_pomega = 3.1;
double ExtendedKalmanConfig::px = 0.0;
double ExtendedKalmanConfig::py = 0.0;
double ExtendedKalmanConfig::pv = 0.0;
double ExtendedKalmanConfig::ptheta = 0.0;
double ExtendedKalmanConfig::pomega = 0.0;

ExtendedKalmanConfig::ExtendedKalmanConfig() {}

ExtendedKalmanConfig::~ExtendedKalmanConfig() {}

bool ExtendedKalmanConfig::InitParams(const char* config) {
  if (config != nullptr) {
    std::string config_path(config);
    YAML::Node config_yaml;
    std::string yaml_path = config_path + "/fusion_track_ekf.yaml";
    config_yaml = YAML::LoadFile(yaml_path);
    if (config_yaml.IsNull()) {
      std::cerr << "There is no config file for ekf tracker." << std::endl;
      return false;
    }

    INIT_PARAMETER(config_yaml, mu_a, double, 0.0)
    INIT_PARAMETER(config_yaml, mu_omega_d, double, 0.0)
    INIT_PARAMETER(config_yaml, sigma_a, double, 10.8)
    INIT_PARAMETER(config_yaml, sigma_omega_d, double, 10.1)
    INIT_PARAMETER(config_yaml, std_px, double, 3.1)
    INIT_PARAMETER(config_yaml, std_py, double, 3.1)
    INIT_PARAMETER(config_yaml, std_pv, double, 3.1)
    INIT_PARAMETER(config_yaml, std_ptheta, double, 3.1)
    INIT_PARAMETER(config_yaml, std_pomega, double, 3.1)
    INIT_PARAMETER(config_yaml, px, double, 0.0)
    INIT_PARAMETER(config_yaml, py, double, 0.0)
    INIT_PARAMETER(config_yaml, pv, double, 0.0)
    INIT_PARAMETER(config_yaml, ptheta, double, 0.0)
    INIT_PARAMETER(config_yaml, pomega, double, 0.0)

    // std::cout << "EKF Config: \nmu_a = " << mu_a
    //     << "\nmu_omega_d = " << mu_omega_d
    //     << "\nsigma_a = " << sigma_a
    //     << "\nsigma_omega_d = " << sigma_omega_d
    //     << "\nv = " << std_px
    //     << "\nstd_py = " << std_py
    //     << "\nstd_pv = " << std_pv
    //     << "\nstd_ptheta = " << std_ptheta
    //     << "\nstd_pomega = " << std_pomega
    //     << "\npx = " << px
    //     << "\npy = " << py
    //     << "\npv = " << pv
    //     << "\nptheta = " << ptheta
    //     << "\npomega = " << pomega << std::endl;
  } else {
    mu_a = 0.0;
    mu_omega_d = 0.0;
    sigma_a = 10.8;
    sigma_omega_d = 10.1;
    std_px = 3.1;
    std_py = 3.1;
    std_pv = 3.1;
    std_ptheta = 3.1;
    std_pomega = 3.1;
    px = 0.0;
    py = 0.0;
    pv = 0.0;
    ptheta = 0.0;
    pomega = 0.0;
  }
  return true;
}

}  // namespace fusion
}  // namespace perception