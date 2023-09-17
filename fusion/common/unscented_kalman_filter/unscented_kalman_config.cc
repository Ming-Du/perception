#include "unscented_kalman_config.h"
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

namespace perception {
namespace fusion {

#define INIT_PARAMETER(node, name, type, default_val)                            \
  if (node[#name].IsDefined()) {                                                 \
    name = node[#name].as<type>();                                               \
  } else {                                                                       \
    name = (default_val);                                                        \
    std::cout << "there is no key <" << #name << "> in yaml file." << std::endl; \
  }
double UnscentedKalmanConfig::std_a = 0.8;
double UnscentedKalmanConfig::std_yawdd = 0.55;
double UnscentedKalmanConfig::std_laspx = 0.15;
double UnscentedKalmanConfig::std_laspy = 0.15;
double UnscentedKalmanConfig::std_lasv = 3;
double UnscentedKalmanConfig::std_lasyaw = 0.1;
double UnscentedKalmanConfig::std_rdpx = 0.15;
double UnscentedKalmanConfig::std_rdpy = 0.15;
double UnscentedKalmanConfig::std_rdv = 0;
double UnscentedKalmanConfig::std_rdyaw = 0.1;
double UnscentedKalmanConfig::px = 0.1;
double UnscentedKalmanConfig::py = 0.1;
double UnscentedKalmanConfig::pv = 1.8;
double UnscentedKalmanConfig::ptheta = 0.02;
double UnscentedKalmanConfig::pomega = 1;

UnscentedKalmanConfig::UnscentedKalmanConfig() {}

UnscentedKalmanConfig::~UnscentedKalmanConfig() {}

bool UnscentedKalmanConfig::InitParams(const char* config) {
  if (config != nullptr) {
    std::string config_path(config);
    YAML::Node config_yaml;
    std::string yaml_path = config_path + "/fusion_track_ukf.yaml";
    config_yaml = YAML::LoadFile(yaml_path);
    if (config_yaml.IsNull()) {
      std::cerr << "There is no config file for ukf tracker." << std::endl;
      return false;
    }
    INIT_PARAMETER(config_yaml, std_a, double, 0.8)
    INIT_PARAMETER(config_yaml, std_yawdd, double, 0.55)
    INIT_PARAMETER(config_yaml, std_laspx, double, 0.15)
    INIT_PARAMETER(config_yaml, std_laspy, double, 0.15)
    INIT_PARAMETER(config_yaml, std_lasv, double, 3.0)
    INIT_PARAMETER(config_yaml, std_lasyaw, double, 0.1)
    INIT_PARAMETER(config_yaml, std_rdpx, double, 0.15)
    INIT_PARAMETER(config_yaml, std_rdpy, double, 0.15)
    INIT_PARAMETER(config_yaml, std_rdv, double, 0.0)
    INIT_PARAMETER(config_yaml, std_rdyaw, double, 0.1)
    INIT_PARAMETER(config_yaml, px, double, 0.1)
    INIT_PARAMETER(config_yaml, py, double, 0.1)
    INIT_PARAMETER(config_yaml, pv, double, 1.8)
    INIT_PARAMETER(config_yaml, ptheta, double, 0.02)
    INIT_PARAMETER(config_yaml, pomega, double, 1)
  } else {
    std_a = 0.8;
    std_yawdd = 0.55;
    std_laspx = 0.15;
    std_laspy = 0.15;
    std_lasv = 3;
    std_lasyaw = 0.1;
    std_rdpx = 0.15;
    std_rdpy = 0.15;
    std_rdv = 0;
    std_rdyaw = 0.1;
    px = 0.1;
    py = 0.1;
    pv = 1.8;
    ptheta = 0.02;
    pomega = 1;
  }
  return true;
}

}  // namespace fusion
}  // namespace perception