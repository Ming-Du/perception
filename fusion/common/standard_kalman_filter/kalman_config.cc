#include "kalman_config.h"
#include <math.h>
#include <ros/ros.h>
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

float KalmanConfig::converged_scale = 6;
float KalmanConfig::converged_scale_lidar_px = 60.0;  // lidar  px py
float KalmanConfig::converged_scale_lidar_py = 60.0;
float KalmanConfig::converged_scale_lidar_vx = 75.0;  // lidar  vx vy
float KalmanConfig::converged_scale_lidar_vy = 75.0;
float KalmanConfig::converged_scale_falcon_px = 45.0;  // falcon  lidar  px py
float KalmanConfig::converged_scale_falcon_py = 45.0;
float KalmanConfig::converged_scale_falcon_vx = 60.0;  // falcon  lidar  vx vy
float KalmanConfig::converged_scale_falcon_vy = 60.0;
float KalmanConfig::converged_scale_radar_px = 1000;  // radar  px py
float KalmanConfig::converged_scale_radar_py = 1000;
float KalmanConfig::converged_scale_radar_vx = 50.0;  // radar  vx vy
float KalmanConfig::converged_scale_radar_vy = 90.0;
float KalmanConfig::converged_scale_vidar_px = 45;  // vidar  px py
float KalmanConfig::converged_scale_vidar_py = 50;
float KalmanConfig::converged_scale_vidar_vx = 50.0;  // vidar  vx vy
float KalmanConfig::converged_scale_vidar_vy = 90.0;
float KalmanConfig::converged_scale_camera = 500.0;
float KalmanConfig::converged_scale_bsm_p = 2;
float KalmanConfig::converged_scale_bsm_v = 2;
float KalmanConfig::converged_scale_v2i_ssm_p = 2;
float KalmanConfig::converged_scale_v2i_ssm_v = 2;
float KalmanConfig::converged_scale_v2n_rsm_p = 2;
float KalmanConfig::converged_scale_v2n_rsm_v = 2;
float KalmanConfig::converged_scale_v2n_rsi_p = 5;
float KalmanConfig::converged_scale_v2n_rsi_v = 5;
float KalmanConfig::unconverged_scale = 10000.0;

KalmanConfig::KalmanConfig() {}

KalmanConfig::~KalmanConfig() {}

bool KalmanConfig::InitParams(const char* config) {
  if (config != nullptr) {
    std::string config_path(config);
    YAML::Node config_yaml;
    std::string yaml_path = config_path + "/fusion_track_cv.yaml";
    config_yaml = YAML::LoadFile(yaml_path);
    if (config_yaml.IsNull()) {
      std::cerr << "There is no config file for cv motion fusion." << std::endl;
      ROS_ERROR("InitParams: There is no config file for cv motion fusion.");
      return false;
    }

    INIT_PARAMETER(config_yaml, converged_scale, float, 6.0)
    INIT_PARAMETER(config_yaml, converged_scale_lidar_px, float, 60.0)
    INIT_PARAMETER(config_yaml, converged_scale_lidar_py, float, 60.0)
    INIT_PARAMETER(config_yaml, converged_scale_lidar_vx, float, 75.0)
    INIT_PARAMETER(config_yaml, converged_scale_lidar_vy, float, 75.0)
    INIT_PARAMETER(config_yaml, converged_scale_falcon_px, float, 45.0)
    INIT_PARAMETER(config_yaml, converged_scale_falcon_py, float, 45.0)
    INIT_PARAMETER(config_yaml, converged_scale_falcon_vx, float, 60.0)
    INIT_PARAMETER(config_yaml, converged_scale_falcon_vy, float, 60.0)
    INIT_PARAMETER(config_yaml, converged_scale_radar_px, float, 1000.0)
    INIT_PARAMETER(config_yaml, converged_scale_radar_py, float, 1000.0)
    INIT_PARAMETER(config_yaml, converged_scale_radar_vx, float, 50.0)
    INIT_PARAMETER(config_yaml, converged_scale_radar_vy, float, 90.0)
    INIT_PARAMETER(config_yaml, converged_scale_vidar_px, float, 45.0)
    INIT_PARAMETER(config_yaml, converged_scale_vidar_py, float, 50.0)
    INIT_PARAMETER(config_yaml, converged_scale_vidar_vx, float, 50.0)
    INIT_PARAMETER(config_yaml, converged_scale_vidar_vy, float, 90.0)
    INIT_PARAMETER(config_yaml, converged_scale_camera, float, 500.0)
    INIT_PARAMETER(config_yaml, converged_scale_bsm_p, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_bsm_v, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2i_ssm_p, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2i_ssm_v, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2n_rsm_p, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2n_rsm_v, float, 2.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2n_rsi_p, float, 5.0)
    INIT_PARAMETER(config_yaml, converged_scale_v2n_rsi_v, float, 5.0)
    INIT_PARAMETER(config_yaml, unconverged_scale, float, 1000.0)
  } else {
    converged_scale = 6.0;
    converged_scale_lidar_px = 60.0;  // lidar  px py
    converged_scale_lidar_py = 60.0;
    converged_scale_lidar_vx = 75.0;  // lidar  vx vy
    converged_scale_lidar_vy = 75.0;
    converged_scale_falcon_px = 45.0;  // lidar  px py
    converged_scale_falcon_py = 45.0;
    converged_scale_falcon_vx = 60.0;  // lidar  vx vy
    converged_scale_falcon_vy = 60.0;
    converged_scale_radar_px = 1000.0; // radar  px py
    converged_scale_radar_py = 1000.0;
    converged_scale_radar_vx = 50.0; // radar  vx vy
    converged_scale_radar_vy = 90.0;
    converged_scale_camera = 500.0;
    converged_scale_bsm_p = 2.0;
    converged_scale_bsm_v = 2.0;
    converged_scale_v2i_ssm_p = 2.0;
    converged_scale_v2i_ssm_v = 2.0;
    converged_scale_v2n_rsm_p = 2.0;
    converged_scale_v2n_rsm_v = 2.0;
    converged_scale_v2n_rsi_p = 5.0;
    converged_scale_v2n_rsi_v = 5.0;
    unconverged_scale = 1000.0;
  }
  return true;
}

}  // namespace fusion
}  // namespace perception