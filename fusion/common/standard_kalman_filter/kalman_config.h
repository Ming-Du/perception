#pragma once
#include <Eigen/Dense>

namespace perception {
namespace fusion {

class KalmanConfig
// : public KalmanConfig
{
 public:
  KalmanConfig();
  ~KalmanConfig();
  static bool InitParams(const char* config);
  //  static bool InitParams();

 public:
  static float converged_scale;
  static float converged_scale_lidar_px;  // lidar  px py
  static float converged_scale_lidar_py;
  static float converged_scale_lidar_vx;  // lidar  vx vy
  static float converged_scale_lidar_vy;
  static float converged_scale_falcon_px;  // falcon lidar  px py
  static float converged_scale_falcon_py;
  static float converged_scale_falcon_vx;  // falcon lidar  vx vy
  static float converged_scale_falcon_vy;
  static float converged_scale_radar_px;  // radar  px py
  static float converged_scale_radar_py;
  static float converged_scale_radar_vx;  // radar  vx vy
  static float converged_scale_radar_vy;
  static float converged_scale_vidar_px;  // vidar  px py
  static float converged_scale_vidar_py;
  static float converged_scale_vidar_vx;  // vidar  vx vy
  static float converged_scale_vidar_vy;
  static float converged_scale_camera;
  static float converged_scale_bsm_p;
  static float converged_scale_bsm_v;
  static float converged_scale_v2i_ssm_p;
  static float converged_scale_v2i_ssm_v;
  static float converged_scale_v2n_rsm_p;
  static float converged_scale_v2n_rsm_v;
  static float converged_scale_v2n_rsi_p;
  static float converged_scale_v2n_rsi_v;
  static float unconverged_scale;
};

}  // namespace fusion
}  // namespace perception