#include "configmanager.h"

ConfigManager::ConfigManager() {}

bool ConfigManager::Init(std::string configpath) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }
  //加载配置文件
  try {
    yamlpath_ = configpath + "/custom/fusionconfig.yaml";
    fusion_config_ = YAML::LoadFile(yamlpath_);
    //解析参数
    double_param_map_["distancescale_lidar2image"] =
        fusion_config_["fusion"]["vc_diff2distance_scale_factor_"].as<double>();

    string_param_map_["mid_fusion_obstacle_topic"] =
        fusion_config_["fusion"]["mid_fusion_obstacle_topic"].as<std::string>();
    integer_param_map_["prediction_count"] = fusion_config_["fusion"]["prediction_count"].as<int>();
    double_param_map_["radar_filter_yaw_thr"] =
        fusion_config_["fusion"]["radar_filter_yaw_thr"].as<double>();

    integer_param_map_["v2x_process_mode"] = fusion_config_["v2x"]["v2x_process_mode"].as<int>();
    integer_param_map_["pass_through_mode"] = fusion_config_["v2x"]["pass_through_mode"].as<int>();
    integer_param_map_["vehicle"] = fusion_config_["fusion"]["vehicle"].as<int>();
    float_param_map_["beyond_vis_range"] =
        fusion_config_["v2x"]["beyond_vis_range"].as<float>();
    float_param_map_["radius_for_fusion_object"] =
        fusion_config_["v2x"]["radius_for_fusion_object"].as<float>();
//    vehicle_ = (perception::fusion::VEHICLE)fusion_config_["fusion"]["vehicle"].as<int>();

    float_param_map_["yaw_kalman_q"] = fusion_config_["fusion"]["yaw_kalman_q"].as<float>();
    float_param_map_["yaw_kalman_r"] = fusion_config_["fusion"]["yaw_kalman_r"].as<float>();
    
    bool_param_map_["release_mode"] = fusion_config_["fusion"]["release_mode"].as<bool>();
    
    float_param_map_["sizel_kalman_q"] = fusion_config_["fusion"]["sizel_kalman_q"].as<float>();
    float_param_map_["sizel_kalman_r"] = fusion_config_["fusion"]["sizel_kalman_r"].as<float>();
    float_param_map_["sizew_kalman_q"] = fusion_config_["fusion"]["sizew_kalman_q"].as<float>();
    float_param_map_["sizew_kalman_r"] = fusion_config_["fusion"]["sizew_kalman_r"].as<float>();
    float_param_map_["sizeh_kalman_q"] = fusion_config_["fusion"]["sizeh_kalman_q"].as<float>();
    float_param_map_["sizeh_kalman_r"] = fusion_config_["fusion"]["sizeh_kalman_r"].as<float>();
    array_float_param_map_["frustum_front60"].resize(6);
    array_float_param_map_["frustum_front60"][0] = fusion_config_["fusion"]["frustum_front60_origin_x"].as<float>();
    array_float_param_map_["frustum_front60"][1] = fusion_config_["fusion"]["frustum_front60_origin_y"].as<float>();
    array_float_param_map_["frustum_front60"][2] = fusion_config_["fusion"]["frustum_front60_left_x"].as<float>();
    array_float_param_map_["frustum_front60"][3] = fusion_config_["fusion"]["frustum_front60_left_y"].as<float>();
    array_float_param_map_["frustum_front60"][4] = fusion_config_["fusion"]["frustum_front60_right_x"].as<float>();
    array_float_param_map_["frustum_front60"][5] = fusion_config_["fusion"]["frustum_front60_right_y"].as<float>();
    float_param_map_["zombie_thr"] =
        fusion_config_["v2x"]["zombie_thr"].as<float>();
    bool_param_map_["enable_only_pub_zombie"] =
        fusion_config_["v2x"]["enable_only_pub_zombie"].as<bool>();
  } catch (...) {
    inited_ = false;
    ROS_WARN_STREAM("Init: yaml file is wrong,cannot open it!");
    return false;
  }

  inited_ = true;
  return true;
}
