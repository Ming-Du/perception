#include "v2x_config_manager.h"

namespace perception {
namespace v2x {
V2xConfigManager::V2xConfigManager() {}

bool V2xConfigManager::Init(std::string config_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }
  //加载配置文件
  try {
    yamlpath_ = config_path + "/v2x_config.yaml";
    v2x_config_ = YAML::LoadFile(yamlpath_);
    //解析参数
    double_param_map_["lon"] = v2x_config_["self_vehicle_filter"]["lon"].as<double>();
    double_param_map_["lat"] = v2x_config_["self_vehicle_filter"]["lat"].as<double>();
    double_param_map_["heading"] = v2x_config_["self_vehicle_filter"]["heading"].as<double>();
    double_param_map_["speed"] = v2x_config_["self_vehicle_filter"]["speed"].as<double>();
    double_param_map_["width"] = v2x_config_["self_vehicle_filter"]["width"].as<double>();
    double_param_map_["length"] = v2x_config_["self_vehicle_filter"]["length"].as<double>();
    double_param_map_["eva"] = v2x_config_["self_vehicle_filter"]["eva"].as<double>();
    double_param_map_["host_length"] =
        v2x_config_["self_vehicle_filter"]["host_length"].as<double>();
    double_param_map_["host_width"] = v2x_config_["self_vehicle_filter"]["host_width"].as<double>();
    double_param_map_["host_high"] = v2x_config_["self_vehicle_filter"]["host_high"].as<double>();
    double_param_map_["iou_thr"] = v2x_config_["self_vehicle_filter"]["iou_thr"].as<double>();
    bool_param_map_["use_filter_ego_car"] = v2x_config_["v2x_fusion"]["use_filter_ego_car"].as<bool>();;
    double_param_map_["TimeCheckOfStoreV2ISSM"] = v2x_config_["v2x_fusion"]["TimeCheckOfStoreV2ISSM"].as<double>();
    double_param_map_["TimeCheckOfStoreV2IRSM"] = v2x_config_["v2x_fusion"]["TimeCheckOfStoreV2IRSM"].as<double>();
    double_param_map_["TimeCheckOfStoreV2NRSM"] = v2x_config_["v2x_fusion"]["TimeCheckOfStoreV2NRSM"].as<double>();
    double_param_map_["TimeCheckOfStoreV2NRSI"] = v2x_config_["v2x_fusion"]["TimeCheckOfStoreV2NRSI"].as<double>();
    double_param_map_["TimeCheckOfStoreV2VBSM"] = v2x_config_["v2x_fusion"]["TimeCheckOfStoreV2VBSM"].as<double>();
    double_param_map_["TimeCheckOfLocalization"] = v2x_config_["v2x_fusion"]["TimeCheckOfLocalization"].as<double>();
    bool_param_map_["pub_v2x_rviz"] = v2x_config_["v2x_display"]["pub_v2x_rviz"].as<bool>();;
  } catch (...) {
    inited_ = false;
    ROS_ERROR_STREAM("Init: yaml file is wrong,cannot open it!" << config_path);
    return false;
  }

  inited_ = true;
  return true;
}

}  // namespace v2x

}  // namespace perception
