#pragma once
#include <fstream>
#include <map>
#include <ros/ros.h>

#include "perception/base/perception_gflags.h"
#include "perception/radar/common/include/yaml.h"
#include "common/include/log.h"
#include "common/include/macros.h"
namespace perception {
namespace v2x {
class V2xConfigManager {
 public:
  //获取参数的通用接口
  bool get_value(const std::string& name, int* value) const {
    return get_value_from_map<int>(name, integer_param_map_, value);
  }

  bool get_value(const std::string& name, std::string* value) const {
    return get_value_from_map<std::string>(name, string_param_map_, value);
  }

  bool get_value(const std::string& name, double* value) const {
    return get_value_from_map<double>(name, double_param_map_, value);
  }

  bool get_value(const std::string& name, float* value) const {
    return get_value_from_map<float>(name, float_param_map_, value);
  }

  bool get_value(const std::string& name, bool* value) const {
    return get_value_from_map<bool>(name, bool_param_map_, value);
  }

  bool get_value(const std::string& name, std::vector<int>* values) const {
    return get_value_from_map<std::vector<int>>(name, array_integer_param_map_, values);
  }

  bool get_value(const std::string& name, std::vector<double>* values) const {
    return get_value_from_map<std::vector<double>>(name, array_double_param_map_, values);
  }

  bool get_value(const std::string& name, std::vector<float>* values) const {
    return get_value_from_map<std::vector<float>>(name, array_float_param_map_, values);
  }

  bool get_value(const std::string& name, std::vector<std::string>* values) const {
    return get_value_from_map<std::vector<std::string>>(name, array_string_param_map_, values);
  }

  bool get_value(const std::string& name, std::vector<bool>* values) const {
    return get_value_from_map<std::vector<bool>>(name, array_bool_param_map_, values);
  }

  bool Init(std::string config_path);

 private:
  template <typename T>
  bool get_value_from_map(const std::string& name, const std::map<std::string, T>& container,
                          T* value) const;

 private:
  std::map<std::string, int> integer_param_map_;         //存储int类型的参数
  std::map<std::string, std::string> string_param_map_;  //存储string类型的参数
  std::map<std::string, double> double_param_map_;       //存储double类型的参数
  std::map<std::string, float> float_param_map_;         //存储float类型的参数
  std::map<std::string, bool> bool_param_map_;           //存储bool类型的参数
  std::map<std::string, std::vector<int>> array_integer_param_map_;
  std::map<std::string, std::vector<std::string>> array_string_param_map_;
  std::map<std::string, std::vector<double>> array_double_param_map_;
  std::map<std::string, std::vector<float>> array_float_param_map_;
  std::map<std::string, std::vector<bool>> array_bool_param_map_;

  bool inited_ = false;
  std::mutex mutex_;
  YAML::Node v2x_config_;
  std::string yamlpath_;
  DECLARE_SINGLETON(V2xConfigManager)
};
template <typename T>
bool V2xConfigManager::get_value_from_map(const std::string& name,
                                       const std::map<std::string, T>& container, T* value) const {
  typename std::map<std::string, T>::const_iterator citer = container.find(name);

  if (citer == container.end()) {
    return false;
  }

  *value = citer->second;
  return true;
}

}  // namespace v2x
}  // namespace perception
