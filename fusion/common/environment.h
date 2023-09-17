

#ifndef CYBER_COMMON_ENVIRONMENT_H_
#define CYBER_COMMON_ENVIRONMENT_H_

#include <ros/ros.h>
#include <cassert>
#include <string>

#include "common/include/log.h"

namespace perception {
namespace fusion {

inline std::string GetEnv(const std::string& var_name, const std::string& default_value = "") {
  const char* var = std::getenv(var_name.c_str());
  if (var == nullptr) {
    ROS_WARN_STREAM("GetEnv: Environment variable [" << var_name << "] not set, fallback to "
                                                     << default_value);
    return default_value;
  }
  return std::string(var);
}

inline const std::string WorkRoot() {
  std::string work_root = GetEnv("CYBER_PATH");
  if (work_root.empty()) {
    work_root = "/apollo/cyber";
  }
  return work_root;
}

}  // namespace fusion
}  // namespace perception

#endif  // CYBER_COMMON_ENVIRONMENT_H_
