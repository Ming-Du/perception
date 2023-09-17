#pragma once

#include <yaml-cpp/yaml.h>
namespace perception {
namespace radar {
bool LoadFile(const std::string& path, YAML::Node& node);

template <typename T>
inline bool YamlRead(const YAML::Node& yaml,
                     const std::string& key,
                     T& out_val,
                     const std::string name = "",
                     const bool silence = true) {
  try {
    if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null) {
      if (!silence) {
      }
      return false;
    } else {
      out_val = yaml[key].as<T>();
    }
  } catch (const char* msg) {
    if (!silence) {
    }
    return false;
  }

  return true;
}

inline bool YamlSubNode(const YAML::Node& yaml,
                        const std::string& node,
                        YAML::Node& ret,
                        const std::string name = "",
                        const bool silence = true) {
  try {
    YAML::Node tmp_ret = yaml[node.c_str()];
    if (!tmp_ret) {
      if (!silence) {
      }
      return false;
    }
    ret = tmp_ret;
    return true;
  } catch (const char* msg) {
    if (!silence) {
    }
  }
  return true;
}
}  // namespace radar
}  // namespace perception