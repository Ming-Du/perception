#pragma once
#include "yaml.h"
#include <unistd.h>
#include <fstream>
namespace perception {
namespace radar {
bool LoadFile(const std::string& path, YAML::Node& node) {
  YAML::Node result;
  std::fstream t_f;
  t_f.open(path);
  if (!t_f) {
    t_f.close();
    return false;
  } else {
    t_f.close();
  }
  if (access(path.c_str(), F_OK) != 0) {
    return false;
  }
  try {
    node = YAML::LoadFile(path);
  } catch (const char* msg) {
    return false;
  }
  return true;
}
}  // namespace radar
}  // namespace perception