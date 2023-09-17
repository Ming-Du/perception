#pragma once

#include <string>

namespace perception {
namespace fusion {

struct BaseInitOptions {
  std::string root_dir;
  std::string conf_file;
};

bool GetFusionInitOptions(const std::string& module_name, BaseInitOptions* options);

}  // namespace fusion
}  // namespace perception
