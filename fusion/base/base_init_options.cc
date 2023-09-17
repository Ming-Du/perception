
#include "base_init_options.h"

#include "common/include/log.h"

#include "perception/lib/config_manager/config_manager.h"

namespace perception {
namespace fusion {

bool GetFusionInitOptions(const std::string& module_name, BaseInitOptions* options) {
  CHECK_NOTNULL(options);
  const lib::ModelConfig* model_config = nullptr;
  if (!lib::ConfigManager::Instance()->GetModelConfig(module_name, &model_config)) {
    return false;
  }
  bool state = model_config->get_value("root_dir", &(options->root_dir)) &&
               model_config->get_value("config_file", &(options->conf_file));
  return state;
}

}  // namespace fusion
}  // namespace perception
