#pragma once

#include <string>
#include <vector>

#include "base/base_forward_declaration.h"
#include "base/frame.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

struct FusionInitOptions {
  std::vector<std::string> main_sensors;
};

struct FusionOptions {};

class BaseFusionSystem {
 public:
  BaseFusionSystem() = default;
  virtual ~BaseFusionSystem() = default;
  BaseFusionSystem(const BaseFusionSystem&) = delete;
  BaseFusionSystem& operator=(const BaseFusionSystem&) = delete;

  virtual bool Init(const FusionInitOptions& options) = 0;

  // @brief: fuse a sensor frame
  // @param [in]: options
  // @param [in]: sensor_frame
  // @param [out]: fused objects
  virtual bool Fuse(const FusionOptions& options,
                    const fusion::FrameConstPtr& sensor_frame,
                    std::vector<fusion::ObjectPtr>* fused_objects) = 0;

  virtual std::string Name() const = 0;

 protected:
  std::vector<std::string> main_sensors_;
};

PERCEPTION_REGISTER_REGISTERER(BaseFusionSystem);
#define FUSION_REGISTER_FUSIONSYSTEM(name) PERCEPTION_REGISTER_CLASS(BaseFusionSystem, name)

}  // namespace fusion
}  // namespace perception
