#pragma once

#include <string>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

class BaseGatekeeper {
 public:
  BaseGatekeeper() {}
  virtual ~BaseGatekeeper() {}
  BaseGatekeeper(const BaseGatekeeper&) = delete;
  BaseGatekeeper& operator=(const BaseGatekeeper&) = delete;

  virtual bool Init() = 0;

  virtual bool AbleToPublish(const TrackPtr& track) = 0;

  virtual std::string Name() const = 0;
};

}  // namespace fusion
}  // namespace perception
