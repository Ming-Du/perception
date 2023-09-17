#pragma once

#include <string>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

class BaseTypeFusion {
 public:
  explicit BaseTypeFusion(TrackPtr track) : track_ref_(track) {}
  virtual ~BaseTypeFusion() {}
  BaseTypeFusion(const BaseTypeFusion&) = delete;
  BaseTypeFusion& operator=(const BaseTypeFusion&) = delete;

  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  virtual void UpdateWithMeasurement(const SensorObjectPtr measurement,
                                     double target_timestamp) = 0;

  virtual void UpdateWithoutMeasurement(const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp,
                                        double min_match_dist) = 0;

  virtual std::string Name() const = 0;

 protected:
  TrackPtr track_ref_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
