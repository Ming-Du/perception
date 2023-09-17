#pragma once

#include <string>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

class BaseMotionFusion {
 public:
  explicit BaseMotionFusion(TrackPtr track) : track_ref_(track) {}
  virtual ~BaseMotionFusion() {}
  BaseMotionFusion(const BaseMotionFusion&) = delete;
  BaseMotionFusion& operator=(const BaseMotionFusion&) = delete;

  virtual bool Init() = 0;

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  virtual void UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                                     double target_timestamp) = 0;

  virtual void UpdateWithoutMeasurement(const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp) = 0;

  virtual std::string Name() const = 0;

 protected:
  TrackPtr track_ref_;
};

}  // namespace fusion
}  // namespace perception
