#pragma once

#include <string>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

class BaseExistenceFusion {
 public:
  explicit BaseExistenceFusion(TrackPtr track) : track_ref_(track) {}
  virtual ~BaseExistenceFusion() {}
  BaseExistenceFusion(const BaseExistenceFusion&) = delete;
  BaseExistenceFusion& operator=(const BaseExistenceFusion&) = delete;

  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  // @param [in/out]: track
  virtual void UpdateWithMeasurement(const SensorObjectPtr measurement,
                                     double target_timestamp,
                                     double match_dist) = 0;

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
