#pragma once

#include <string>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {
struct TrackerOptions {
  double match_distance = 0.0;
};

class BaseTracker {
 public:
  BaseTracker() {}
  virtual ~BaseTracker() {}
  BaseTracker(const BaseTracker&) = delete;
  BaseTracker& operator=(const BaseTracker&) = delete;

  virtual bool Init(TrackPtr track, SensorObjectPtr measurement) = 0;

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  // @param [in/out]: track
  virtual void UpdateWithMeasurement(const TrackerOptions& options,
                                     const SensorObjectPtr measurement,
                                     double target_timestamp) = 0;

  virtual void UpdateWithoutMeasurement(const TrackerOptions& options,
                                        const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp) = 0;

  virtual std::string Name() const = 0;

 protected:
  TrackPtr track_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
