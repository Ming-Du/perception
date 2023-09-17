#pragma once

#include <memory>
#include <string>

#include "base/base_init_options.h"
#include "lib/data_fusion/tracker_optimizer/tracker_optimizer.h"
#include "lib/interface/base_existence_fusion.h"
#include "lib/interface/base_motion_fusion.h"
#include "lib/interface/base_shape_fusion.h"
#include "lib/interface/base_tracker.h"
#include "lib/interface/base_type_fusion.h"
#include "common/proto/pbf_tracker_config.pb.h"

namespace perception {
namespace fusion {

class PbfTracker : public BaseTracker {
 public:
  PbfTracker();
  virtual ~PbfTracker();

  PbfTracker(const PbfTracker&) = delete;
  PbfTracker& operator=(const PbfTracker&) = delete;

  static bool InitParams();

  bool Init(TrackPtr track, SensorObjectPtr measurement) override;

  void UpdateWithMeasurement(const TrackerOptions& options,
                             const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const TrackerOptions& options,
                                const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override;

 protected:
  bool InitMethods();

 protected:
  static std::string s_type_fusion_method_;
  static std::string s_motion_fusion_method_;
  static std::string s_shape_fusion_method_;
  static std::string s_existence_fusion_method_;

  std::unique_ptr<BaseTypeFusion> type_fusion_ = nullptr;
  std::unique_ptr<BaseMotionFusion> motion_fusion_ = nullptr;
  std::unique_ptr<BaseExistenceFusion> existence_fusion_ = nullptr;
  std::unique_ptr<BaseShapeFusion> shape_fusion_ = nullptr;
  std::unique_ptr<tracker_optimizer> tracker_optimizer_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
