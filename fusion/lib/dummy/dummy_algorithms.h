#pragma once

#include <string>
#include <vector>

#include "lib/interface/base_data_association.h"
#include "lib/interface/base_existence_fusion.h"
#include "lib/interface/base_fusion_system.h"
#include "lib/interface/base_motion_fusion.h"
#include "lib/interface/base_shape_fusion.h"
#include "lib/interface/base_tracker.h"
#include "lib/interface/base_type_fusion.h"

namespace perception {
namespace fusion {

class DummyFusionSystem : public BaseFusionSystem {
 public:
  DummyFusionSystem() = default;
  ~DummyFusionSystem() = default;
  DummyFusionSystem(const DummyFusionSystem&) = delete;
  DummyFusionSystem& operator=(const DummyFusionSystem&) = delete;

  bool Init(const FusionInitOptions& options) override;
  std::string Name() const override { return "DummyFusionSystem"; }

  bool Fuse(const FusionOptions& options,
            const fusion::FrameConstPtr& sensor_frame,
            std::vector<fusion::ObjectPtr>* fused_objects) override;
};

class DummyDataAssociation : public BaseDataAssociation {
 public:
  DummyDataAssociation() = default;
  ~DummyDataAssociation() = default;
  DummyDataAssociation(const DummyDataAssociation&) = delete;
  DummyDataAssociation& operator=(const DummyDataAssociation&) = delete;

  bool Init() override;
  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements,
                 ScenePtr scene,
                 AssociationResult* association_result) override;

  std::string Name() const override { return "DummyDataAssociation"; }
};

class DummyTracker : public BaseTracker {
 public:
  DummyTracker() = default;
  ~DummyTracker() = default;
  DummyTracker(const DummyTracker&) = delete;
  DummyTracker& operator=(const DummyTracker&) = delete;

  bool Init(TrackPtr track, SensorObjectPtr measurement) override;

  void UpdateWithMeasurement(const TrackerOptions& options,
                             const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const TrackerOptions& options,
                                const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override { return "DummyTracker"; }
};

}  // namespace fusion
}  // namespace perception
