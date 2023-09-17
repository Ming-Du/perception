
#include "dummy_algorithms.h"

namespace perception {
namespace fusion {

// class DummyFusionSystem implementation
bool DummyFusionSystem::Init(const FusionInitOptions& options) {
  main_sensors_ = options.main_sensors;
  return true;
}

bool DummyFusionSystem::Fuse(const FusionOptions& options,
                             const fusion::FrameConstPtr& sensor_frame,
                             std::vector<fusion::ObjectPtr>* fused_objects) {
  if (fused_objects == nullptr) {
    return false;
  }

  fused_objects->clear();
  if (std::find(main_sensors_.begin(), main_sensors_.end(), sensor_frame->sensor_info.name()) ==
      main_sensors_.end()) {
    return true;
  }

  fused_objects->resize(sensor_frame->objects.size());
  for (size_t i = 0; i < sensor_frame->objects.size(); i++) {
    (*fused_objects)[i] = sensor_frame->objects[i];
  }
  return true;
}

// class DummyDataAssociation implementation
bool DummyDataAssociation::Init() {
  return true;
}

bool DummyDataAssociation::Associate(const AssociationOptions& options,
                                     SensorFramePtr sensor_measurements,
                                     ScenePtr scene,
                                     AssociationResult* association_result) {
  return true;
}

// class DummyTracker implementation
bool DummyTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  return true;
}

void DummyTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                         const SensorObjectPtr measurement,
                                         double target_timestamp) {}

void DummyTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                            const std::string& sensor_id,
                                            double measurement_timestamp,
                                            double target_timestamp) {}

FUSION_REGISTER_FUSIONSYSTEM(DummyFusionSystem);

}  // namespace fusion
}  // namespace perception
