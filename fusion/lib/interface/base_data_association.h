#pragma once

#include <string>
#include <utility>
#include <vector>

#include "base/base_forward_declaration.h"
#include "base/scene.h"
#include "base/sensor_frame.h"
#include "perception/lib/registerer/registerer.h"

namespace perception {
namespace fusion {

struct AssociationOptions {};

typedef std::pair<size_t, size_t> TrackMeasurmentPair;

struct AssociationResult {
  std::vector<TrackMeasurmentPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_measurements;
  std::vector<double> track2measurements_dist;
  std::vector<double> measurement2track_dist;
};

class BaseDataAssociation {
 public:
  BaseDataAssociation() {}
  virtual ~BaseDataAssociation() {}
  BaseDataAssociation(const BaseDataAssociation&) = delete;
  BaseDataAssociation& operator=(const BaseDataAssociation&) = delete;

  virtual bool Init() = 0;

  // @brief: associate sensor measurements with global scene
  // @param [in]: options
  // @param [in]: sensor_measurements
  // @param [in]: scene
  // @param [out]: association_result
  virtual bool Associate(const AssociationOptions& options,
                         SensorFramePtr sensor_measurements,
                         ScenePtr scene,
                         AssociationResult* association_result) = 0;

  virtual std::string Name() const = 0;
};

}  // namespace fusion
}  // namespace perception
