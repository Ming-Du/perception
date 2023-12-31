#pragma once

#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>

#include "common/dst_evidence.h"
#include "lib/interface/base_existence_fusion.h"

namespace perception {
namespace fusion {

struct ToicDstMaps {
  // for (N)TOIC: (not)target of interest in camera judgement
  enum { TOIC = (1 << 0), NTOIC = (1 << 1), TOICUNKNOWN = (TOIC | NTOIC) };
  std::vector<uint64_t> fod_subsets_ = {TOIC, NTOIC, TOICUNKNOWN};
  std::vector<std::string> subset_names_ = {"TOIC", "NTOIC", "TOICUNKNOWN"};
};

struct ExistenceDstMaps {
  enum { EXIST = (1 << 0), NEXIST = (1 << 1), EXISTUNKNOWN = (EXIST | NEXIST) };
  std::vector<uint64_t> fod_subsets_ = {EXIST, NEXIST, EXISTUNKNOWN};
  std::vector<std::string> subset_names_ = {"EXIST", "NEXIST", "EXISTUNKNOWN"};
};

struct DstExistenceFusionOptions {
  std::map<std::string, double> camera_max_valid_dist_ = {
      {"camera_smartereye", 110},
      {"camera_front_obstacle", 110},
      {"camera_front_narrow", 150},
      {"front_6mm", 110},
  };
  double track_object_max_match_distance_ = 4.0;
};

class DstExistenceFusion : public BaseExistenceFusion {
 public:
  explicit DstExistenceFusion(TrackPtr track);
  ~DstExistenceFusion() {}

  // @brief: add dst application
  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  void UpdateWithMeasurement(const SensorObjectPtr measurement,
                             double target_timestamp,
                             double match_dist) override;

  void UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp,
                                double min_match_dist) override;

  std::string Name() const;
  double GetToicScore() const { return toic_score_; }
  double GetExistenceProbability() const;

 private:
  void UpdateToicWithCameraMeasurement(const SensorObjectPtr& camera_obj, double match_dist);
  void UpdateToicWithoutCameraMeasurement(const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double match_dist);

  double ComputeDistDecay(fusion::ObjectConstPtr obj,
                          const std::string& sensor_id,
                          double timestamp);
  double ComputeFeatureInfluence(const SensorObjectPtr measurement);
  double GetExistReliability(const SensorObjectPtr measurement);
  double GetUnexistReliability(const std::string& sensor_id);
  double GetToicProbability() const;

  // Update state
  void UpdateExistenceState();

 private:
  double existence_score_ = 0.0;
  Dst fused_toic_;
  Dst fused_existence_;
  double toic_score_ = 0.0;

 private:
  static const char* name_;
  static const char* toic_name_;
  static ExistenceDstMaps existence_dst_maps_;
  static ToicDstMaps toic_dst_maps_;
  static DstExistenceFusionOptions options_;
  // to get sensor type from sensor name - syf add
  perception::base::SensorInfo camera_sensor_info_;
};

}  // namespace fusion
}  // namespace perception
