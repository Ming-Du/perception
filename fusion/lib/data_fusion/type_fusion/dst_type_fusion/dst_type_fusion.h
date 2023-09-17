#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/dst_evidence.h"
#include "lib/interface/base_type_fusion.h"

namespace perception {
namespace fusion {

struct DstMaps {
  // dst hypothesis types
  enum {
    PEDESTRIAN = (1 << 0),
    BICYCLE = (1 << 1),
    MOTOR = (1 << 2),
    CAR = (1 << 3),
    BUS = (1 << 4),
    TRUCK = (1 << 5),
    VEGETATION = (1 << 6),
    TRAFFICCONE = (1 << 7),
    UNKNOWN = (1 << 8),
  };
  enum {
    OTHERS = (VEGETATION | PEDESTRIAN | BICYCLE | CAR | BUS | TRUCK | TRAFFICCONE | MOTOR | UNKNOWN)
  };

  std::vector<uint64_t> fod_subsets_ = {
    PEDESTRIAN, BICYCLE, MOTOR, CAR, BUS, TRUCK, VEGETATION, TRAFFICCONE,UNKNOWN,OTHERS};
  std::vector<std::string> subset_names_ = {
      "PEDESTRIAN","BICYCLE","MOTOR","CAR","BUS","TRUCK","VEGETATION", "TRAFFICCONE","UNKNOWN","OTHERS"};
  std::unordered_map<size_t, uint64_t> typ_to_hyp_map_ = {
      {static_cast<size_t>(fusion::ObjectType::PEDESTRIAN), PEDESTRIAN},
      {static_cast<size_t>(fusion::ObjectType::BICYCLE), BICYCLE},
      {static_cast<size_t>(fusion::ObjectType::MOTOR), MOTOR},
      {static_cast<size_t>(fusion::ObjectType::CAR), CAR},
      {static_cast<size_t>(fusion::ObjectType::BUS), BUS},
      {static_cast<size_t>(fusion::ObjectType::TRUCK), TRUCK},
      {static_cast<size_t>(fusion::ObjectType::VEGETATION), VEGETATION},
      {static_cast<size_t>(fusion::ObjectType::TRAFFICCONE), TRAFFICCONE},
      {static_cast<size_t>(fusion::ObjectType::UNKNOWN), UNKNOWN},
  };
  std::map<uint64_t, size_t> hyp_to_typ_map_ = {
      {PEDESTRIAN, static_cast<size_t>(fusion::ObjectType::PEDESTRIAN)},
      {BICYCLE, static_cast<size_t>(fusion::ObjectType::BICYCLE)},
      {MOTOR, static_cast<size_t>(fusion::ObjectType::MOTOR)},
      {CAR, static_cast<size_t>(fusion::ObjectType::CAR)},
      {BUS, static_cast<size_t>(fusion::ObjectType::BUS)},
      {TRUCK, static_cast<size_t>(fusion::ObjectType::TRUCK)},
      {VEGETATION, static_cast<size_t>(fusion::ObjectType::VEGETATION)},
      {TRAFFICCONE, static_cast<size_t>(fusion::ObjectType::TRAFFICCONE)},
      {UNKNOWN, static_cast<size_t>(fusion::ObjectType::UNKNOWN)},
      {OTHERS, static_cast<size_t>(fusion::ObjectType::UNKNOWN)},
  };
};

struct DstTypeFusionOptions {
  std::map<std::string, double> camera_max_valid_dist_ = {
      {"SENSING_30", 110},
      {"SENSING_60", 110},
      {"SENSING_120", 110},
  };
  std::map<std::string, double> sensor_reliability_ = {
      {"LSLIDAR_C16", 0.5},
      {"LSLIDAR_C32", 0.5},
      {"LSLIDAR_CH", 0.5},
      {"LIVOX_HORIZON", 0.5},
      {"HESAI_XT32", 0.5},
      {"VELODYNE_64", 0.5},
      {"SENSING_30", 0.8},
      {"SENSING_60", 0.8},
      {"SENSING_120", 0.8},
      {"HESAI_128", 0.5},
      {"INNOVUSION_FALCON_LIDAR", 0.5},
      {"ZVISIONLIDAR", 0.5},
      {"RSLIDAR_80", 0.5},
      {"RSLIDAR_M1", 0.5},
      {"RSLIDAR_HELIOS", 0.5},
      {"V2X", 0.5},
      {"OBU_RSM", 0.5},
      {"VIDAR", 0.8},
  };
  std::map<std::string, double> sensor_reliability_for_unknown_ = {
      {"LSLIDAR_C16", 0.0},
      {"LSLIDAR_C32", 0.0},
      {"LSLIDAR_CH", 0.0},
      {"LIVOX_HORIZON", 0.0},
      {"HESAI_XT32", 0.0},
      {"VELODYNE_64", 0.0},
      {"SENSING_30", 0.0},
      {"SENSING_60", 0.0},
      {"SENSING_120", 0.0},
      {"HESAI_128", 0.0},
      {"INNOVUSION_FALCON_LIDAR", 0.0},
      {"ZVISIONLIDAR", 0.0},
      {"RSLIDAR_80", 0.0},
      {"RSLIDAR_M1", 0.0},
      {"RSLIDAR_HELIOS", 0.0},
      {"V2X", 0.0},
      {"OBU_RSM", 0.0},
      {"VIDAR", 0.0},
  };
};

class DstTypeFusion : public BaseTypeFusion {
 public:
  explicit DstTypeFusion(TrackPtr track);
  ~DstTypeFusion() {}

  // @brief: init dst application and options_
  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  void UpdateWithMeasurement(const SensorObjectPtr measurement, double target_timestamp) override;

  void UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp,
                                double min_match_dist) override;

  std::string Name() const;

 private:
  bool TypToHyp(size_t object_type, uint64_t *hypothesis_type) const;
  bool HypToTyp(uint64_t hypothesis_type, size_t *object_type) const;
  Dst TypeProbsToDst(const std::vector<float> &type_probs);
  double GetReliability(const std::string &sensor_id) const;
  double GetReliabilityForUnKnown(const std::string &sensor_id,
                                  double measurement_timestamp) const;

  // Update state
  void UpdateTypeState();

 private:
  Dst fused_dst_;
  double sensor_reliability_;//设置默认sensor_reliability，防止map查找不到时候计算错误

 private:
  static std::string name_;
  static DstMaps dst_maps_;
  static DstTypeFusionOptions options_;
};

}  // namespace fusion
}  // namespace perception
