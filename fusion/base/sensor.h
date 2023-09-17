
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include <common/proto/sensor_meta.pb.h>
#include "base/base_forward_declaration.h"
#include "base/sensor_frame.h"
#include "common/proto/sensor_meta.pb.h"

namespace perception {
namespace fusion {

/**
 * MogoSensorType mapping    - syf
 */
const std::map<perception::base::SensorType, std::string> kMogoSensorType2NameMap = {
    {perception::base::SensorType::UNKNOWN_SENSOR_TYPE, "UNKNOWN_SENSOR_TYPE"},
    {perception::base::SensorType::GNSS, "GNSS"},
    {perception::base::SensorType::CAMERA_6MM, "CAMERA_6MM"},
    {perception::base::SensorType::CAMERA_12MM, "CAMERA_12MM"},
    {perception::base::SensorType::CT_RADAR, "CT_RADAR"},
    {perception::base::SensorType::CONTI_RADAR_ARS408, "CONTI_RADAR_ARS408"},
    {perception::base::SensorType::CONTI_RADAR_SRR308, "CONTI_RADAR_SRR308"},
    {perception::base::SensorType::LSLIDAR_C16, "LSLIDAR_C16"},
    {perception::base::SensorType::LSLIDAR_C32, "LSLIDAR_C32"},
    {perception::base::SensorType::LSLIDAR_CH, "LSLIDAR_CH"},
    {perception::base::SensorType::LIVOX_HORIZON, "LIVOX_HORIZON"},
    {perception::base::SensorType::HESAI_XT32, "HESAI_XT32"},
    {perception::base::SensorType::VELODYNE_64, "VELODYNE_64"},
    {perception::base::SensorType::SENSING_30, "SENSING_30"},
    {perception::base::SensorType::SENSING_60, "SENSING_60"},
    {perception::base::SensorType::SENSING_120, "SENSING_120"},
    {perception::base::SensorType::HESAI_128, "HESAI_128"},
    {perception::base::SensorType::INNOVUSION_FALCON_LIDAR, "INNOVUSION_FALCON_LIDAR"},//Modify @jiangnan
    {perception::base::SensorType::ZVISIONLIDAR, "ZVISIONLIDAR"},
    {perception::base::SensorType::RSLIDAR_80, "RSLIDAR_80"},
    {perception::base::SensorType::RSLIDAR_M1, "RSLIDAR_M1"},
    {perception::base::SensorType::RSLIDAR_HELIOS, "RSLIDAR_HELIOS"},

    // Modify(@liuxinyu): obu_test
    {perception::base::SensorType::V2X, "V2X"},
    {perception::base::SensorType::OBU_RSM, "OBU_RSM"},
    {perception::base::SensorType::VIDAR, "VIDAR"}};

const std::map<std::string, size_t> kMogoSensorName2TypeMap = {
    {"UNKNOWN_SENSOR_TYPE", perception::base::SensorType::UNKNOWN_SENSOR_TYPE},
    {"GNSS", perception::base::SensorType::GNSS},
    {"CAMERA_6MM", perception::base::SensorType::CAMERA_6MM},
    {"CAMERA_12MM", perception::base::SensorType::CAMERA_12MM},
    {"CT_RADAR", perception::base::SensorType::CT_RADAR},
    {"CONTI_RADAR_ARS408", perception::base::SensorType::CONTI_RADAR_ARS408},
    {"CONTI_RADAR_SRR308", perception::base::SensorType::CONTI_RADAR_SRR308},
    {"LSLIDAR_C16", perception::base::SensorType::LSLIDAR_C16},
    {"LSLIDAR_C32", perception::base::SensorType::LSLIDAR_C32},
    {"LSLIDAR_CH", perception::base::SensorType::LSLIDAR_CH},
    {"LIVOX_HORIZON", perception::base::SensorType::LIVOX_HORIZON},
    {"HESAI_XT32", perception::base::SensorType::HESAI_XT32},
    {"VELODYNE_64", perception::base::SensorType::VELODYNE_64},
    {"SENSING_30", perception::base::SensorType::SENSING_30},
    {"SENSING_60", perception::base::SensorType::SENSING_60},
    {"SENSING_120", perception::base::SensorType::SENSING_120},
    {"HESAI_128", perception::base::SensorType::HESAI_128},
    {"INNOVUSION_FALCON_LIDAR", perception::base::SensorType::INNOVUSION_FALCON_LIDAR},//Modify @jiangnan
    {"ZVISIONLIDAR", perception::base::SensorType::ZVISIONLIDAR},
    {"RSLIDAR_80", perception::base::SensorType::RSLIDAR_80},
    {"RSLIDAR_M1", perception::base::SensorType::RSLIDAR_M1},
    {"RSLIDAR_HELIOS", perception::base::SensorType::RSLIDAR_HELIOS},

    // Modify(@liuxinyu): obu_test
    {"V2X", perception::base::SensorType::V2X},
    {"OBU_RSM", perception::base::SensorType::OBU_RSM},
    {"VIDAR", perception::base::SensorType::VIDAR}};

class Sensor {
 public:
  Sensor() = delete;

  explicit Sensor(const base::SensorInfo& sensor_info) : sensor_info_(sensor_info) {}

  // query frames whose time stamp is in range
  // (_latest_fused_time_stamp, time_stamp]
  void QueryLatestFrames(double timestamp, std::vector<SensorFramePtr>* frames);

  // query latest frame whose time stamp is in range
  // (_latest_fused_time_stamp, time_stamp]
  SensorFramePtr QueryLatestFrame(double timestamp);

  bool GetPose(double timestamp, Eigen::Affine3d* pose) const;

  // Getter
  inline std::string GetSensorId() const { return sensor_info_.name(); }

  // Getter
  inline base::SensorType GetSensorType() const { return sensor_info_.type(); }

  void AddFrame(const fusion::FrameConstPtr& frame_ptr);

  inline static void SetMaxCachedFrameNumber(size_t number) { kMaxCachedFrameNum = number; }

  void SetLatestQueryTimestamp(const double latest_query_timestamp) {
    latest_query_timestamp_ = latest_query_timestamp;
  }

 private:
  FRIEND_TEST(SensorTest, test);

  base::SensorInfo sensor_info_;

  double latest_query_timestamp_ = 0.0;

  std::deque<SensorFramePtr> frames_;

  static size_t kMaxCachedFrameNum;
};

}  // namespace fusion
}  // namespace perception
