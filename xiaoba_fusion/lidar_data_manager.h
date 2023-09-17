#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include "perception/base/object.h"
#include "perception/base/perception_gflags.h"
#include "perception/base/sensor_manager/sensor_manager.h"

#include "common/include/macros.h"


using namespace perception;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudTPtr;

namespace perception {
namespace xiaoba_fusion {

typedef std::pair<ros::Time, sensor_msgs::PointCloud2Ptr> PointCloudData;

class LidarData {
public:
  LidarData();

  void SetTransform(const Eigen::Affine3d &matrix);
  Eigen::Affine3d GetTransform();

  void AddData(const PointCloudData &data);
  bool ModifiedLatestData(const PointCloudData &data);

  PointCloudData GetData(int index);
  PointCloudData GetLatestData();

  void SetStatus(bool status);
  bool GetStatus();

  size_t GetBufferSize();
  bool IsBufferFull();

private:
  std::mutex m_mutex_;
  bool data_status_;
  std::string topic_name_;
  Eigen::Affine3d lidar2base;
  boost::circular_buffer<PointCloudData> circular_buffer_time;
};

typedef std::shared_ptr<LidarData> LidarDataPtr;
typedef std::shared_ptr<const LidarData> LidarDataConstPtr;

using LidarDataMap = std::unordered_map<std::string, LidarDataPtr>;

class LidarDataManager {
public:
  bool Init();
  void Reset();
  void SetMonitorStampFlag(bool flag);
  std::string GetTopicName(const std::string &sensor_name);
  void AddSensorMeasurements(const PointCloudData &cloud_data);

  bool GetLatestSensorFrames(const std::string &sensor_id, const sensor_msgs::PointCloud2Ptr &frame_ptr);

  LidarDataMap GetLidarDataMap();

private:
  bool pclDataTransform(const sensor_msgs::PointCloud2Ptr &src_msg,
                        const sensor_msgs::PointCloud2Ptr &dst_msg, const Eigen::Affine3d &transform);

private:
  bool inited_;
  bool monitor_stamp_flag_;
  base::SensorManager *sensor_manager;
  LidarDataMap lidar_data_map_;

  std::mutex m_mutex_;

  DECLARE_SINGLETON(LidarDataManager)
};

}  // namespace xiaoba_fusion
}  // namespace perception

