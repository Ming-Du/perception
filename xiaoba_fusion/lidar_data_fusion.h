#pragma once

#include "lidar_data_manager.h"
#include "lidar_fault_reporter.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <float.h>

namespace perception{
namespace xiaoba_fusion{

class LidarDataFusion {
public:
  LidarDataFusion();
  ~LidarDataFusion();

  void Init();
  void FusionAndPublishCloud();

  void PointCouldCallBack(const sensor_msgs::PointCloud2ConstPtr &msg);
  void OvertimeEvent(const ros::TimerEvent &);

  void Run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Timer over_timer;

  double SYS_TIMESTAMP_OVERTIME_THRESHOLD = 2;

  // param
  bool monitor_stamp_flag_;
  std::string fusion_frame_id_;
  double over_time_threshold_; // s
  double lidar_time_offset_;
  std::vector<std::string> lidar_names_;

  double init_time_;
  double first_lidar_stamp;
  double current_topic_start_time_;
  ros::Time last_topic_publish_time_;
  std::mutex storage_mutex_;

  uint32_t pub_sequence_;
  int pub_abnormal_count_;

  PointCloudData point_cloud_data_;
  sensor_msgs::PointCloud2Ptr temp_cloud_ptr;
  std::deque<PointCloudData> frames_deque_;
  std::unordered_map<std::string, bool> isget_map_;

  ros::Publisher fusion_pub;
  std::vector<ros::Subscriber> m_vecSubsrciber;

  LidarDataManager *m_pclsLidarDataManager;
  LidarFaultReporterPtr m_pclsLidarFaultReporter;
};

}   // namespace xiaoba_fusion
}   // namespace perception


