#pragma once

#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <condition_variable>

#include "common/include/macros.h"
#include "perception/lib/thread/thread_worker.h"

#include "lidar_data_manager.h"

namespace perception {
namespace xiaoba_fusion {

class LidarDataMonitor {
public:
  void Notify(const std::string &sensor_id);

private:
  ~LidarDataMonitor();
  void Process();

private:
  double m_stamp_bias_;
  bool m_exit_flag_;
  bool m_ready_process_;
  std::string m_sensor_id_;
  std::mutex m_mutex_;
  std::condition_variable m_cv_;

  LidarDataManager *m_pclsLidarDataManager;

  LidarDataMap lidar_data_map_;
  PointCloudData latest_data_;
  PointCloudData last_data_;

  ros::Time output_timestamp;

  std::shared_ptr<std::thread> t_data_process_;

  DECLARE_SINGLETON(LidarDataMonitor)
};

}  // namespace xiaoba_fusion
}  // namespace perception