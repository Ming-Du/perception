#include "lidar_data_manager.h"
#include "lidar_data_monitor.h"

namespace perception {
namespace xiaoba_fusion {

LidarDataMonitor::LidarDataMonitor()
    : m_stamp_bias_(0.1)
    , m_exit_flag_(false)
    , m_ready_process_(false)
    , m_pclsLidarDataManager(nullptr)
{
  output_timestamp = ros::Time::now();
  m_pclsLidarDataManager = LidarDataManager::Instance();
  t_data_process_ = std::make_shared<std::thread>(std::bind(&LidarDataMonitor::Process, this));
}

LidarDataMonitor::~LidarDataMonitor() {
  if (t_data_process_ == nullptr)
    return;
  {
    std::lock_guard<std::mutex> lock(m_mutex_);
    m_exit_flag_ = true;
    m_ready_process_ = true;
  }
  m_cv_.notify_one();
  t_data_process_->join();
}

void LidarDataMonitor::Notify(const std::string& sensor_id) {
  m_sensor_id_ = sensor_id;
  LidarDataMap lidar_data_map_ = m_pclsLidarDataManager->GetLidarDataMap();
  LidarDataMap::iterator it = lidar_data_map_.find(m_sensor_id_);
  if (it == lidar_data_map_.end()) {
    ROS_WARN("Find lidar map is abnormal, sensor_id = %s.", m_sensor_id_.c_str());
    return;
  }
  if (!it->second->IsBufferFull())
    return;
  if (!it->second->GetStatus()) {
    m_ready_process_ = true;
    m_cv_.notify_one();
  }
  else
    m_ready_process_ = false;  
}

void LidarDataMonitor::Process() {
  while (ros::ok()) {
    try {
      {
        std::unique_lock<std::mutex> lock(m_mutex_);
        m_cv_.wait(lock, [&](){ return m_ready_process_; });
        if (m_exit_flag_)
          break;
      }

      std::lock_guard<std::mutex> lock(m_mutex_);
      LidarDataMap lidar_data_map_ = m_pclsLidarDataManager->GetLidarDataMap();
      LidarDataMap::iterator it = lidar_data_map_.find(m_sensor_id_);
      if (it == lidar_data_map_.end()) {
        ROS_WARN("Find lidar map is abnormal, sensor_id = %s.", m_sensor_id_.c_str());
        continue;
      }

      bool exception_occure = false;
      size_t buffer_size_ = it->second->GetBufferSize();
      last_data_ = it->second->GetData(buffer_size_ - 2);
      latest_data_ = it->second->GetData(buffer_size_ - 1);
      if ((last_data_.second == nullptr) || (latest_data_.second == nullptr)) {
        ROS_WARN("lidar data is null, sensor_id = %s.", m_sensor_id_.c_str());
        continue;
      }

      output_timestamp = latest_data_.second->header.stamp;

      double system_tt_delta = latest_data_.first.toSec() - last_data_.first.toSec();
      double cloud_tt_delta = latest_data_.second->header.stamp.toSec() - last_data_.second->header.stamp.toSec();

      // exception 0: [ 1.8, 1.9, 100.0, 100.1 ]
      // case: 1692087117 ---> 2951626317
      if (cloud_tt_delta > 31536000) {
        ROS_WARN("Exception 0 occure. lidar time is much greater than system time.");
        output_timestamp = ros::Time(last_data_.second->header.stamp.toSec() + 0.1);
        exception_occure = true;
      }

      // exception 1: [ 1.8, 1.9, 1.0, 2.1 ]
      // exception 4: [ 1.0, 1.1, ..., 1.8, 1.9, 1.0, 1.1, ... , 1.8, 1.9, 3.0]
      if ((cloud_tt_delta > -0.95) && (cloud_tt_delta < -0.85)) {
        ROS_WARN("Exception 1 or 4 occure.");
        output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() + 1.0);
        exception_occure = true;
      }

      // exception 2: [ 1.8, 2.9, 2.0, 2.1 ]
      // exception 6: [ 1.8, *, *, ... , 2.8, 2.9, 3.0 ]
      if ((cloud_tt_delta > 1.05) && (cloud_tt_delta < 1.15)) {
        if (system_tt_delta < 0.15) {
          ROS_WARN("Exception 2 occure.");
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() - 1.0);
          exception_occure = true;
        }else {
          ROS_WARN("Exception 6.1 occure.");
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec());
        }
      }

      // exception 3: [1.8, 2.0, 1.9, 2.1]
      // exception 6: [ 1.8, 1.9, *, 2.1, 2,2 ]
      if ((cloud_tt_delta > 0.15) && (cloud_tt_delta < 0.25)) {
        if (system_tt_delta < 0.15) {
          ROS_WARN("Exception 3 occure.");
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() - 0.1);
          exception_occure = true;
        }
        else {
          ROS_WARN("Exception 6.2 occure.");
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec());
        }
      }

      // exception 5: [ 1.8, 1.9001, 1.9002, 2.1 ]
      if (abs(cloud_tt_delta) < 0.001) {
        ROS_WARN("Exception 5 occure.");
        output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() + 0.1);
        exception_occure = true;
      }

      // exception 7: 24h
      if ((abs(cloud_tt_delta) > 86439) && (abs(cloud_tt_delta) < 86441)) {
        ROS_WARN("Exception 7 occure.");
        if (latest_data_.second->header.stamp.toSec() < last_data_.second->header.stamp.toSec())
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() + 86400);
        else
          output_timestamp = ros::Time(latest_data_.second->header.stamp.toSec() - 86400);

        exception_occure = true;
      }

      if (exception_occure) {
        // output timestamp from buffer
        std::stringstream ss;
        ss << "(system time, cloud time): ";
        for (int index = 0; index < buffer_size_; index++) {
          double system_tt = it->second->GetData(index).first.toSec();
          double cloud_tt = it->second->GetData(index).second->header.stamp.toSec();
          ss << " (" << std::setprecision(18) << system_tt << ", " << std::setprecision(18) << cloud_tt << ")";
        }
        ROS_INFO(ss.str().c_str());

        ROS_WARN("Modified timestamp from %f to %f in %s", latest_data_.second->header.stamp.toSec(), output_timestamp.toSec(), m_sensor_id_.c_str());
        latest_data_.second->header.stamp = output_timestamp;
      }
      it->second->SetStatus(true);
      m_ready_process_ = false;
      exception_occure = false;

    } catch(const std::exception& e) {
      ROS_ERROR_STREAM("monitor exception: " << e.what());
    }
  }
}

}  // namespace xiaoba_fusion
}  // namespace perception
