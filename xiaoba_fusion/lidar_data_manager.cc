#include "lidar_data_manager.h"
#include "lidar_data_monitor.h"

#include <algorithm>
#include <utility>

#include "common/include/log.h"

namespace perception {
namespace xiaoba_fusion {

LidarData::LidarData()
  : data_status_(true)
{
  circular_buffer_time = boost::circular_buffer<PointCloudData>(5);
}

void LidarData::SetTransform(const Eigen::Affine3d& matrix) {
  lidar2base = matrix;
}

Eigen::Affine3d LidarData::GetTransform() {
  return lidar2base;
}

void LidarData::AddData(const PointCloudData& data) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  circular_buffer_time.push_back(data);
}

bool LidarData::ModifiedLatestData(const PointCloudData& data) {
  if (!circular_buffer_time.empty()) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    circular_buffer_time.pop_back();
    circular_buffer_time.push_back(data);
  } else {
    ROS_WARN("The circular_buffer_time is empty.");
  }
  return true;
}

PointCloudData LidarData::GetData(int index) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  if (index < circular_buffer_time.size())
    return circular_buffer_time.at(index);
  else{
    ROS_ERROR("Failed to get data from buffer");
    return std::make_pair(ros::Time::now(), nullptr);
  }
}

PointCloudData LidarData::GetLatestData() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return circular_buffer_time.back();
}

void LidarData::SetStatus(bool status) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  data_status_ = status;
}

bool LidarData::GetStatus() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return data_status_;
}

bool LidarData::IsBufferFull() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return circular_buffer_time.full();
}

size_t LidarData::GetBufferSize() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return circular_buffer_time.size();
}

LidarDataManager::LidarDataManager()
    : inited_(false)
    , sensor_manager(nullptr)
{
  CHECK_EQ(this->Init(), true);
}

bool LidarDataManager::Init() {
  if (inited_)
    return true;
  if (sensor_manager == nullptr)
    sensor_manager = base::SensorManager::Instance();

  inited_ = true;
  return true;
}

void LidarDataManager::Reset() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  lidar_data_map_.clear();
}

void LidarDataManager::SetMonitorStampFlag(bool flag){
  monitor_stamp_flag_ = flag;
}

std::string LidarDataManager::GetTopicName(const std::string& sensor_name) {
  base::SensorInfo sensor_info;
  if (!sensor_manager->GetSensorInfo(sensor_name, &sensor_info)) {
    ROS_ERROR("Can not get sensor info.");
    return NULL;
  }
  return sensor_info.topic();
}

bool LidarDataManager::pclDataTransform(const sensor_msgs::PointCloud2Ptr& src_msg,
  const sensor_msgs::PointCloud2Ptr& dst_msg, const Eigen::Affine3d& transform) {
  try{
    *dst_msg = *src_msg;
    // Get the index we need
    int x_idx = -1, y_idx = -1, z_idx = -1, i_idx = -1;
    for (size_t d = 0; d < src_msg->fields.size (); ++d){
      if (src_msg->fields[d].name == "x")
        x_idx = d;
      else if(src_msg->fields[d].name == "y")
        y_idx = d;
      else if(src_msg->fields[d].name == "z")
        z_idx = d;
      else if (src_msg->fields[d].name == "intensity")
        i_idx = d;
    }
    if (x_idx == -1 || y_idx == -1 || z_idx == -1){
      std::cerr << "x/y/z coordinates not found!" << std::endl;
      return false;
    }
    // Get the x/y/z field offsets
    int x_offset = 0, y_offset = 0, z_offset = 0, i_offset = 0;
    uint8_t x_datatype = 0, y_datatype = 0, z_datatype = 0, i_datatype = 0;
    x_offset = src_msg->fields[x_idx].offset;
    y_offset = src_msg->fields[y_idx].offset;
    z_offset = src_msg->fields[z_idx].offset;
    x_datatype = src_msg->fields[x_idx].datatype;
    y_datatype = src_msg->fields[y_idx].datatype;
    z_datatype = src_msg->fields[z_idx].datatype;
    if (i_idx > 0) {
      i_offset = src_msg->fields[i_idx].offset;
      i_datatype = src_msg->fields[i_idx].datatype;
      dst_msg->fields[i_idx].datatype = sensor_msgs::PointField::FLOAT32;
    }

    // transform the data points
    Eigen::Matrix<float, 4, 4> tf(transform.matrix().cast <float> ());
    if (src_msg->is_dense) {
      for (size_t index = 0; index < (src_msg->width * src_msg->height); ++index){
        float x = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + x_offset], x_datatype);
        float y = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + y_offset], y_datatype);
        float z = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + z_offset], z_datatype);

        float x_tf = static_cast<float> (tf(0, 0) * x + tf(0, 1) * y + tf(0, 2) * z + tf(0, 3));
        float y_tf = static_cast<float> (tf(1, 0) * x + tf(1, 1) * y + tf(1, 2) * z + tf(1, 3));
        float z_tf = static_cast<float> (tf(2, 0) * x + tf(2, 1) * y + tf(2, 2) * z + tf(2, 3));

        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + x_offset], x_datatype, x_tf);
        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + y_offset], y_datatype, y_tf);
        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + z_offset], z_datatype, z_tf);

        if (i_idx > 0) {
          float intensity = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + i_offset], i_datatype);
          sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + i_offset], sensor_msgs::PointField::FLOAT32, intensity);
        }
      }
    }else {
      for (size_t index = 0; index < (src_msg->width * src_msg->height); ++index) {
        float x = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + x_offset], x_datatype);
        float y = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + y_offset], y_datatype);
        float z = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + z_offset], z_datatype);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
          continue;

        float x_tf = static_cast<float> (tf(0, 0) * x + tf(0, 1) * y + tf(0, 2) * z + tf(0, 3));
        float y_tf = static_cast<float> (tf(1, 0) * x + tf(1, 1) * y + tf(1, 2) * z + tf(1, 3));
        float z_tf = static_cast<float> (tf(2, 0) * x + tf(2, 1) * y + tf(2, 2) * z + tf(2, 3));

        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + x_offset], x_datatype, x_tf);
        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + y_offset], y_datatype, y_tf);
        sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + z_offset], z_datatype, z_tf);
        
        if (i_idx > 0) {
          float intensity = sensor_msgs::readPointCloud2BufferValue<float>(&src_msg->data[index * src_msg->point_step + i_offset], i_datatype);
          sensor_msgs::writePointCloud2BufferValue<float>(&dst_msg->data[index * src_msg->point_step + i_offset], sensor_msgs::PointField::FLOAT32, intensity);
        }
      }    
    }
  } catch(const std::exception& e) {
      ROS_ERROR_STREAM("pcl transform exception: " << e.what());
      return false;
  }
  return true;
}

void LidarDataManager::AddSensorMeasurements(const PointCloudData& cloud_data) {
  ros::Time msgs_stamp = cloud_data.second->header.stamp;
  std::string sensor_id = cloud_data.second->header.frame_id;
  const auto it = lidar_data_map_.find(sensor_id);
  LidarDataPtr lidar_data_ptr_ = nullptr;
  if (it == lidar_data_map_.end()) {
    ROS_INFO("Create new sensor container: %s", sensor_id.c_str());
    base::SensorInfo sensor_info;
    if (!sensor_manager->GetSensorInfo(sensor_id, &sensor_info)) {
      ROS_ERROR("Can not get sensor info.");
      return;
    }
    Eigen::Affine3d lidar2base;
    if (!base::SetSensorExtrinsics(sensor_info.extrinsic(), lidar2base)) {
      ROS_ERROR("%s has no extrinsics files", sensor_id.c_str());
      return;
    }
    lidar_data_ptr_.reset(new LidarData());
    lidar_data_ptr_->SetTransform(lidar2base);
    lidar_data_map_.emplace(sensor_id, lidar_data_ptr_);
  }else {
    lidar_data_ptr_ = it->second;
  }

  sensor_msgs::PointCloud2::Ptr point_msgs(new sensor_msgs::PointCloud2);
  if(!pclDataTransform(cloud_data.second, point_msgs, lidar_data_ptr_->GetTransform())){
    ROS_ERROR("Failed to transform point cloud, sensor_id = %s", sensor_id.c_str());
    return;
  }  

  if(monitor_stamp_flag_)
    lidar_data_ptr_->SetStatus(false);
  else
    lidar_data_ptr_->SetStatus(true);

  lidar_data_ptr_->AddData(std::make_pair(cloud_data.first, point_msgs));

  if(monitor_stamp_flag_)
    LidarDataMonitor::Instance()->Notify(sensor_id);
}

bool LidarDataManager::GetLatestSensorFrames(const std::string& sensor_id, const sensor_msgs::PointCloud2Ptr& frame_ptr) {
  LidarDataMap::iterator it = lidar_data_map_.find(sensor_id);
  if (it == lidar_data_map_.end()) {
    ROS_WARN("Can not find sensor info from data map;");
    return false;
  }
  if (!it->second->IsBufferFull()) {
    ROS_WARN("Data is not available when the system initialized;");
    return false;
  }

  int quary_num = 3;
  while (quary_num--) {
    if (it->second->GetStatus()) {
      *frame_ptr = *(it->second->GetLatestData().second);
      break;
    }
    if(monitor_stamp_flag_)
      LidarDataMonitor::Instance()->Notify(sensor_id);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (quary_num < 1) {
    ROS_WARN("Data status error, Some exception data may be exist;");
    return false;
  }

  return true;
}

LidarDataMap LidarDataManager::GetLidarDataMap() {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return lidar_data_map_;
}

}  // namespace xiaoba_fusion
}  // namespace perception

