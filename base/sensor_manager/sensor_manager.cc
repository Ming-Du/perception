#include "perception/base/sensor_manager/sensor_manager.h"

#include <utility>

#include "common/include/log.h"
#include "common/include/pb_utils.h"


namespace perception {
namespace base {

SensorManager::SensorManager() { CHECK_EQ(this->Init(), true); }

bool SensorManager::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }

  sensor_info_map_.clear();
  distort_model_map_.clear();
  undistort_model_map_.clear();

  const std::string file_path = FLAGS_sensor_meta_path;

  RegisteredSensor sensor_list_proto;
  if (!common::GetProtoFromFile(file_path, sensor_list_proto)) {
    AERROR << "Invalid MultiSensorMeta file: " << FLAGS_sensor_meta_path;
    return false;
  }

  auto AddSensorInfo = [this](const SensorInfo& sensor_info) {
    // sensor_info.name = sensor_meta_proto.name();
    // sensor_info.type = static_cast<SensorType>(sensor_meta_proto.type());
    // sensor_info.orientation =
    //     static_cast<SensorOrientation>(sensor_meta_proto.orientation());
    // sensor_info.frame_id = sensor_meta_proto.name();

    auto pair = sensor_info_map_.insert(
        std::make_pair(sensor_info.name(), sensor_info));
    if (!pair.second) {
      AERROR << "Duplicate sensor name error.";
      return false;
    }

    if (this->IsCamera(sensor_info.type())) {
       if (sensor_info.has_intrinsic() && sensor_info.has_extrinsic()) {
        std::shared_ptr<PinholeCameraModel> undistort_model(new PinholeCameraModel());
        undistort_model->set_width(sensor_info.intrinsic().width());
        undistort_model->set_height(sensor_info.intrinsic().height());
            
        Eigen::Matrix3f intrinsic_params(3, 3); 
        for (int i = 0; i < 9; i++) {
          intrinsic_params(i / 3, i % 3) = sensor_info.intrinsic().matrix(i);
        }   

        undistort_model->set_intrinsic_params(intrinsic_params);
        undistort_model_map_.insert(make_pair(sensor_info.name(), undistort_model));

        //Modify-lijian
        std::shared_ptr<BrownCameraDistortionModel> distort_model(new BrownCameraDistortionModel());
        Eigen::VectorXf camera_params(14);
        for (size_t m = 0; m < 9; ++m) {
          camera_params(m) = sensor_info.intrinsic().matrix(m);
        }
        for (size_t n = 0; n < 5; ++n) {
          camera_params(n+9) = sensor_info.distcoeff().distort_matrix(n);
        }
        distort_model->set_params(sensor_info.intrinsic().width(), sensor_info.intrinsic().height(), 
                                  camera_params);
        distort_model_map_.insert(make_pair(sensor_info.name(), distort_model));
      }
    }

    ROS_INFO_STREAM("Init: register sensor: " << sensor_info.name());
    return true;
  };

  for (const SensorInfo& sensor_meta_info : sensor_list_proto.sensor_info()) {
    if (!AddSensorInfo(sensor_meta_info)) {      
      ROS_ERROR_STREAM("Init: Failed to add sensor_info: " << sensor_meta_info.name());  
      return false;
    }
  }
  base_frame_id_ = sensor_list_proto.base();

  inited_ = true;
  ROS_INFO_STREAM("Init: Init sensor_manager success.");  
  return true;
}

bool SensorManager::IsSensorExist(const std::string& name) const {
  return sensor_info_map_.find(name) != sensor_info_map_.end();
}

bool SensorManager::GetSensorInfo(const std::string& name,
                                  SensorInfo* sensor_info) const {
  if (sensor_info == nullptr) {
    ROS_ERROR("GetSensorInfo: Nullptr error.");
    return false;
  }

  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  }

  *sensor_info = itr->second;
  return true;
}

std::shared_ptr<BaseCameraDistortionModel> SensorManager::GetDistortCameraModel(
    const std::string& name) const {
  const auto& itr = distort_model_map_.find(name);

  return itr == distort_model_map_.end() ? nullptr : itr->second;
}

std::shared_ptr<BaseCameraModel> SensorManager::GetUndistortCameraModel(
    const std::string& name) const {
  const auto& itr = undistort_model_map_.find(name);

  return itr == undistort_model_map_.end() ? nullptr : itr->second;
}

bool SensorManager::IsLidar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  }

  SensorType type = itr->second.type();
  return this->IsLidar(type);
}

bool SensorManager::IsLidar(const SensorType& type) const {
  return type == SensorType::LSLIDAR_C16 ||
         type == SensorType::LSLIDAR_C32 ||
         type == SensorType::LSLIDAR_CH ||
         type == SensorType::LIVOX_HORIZON ||
         type == SensorType::HESAI_XT32 ||
         type == SensorType::VELODYNE_64 ||
         type == SensorType::HESAI_128 ||
         type == SensorType::RSLIDAR_80 ||
         type == SensorType::RSLIDAR_M1 ||
         type == SensorType::ZVISIONLIDAR ||
         type == SensorType::RSLIDAR_HELIOS;
}

bool SensorManager::IsRadar(const SensorType& type) const {
  return type == SensorType::CT_RADAR || 
         type == SensorType::CONTI_RADAR_ARS408 ||
         type == SensorType::CONTI_RADAR_SRR308;

}

bool SensorManager::IsRadar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    ROS_INFO_STREAM("IsRadar: Can't find radar's type!");
    return false;
  }

  SensorType type = itr->second.type();
  return this->IsRadar(type);
}

bool SensorManager::IsCamera(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  }

  SensorType type = itr->second.type();
  return this->IsCamera(type);
}

bool SensorManager::IsCamera(const SensorType& type) const {
  return type == SensorType::CAMERA_6MM || 
         type == SensorType::CAMERA_12MM || 
         type == SensorType::SENSING_30 || 
         type == SensorType::SENSING_60 || 
         type == SensorType::SENSING_120;
}

// Modify(@liuxinyu): obu_test
bool SensorManager::IsObu(const std::string &name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  }

  SensorType type = itr->second.type();
  return this->IsObu(type);
}

bool SensorManager::IsObu(const SensorType &type) const {
  return type == SensorType::V2X ||
         type == SensorType::OBU_RSM;
}

bool SensorManager::IsVidar(const SensorType &type) const {
  return type == SensorType::VIDAR;
}

bool SensorManager::IsVidar(const std::string &name) const {
  auto it = sensor_info_map_.find(name);
  if (it == sensor_info_map_.end()) return false;
  SensorType type = it->second.type();
  return this->IsVidar(type);
}

std::string SensorManager::GetFrameId(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  return itr == sensor_info_map_.end() ? std::string("") : itr->second.name();
}

std::string SensorManager::GetTopic(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  return itr == sensor_info_map_.end() ? std::string("") : itr->second.topic();
}

//Modify @jiangnan: add falcon lidar
bool SensorManager::IsFalconLidar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    ROS_INFO("IsFalconLidar: Can't find falcon lidar's type!");
    return false;
  }
  SensorType type = itr->second.type();
  return this->IsFalconLidar(type);
}
bool SensorManager::IsFalconLidar(const SensorType& type) const {

return type == SensorType::INNOVUSION_FALCON_LIDAR;
//return type == SensorType::LSLIDAR_C32;  //temp
}


}  // namespace base
}  // namespace perception
