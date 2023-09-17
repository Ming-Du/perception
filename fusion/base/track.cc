#include "track.h"
#include <limits>
#include "base/configmanager.h"
namespace perception {
namespace fusion {

int Track::prediction_count_ = 4;
size_t Track::s_track_idx_ = 1;
double Track::s_max_lidar_invisible_period_ = 0.25;   // in second
double Track::s_max_radar_invisible_period_ = 0.50;   // in second
double Track::s_max_camera_invisible_period_ = 0.75;  // in second
// Modify @jiangnan:add falcon lidar
double Track::s_max_falcon_lidar_invisible_period_ = 0.25;  // in second
double Track::s_max_obu_invisible_period_ = 0.75;           // in second
double Track::s_max_vidar_invisible_period_ = 0.25;         // in second

Track::Track() {
  fused_object_.reset(new FusedObject());
}

bool Track::Initialize(SensorObjectPtr obj, bool is_background) {
  Reset();
  int track_id = static_cast<int>(GenerateNewTrackId());
  is_background_ = is_background;
  std::shared_ptr<fusion::Object> fused_base_obj = fused_object_->GetBaseObject();
  std::shared_ptr<const fusion::Object> sensor_base_obj = obj->GetBaseObject();
  *fused_base_obj = *sensor_base_obj;
  fused_base_obj->track_id = track_id;
  
  UpdateWithSensorObject(obj);
  return true;
}

void Track::Reset() {
  fused_object_->GetBaseObject()->track_id = 0;
  lidar_objects_.clear();
  radar_objects_.clear();
  camera_objects_.clear();
  vidar_objects_.clear();
  falcon_lidar_objects_.clear();  // Modify @jiangnan  add falcon lidar
  //  obu_objects_.clear();
  // Modify(@liuxinyu): obu_test
  obu_objects_.clear();
  tracking_period_ = 0.0;
  is_background_ = false;
  is_alive_ = true;
  tracked_times_ = 0;
  toic_prob_ = 0.0;
  is_predicted_ = false;
  prediction_time_ = 0;
  static bool getValueFlag = false;
  if (!getValueFlag) {
    getValueFlag = true;
    bool ret_x = ConfigManager::Instance()->get_value("prediction_count",
                                                      &prediction_count_);
    if (!ret_x) {
      // Modify @jiangnan: if read failed,reset value
      prediction_count_ = 5;
    }
  }
}

SensorObjectConstPtr Track::GetSensorObject(const std::string& sensor_id) const {
  auto lidar_it = lidar_objects_.find(sensor_id);
  if (lidar_it != lidar_objects_.end()) {
    return lidar_it->second;
  }

  auto radar_it = radar_objects_.find(sensor_id);
  if (radar_it != radar_objects_.end()) {
    return radar_it->second;
  }

  auto camera_it = camera_objects_.find(sensor_id);
  if (camera_it != camera_objects_.end()) {
    return camera_it->second;
  }

  // Modify @jiangnan: add falcon lidar
  auto falcon_lidar_it = falcon_lidar_objects_.find(sensor_id);
  if (falcon_lidar_it != falcon_lidar_objects_.end()) {
    return falcon_lidar_it->second;
  }

  // Modify(@liuxinyu): obu_test
  auto obu_it = obu_objects_.find(sensor_id);
  if (obu_it != obu_objects_.end()) {
    return obu_it->second;
  }

  auto vidar_it = vidar_objects_.find(sensor_id);
  if (vidar_it != vidar_objects_.end()) {
    return vidar_it->second;
  }

  return nullptr;
}

SensorObjectConstPtr Track::GetLatestLidarObject() const {
  return GetLatestSensorObject(lidar_objects_);
}

SensorObjectConstPtr Track::GetLatestRadarObject() const {
  return GetLatestSensorObject(radar_objects_);
}

SensorObjectConstPtr Track::GetLatestCameraObject() const {
  return GetLatestSensorObject(camera_objects_);
}
// Modify @jiangnan:add falcon lidar
SensorObjectConstPtr Track::GetLatestFalconLidarObject() const {
  return GetLatestSensorObject(falcon_lidar_objects_);
}

SensorObjectConstPtr Track::GetLatestObuObject() const {
  return GetLatestSensorObject(obu_objects_);
}

SensorObjectConstPtr Track::GetLatestVidarObject() const {
  return GetLatestSensorObject(vidar_objects_);
}

SensorObjectConstPtr Track::GetLatestSensorObject(const SensorId2ObjectMap& objects) const {
  SensorObjectConstPtr obj = nullptr;
  // printf("\033[31m latest_objects_ is %d.\033[0m\n", objects.size());
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    if (obj == nullptr || obj->GetTimestamp() < it->second->GetTimestamp()) {
      obj = it->second;
    }
  }
  return obj;
}

void Track::SetAlive(bool is_alive) {
  is_alive_ = is_alive;
}

size_t Track::GenerateNewTrackId() {
  int ret_track_id = static_cast<int>(s_track_idx_);
  if (s_track_idx_ == std::numeric_limits<unsigned int>::max()) {
    s_track_idx_ = 1;
  } else {
    s_track_idx_++;
  }
  return ret_track_id;
}

void Track::UpdateSensorObject(SensorId2ObjectMap* objects, const SensorObjectPtr& obj) {
  std::string sensor_id = obj->GetSensorId();
  auto it = objects->find(sensor_id);
  if (it == objects->end()) {
    (*objects)[sensor_id] = obj;
  } else {
    it->second = obj;
  }
}

void Track::UpdateWithSensorObject(const SensorObjectPtr& obj) {
  std::string sensor_id = obj->GetSensorId();
  SensorId2ObjectMap* objects = nullptr;
  if (IsLidar(obj)) {
    objects = &lidar_objects_;
  } else if (IsFalconLidar(obj)) {
    objects = &falcon_lidar_objects_;
  } else if (IsRadar(obj)) {
    objects = &radar_objects_;
  } else if (IsCamera(obj)) {
    objects = &camera_objects_;
    // Modify(@liuxinyu): obu_test
  } else if (IsObu(obj)) {
    objects = &obu_objects_;
  } else if (IsVidar(obj)) {
    objects = &vidar_objects_;
  } else {
    return;
  }

  // Modify@jiangnan:Determine whether the fusion tracker is rb
  if (!(IsCamera(obj)) && !(IsRadar(obj))) {
    fused_object_->GetBaseObject()->is_lidar_rb = obj->GetBaseObject()->is_lidar_rb;
  }
  // update roadType
  if (IsLidar(obj) || IsFalconLidar(obj)) {
    fused_object_->GetBaseObject()->status = obj->GetBaseObject()->status;
  }

  UpdateSensorObject(objects, obj);
  double time_diff = obj->GetTimestamp() - fused_object_->GetTimestamp();
  tracking_period_ += time_diff;

  UpdateSensorObjectWithMeasurement(&lidar_objects_, sensor_id, obj->GetTimestamp(),
                                    s_max_lidar_invisible_period_);
  UpdateSensorObjectWithMeasurement(&radar_objects_, sensor_id, obj->GetTimestamp(),
                                    s_max_radar_invisible_period_);
  UpdateSensorObjectWithMeasurement(&camera_objects_, sensor_id, obj->GetTimestamp(),
                                    s_max_camera_invisible_period_);
  UpdateSensorObjectWithMeasurement(
      &falcon_lidar_objects_, sensor_id, obj->GetTimestamp(),
      s_max_falcon_lidar_invisible_period_);  // Modify @jiangnan : add falcon lidar
  // Modify(@liuxinyu): obu_test
  UpdateSensorObjectWithMeasurement(&obu_objects_, sensor_id, obj->GetTimestamp(),
                                    s_max_obu_invisible_period_);
  // ming.du
  UpdateSensorObjectWithMeasurement(&vidar_objects_, sensor_id, obj->GetTimestamp(),
                                    s_max_vidar_invisible_period_);

  if (is_background_) {
    return UpdateWithSensorObjectForBackground(obj);
  }

  fused_object_->GetBaseObject()->latest_tracked_time = obj->GetTimestamp();
  UpdateSupplementState(obj);
  UpdateUnfusedState(obj);
  is_alive_ = true;
}

void Track::UpdateWithoutSensorObject(const std::string& sensor_id, double measurement_timestamp) {
  UpdateSensorObjectWithoutMeasurement(&lidar_objects_, sensor_id, measurement_timestamp,
                                       s_max_lidar_invisible_period_);
  UpdateSensorObjectWithoutMeasurement(&radar_objects_, sensor_id, measurement_timestamp,
                                       s_max_radar_invisible_period_);
  UpdateSensorObjectWithoutMeasurement(&camera_objects_, sensor_id, measurement_timestamp,
                                       s_max_camera_invisible_period_);
  UpdateSensorObjectWithoutMeasurement(
      &falcon_lidar_objects_, sensor_id, measurement_timestamp,
      s_max_falcon_lidar_invisible_period_);  // Modify @jiangnan : add falcon lidar
  // Modify(@liuxinyu): obu_test
  UpdateSensorObjectWithoutMeasurement(&obu_objects_, sensor_id, measurement_timestamp,
                                       s_max_obu_invisible_period_);
  // ming.du
  UpdateSensorObjectWithoutMeasurement(&vidar_objects_, sensor_id, measurement_timestamp,
                                       s_max_vidar_invisible_period_);

  UpdateSupplementState();
  // Modify @jiangnan:update timestamp (without sensor object)
  fused_object_->GetBaseObject()->latest_tracked_time = measurement_timestamp;
  is_alive_ = (!lidar_objects_.empty()) || (!radar_objects_.empty()) ||
              (!camera_objects_.empty()) || (!obu_objects_.empty()) || (!vidar_objects_.empty()) ||
              (!falcon_lidar_objects_.empty());
}

void Track::UpdateSensorObjectWithoutMeasurement(SensorId2ObjectMap* objects,
                                                 const std::string& sensor_id,
                                                 double measurement_timestamp,
                                                 double max_invisible_period) {
  for (auto it = objects->begin(); it != objects->end();) {
    double period = measurement_timestamp - it->second->GetTimestamp();
    if (it->first == sensor_id) {
      it->second->SetInvisiblePeriod(period);
    } else if (it->second->GetInvisiblePeriod() > 0.0) {
      it->second->SetInvisiblePeriod(period);
    }

    if (period > max_invisible_period) {
      it->second = nullptr;
      it = objects->erase(it);
    } else {
      ++it;
    }
  }
}

void Track::UpdateSensorObjectWithMeasurement(SensorId2ObjectMap* objects,
                                              const std::string& sensor_id,
                                              double measurement_timestamp,
                                              double max_invisible_period) {
  for (auto it = objects->begin(); it != objects->end();) {
    if (it->first != sensor_id) {
      double period = measurement_timestamp - it->second->GetTimestamp();
      if (period > max_invisible_period) {
        it->second = nullptr;
        it = objects->erase(it);
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void Track::UpdateSupplementState(const SensorObjectPtr& src_object) {
  std::shared_ptr<fusion::Object> dst_obj = fused_object_->GetBaseObject();
  if (src_object != nullptr) {
    std::shared_ptr<const fusion::Object> src_obj = src_object->GetBaseObject();
    if (IsLidar(src_object)) {
      dst_obj->lidar_supplement = src_obj->lidar_supplement;
    } else if (IsRadar(src_object)) {
      dst_obj->radar_supplement = src_obj->radar_supplement;
    } else if (IsCamera(src_object)) {
      dst_obj->camera_supplement = src_obj->camera_supplement;
    } else if (IsFalconLidar(src_object)) {
      dst_obj->falcon_lidar_supplement = src_obj->falcon_lidar_supplement;
    }  // Modify @jiangnan
    else if (IsObu(src_object)) {
      dst_obj->obu_supplement = src_obj->obu_supplement;
    } else if (IsVidar(src_object)) {
      dst_obj->vidar_supplement = src_obj->vidar_supplement;
    }
  }

  if (lidar_objects_.empty()) {
    dst_obj->lidar_supplement.Reset();
  }
  if (radar_objects_.empty()) {
    dst_obj->radar_supplement.Reset();
  }
  if (camera_objects_.empty()) {
    dst_obj->camera_supplement.Reset();
  }
  // Modify @jiangnan :add falcon lidar
  if (falcon_lidar_objects_.empty()) {
    dst_obj->falcon_lidar_supplement.Reset();
  }
  // Modify(@liuxinyu): obu_test
  if (obu_objects_.empty()) {
    dst_obj->obu_supplement.Reset();
  }
  if (vidar_objects_.empty()) {
    dst_obj->vidar_supplement.Reset();
  }
}

void Track::UpdateUnfusedState(const SensorObjectPtr& src_object) {
  std::shared_ptr<fusion::Object> dst_obj = fused_object_->GetBaseObject();
  std::shared_ptr<const fusion::Object> src_obj = src_object->GetBaseObject();
  if (IsLidar(src_object) || IsFalconLidar(src_object)) {
    dst_obj->confidence = src_obj->confidence;
    dst_obj->velocity_converged = src_obj->velocity_converged;
  } else if (IsRadar(src_object)) {
    // update nothing temporarily
  } else if (IsCamera(src_object)) {
    dst_obj->confidence = src_obj->confidence;
  }
  // Modify(@liuxinyu): obu_test
  else if (IsObu(src_object)) {
    // update nothing temporarily
  } else if (IsVidar(src_object)) {
    dst_obj->confidence = src_obj->confidence;
  }
}

void Track::UpdateWithSensorObjectForBackground(const SensorObjectPtr& obj) {
  std::shared_ptr<fusion::Object> fused_base_object = fused_object_->GetBaseObject();
  std::shared_ptr<const fusion::Object> measurement_base_object = obj->GetBaseObject();
  int track_id = fused_base_object->track_id;
  *fused_base_object = *measurement_base_object;
  fused_base_object->track_id = track_id;
}

void Track::UpdateWithoutSensorObjectForBackground(const std::string& sensor_id,
                                                   double measurement_timestamp) {}

bool Track::IsVisible(const std::string& sensor_id) const {
  SensorObjectConstPtr sensor_obj = GetSensorObject(sensor_id);
  return (sensor_obj != nullptr && sensor_obj->GetInvisiblePeriod() < 1.0e-6);
}

bool Track::IsLidarVisible() const {
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < s_max_lidar_invisible_period_) {
      return true;
    }
  }
  return false;
}

bool Track::IsRadarVisible() const {
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}

bool Track::IsCameraVisible() const {
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}
// Modify @jiangnan:add falcon lidar:
bool Track::IsFalconLidarVisible() const {
  for (auto it = falcon_lidar_objects_.begin(); it != falcon_lidar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < s_max_falcon_lidar_invisible_period_) {
      return true;
    }
  }
  return false;
}
// ming.du
bool Track::IsVidarVisible() const {
  for (auto it = vidar_objects_.begin(); it != vidar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}

bool Track::IsObuVisible() const {
  for (auto it = obu_objects_.begin(); it != obu_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < s_max_obu_invisible_period_) {
      return true;
    }
  }
  return false;
}

bool Track::IsPredicted() const {
  return is_predicted_;
}
//bool Track::IsMultiOverlapTrack() const {
//  return is_multi_overlap_track_;
//}


void Track::SetPredictionState(bool is_predict) {
  // Modify @jiangnan: Judge the predict_state  according to the number of times
  if (is_predict) {
    prediction_time_++;
  } else {
    is_predicted_ = false;
    prediction_time_ = 0;
  }
  if (prediction_time_ >= prediction_count_) {
    is_predicted_ = true;
  }
}

//void Track::SetMultiOverlapTrack(bool flag) {
//  this->is_multi_overlap_track_ = flag;
//}

std::string Track::DebugString() const {
  std::ostringstream oss;
  oss << "fusion_track[id: " << this->GetTrackId() << ", fused_object("
      << fused_object_->GetBaseObject()->ToString() << ")\n";

  oss << "lidar_measurments[number: " << lidar_objects_.size() << ",";
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";

  oss << "radar_measurments[number: " << radar_objects_.size() << ",";
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";

  oss << "camera_measurments[number: " << camera_objects_.size() << ",";
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";

  oss << "obu_measurments[number: " << obu_objects_.size() << ",";
  for (auto it = obu_objects_.begin(); it != obu_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";
  return oss.str();
}
}  // namespace fusion
}  // namespace perception
