#include "pbf_shape_fusion.h"

#include "base/configmanager.h"
#include "fstream"
#include "iostream"
namespace perception {
namespace fusion {

bool PbfShapeFusion::s_use_camera_3d_ = true;
float PbfShapeFusion::s_camera_radar_time_diff_th_ = 0.3f;
float PbfShapeFusion::s_obu_radar_time_diff_th_ = 0.3f;

bool PbfShapeFusion::Init() {
  ConfigManager::Instance()->get_value("vehicle", &vehicle_);
  return true;
}

void PbfShapeFusion::UpdateWithMeasurement(const SensorObjectPtr measurement,
                                           double target_timestamp) {
    if (!isConfirmedDetection_) {
        isConfirmedDetection_ = ComputeTrackConfirmedState(measurement);
    }
    if (IsCamera(measurement)) return;
    UpdateSizeKF(measurement);
    UpdateYawByKF(measurement);
    UpdatePedestrianYaw(measurement);
    UpdatePolygon(measurement);
    UpdateCenter(measurement);
    lastposition_ = track_ref_->GetFusedObject()->GetBaseObject()->position;
                                           
}
  
void PbfShapeFusion::UpdateWithoutMeasurement(const std::string &sensor_id,
                                              double measurement_timestamp,
                                              double target_timestamp) {
  if(track_ref_->IsPredicted()){
    UpdatePolygon();
  }
  lastposition_ = track_ref_->GetFusedObject()->GetBaseObject()->position;
}

std::string PbfShapeFusion::Name() const { return "PbfShapeFusion"; }
void PbfShapeFusion::ComputeBox(const SensorObjectConstPtr& measurement, const Eigen::Vector3f& size, PointCloud<PointD>& pts) {
    //求出最近点
    fusion::ObjectConstPtr src_obj = measurement->GetBaseObject();
    float yaw = src_obj->theta + measurement->GetBaseObject()->host_yaw;
    if (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    }
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    double min_y_value = 9999999;
    int min_index = 0;
    std::vector<PointD> polygon_others;

    for (int i = 0; i < src_obj->polygon_utm.size(); ++i) {
        PointD pt_f_other;
        ConvertfromUtm2Other(yaw, src_obj->position, src_obj->polygon_utm[i], &pt_f_other);
        polygon_others.emplace_back(pt_f_other);
        double value = src_obj->polygon_ego[i].y * src_obj->polygon_ego[i].y + src_obj->polygon_ego[i].x * src_obj->polygon_ego[i].x;
        if (min_y_value > value) {
            min_y_value = value;
            min_index = i;
        }
    }
    //根据size调整坐标
    std::vector<PointD> new_polygon_other = OptBoxbySize(polygon_others, min_index, size);
    if (!CheckBoxValid(new_polygon_other)) {
        new_polygon_other = polygon_others;
    }
    for (int i = 0; i < new_polygon_other.size(); ++i) {
        PointD temppt_utm;
        ConvertfromOther2Utm(yaw, src_obj->position, new_polygon_other[i], &temppt_utm);
        pts.push_back(temppt_utm);
    }
}
std::vector<PointD> PbfShapeFusion::OptBoxbySize(const std::vector<PointD>& polygon_others,
                                                 const int& min_index,
                                                 const Eigen::Vector3f& size) {
    std::vector<PointD> new_polygon_other;                                               
    if (polygon_others[min_index].x > 0 && polygon_others[min_index].y > 0) {
        int index_ld, index_rd, index_rt;
        index_ld = (min_index + 1) > 3 ? 0 : (min_index + 1);
        index_rd = (index_ld + 1) > 3 ? 0 : (index_ld + 1);
        index_rt = (index_rd + 1) > 3 ? 0 : (index_rd + 1);
        PointD pt_rt, pt_ld, pt_rd;
        pt_ld.x = polygon_others[min_index].x - size[0];
        pt_ld.y = polygon_others[index_ld].y;
        pt_rt.x = polygon_others[index_rt].x;
        pt_rt.y = polygon_others[index_rt].y;
        pt_rd.x = polygon_others[min_index].x - size[0];
        pt_rd.y = polygon_others[index_rd].y;
        new_polygon_other.emplace_back(polygon_others[min_index]);
        new_polygon_other.emplace_back(pt_ld);
        new_polygon_other.emplace_back(pt_rd);
        new_polygon_other.emplace_back(pt_rt);
    }  //左前轮
    if (polygon_others[min_index].x < 0 && polygon_others[min_index].y > 0) {
        int index_lt, index_rd, index_rt;
        index_rd = (min_index + 1) > 3 ? 0 : (min_index + 1);
        index_rt = (index_rd + 1) > 3 ? 0 : (index_rd + 1);
        index_lt = (index_rt + 1) > 3 ? 0 : (index_rt + 1);
        PointD pt_rt, pt_lt, pt_rd;
        pt_rt.x = polygon_others[min_index].x + size[0];
        pt_rt.y = polygon_others[index_rt].y;
        pt_lt.x = polygon_others[min_index].x + size[0];
        pt_lt.y = polygon_others[index_lt].y;
        pt_rd.x = polygon_others[index_rd].x;
        pt_rd.y = polygon_others[index_rd].y;
        new_polygon_other.emplace_back(pt_lt);
        new_polygon_other.emplace_back(polygon_others[min_index]);
        new_polygon_other.emplace_back(pt_rd);
        new_polygon_other.emplace_back(pt_rt);
    }  //左后
    if (polygon_others[min_index].x < 0 && polygon_others[min_index].y < 0) {
        int index_lt, index_ld, index_rt;
        index_rt = (min_index + 1) > 3 ? 0 : (min_index + 1);
        index_lt = (index_rt + 1) > 3 ? 0 : (index_rt + 1);
        index_ld = (index_lt + 1) > 3 ? 0 : (index_lt + 1);
        PointD pt_rt, pt_lt, pt_ld;
        pt_rt.x = polygon_others[min_index].x + size[0];
        pt_rt.y = polygon_others[index_rt].y;
        pt_lt.x = polygon_others[min_index].x + size[0];
        pt_lt.y = polygon_others[index_lt].y;
        pt_ld.x = polygon_others[index_ld].x;
        pt_ld.y = polygon_others[index_ld].y;
        new_polygon_other.emplace_back(pt_lt);
        new_polygon_other.emplace_back(pt_ld);
        new_polygon_other.emplace_back(polygon_others[min_index]);
        new_polygon_other.emplace_back(pt_rt);
    }  //右后轮
    if (polygon_others[min_index].x > 0 && polygon_others[min_index].y < 0) {
        int index_lt, index_ld, index_rd;
        index_lt = (min_index + 1) > 3 ? 0 : (min_index + 1);
        index_ld = (index_lt + 1) > 3 ? 0 : (index_lt + 1);
        index_rd = (index_ld + 1) > 3 ? 0 : (index_ld + 1);
        PointD pt_rd, pt_lt, pt_ld;
        pt_rd.x = polygon_others[min_index].x - size[0];
        pt_rd.y = polygon_others[index_rd].y;
        pt_lt.x = polygon_others[index_lt].x;
        pt_lt.y = polygon_others[index_lt].y;
        pt_ld.x = polygon_others[min_index].x - size[0];
        pt_ld.y = polygon_others[index_ld].y;
        new_polygon_other.emplace_back(pt_lt);
        new_polygon_other.emplace_back(pt_ld);
        new_polygon_other.emplace_back(pt_rd);
        new_polygon_other.emplace_back(polygon_others[min_index]);
    }  //右前轮
    return new_polygon_other;
}
bool PbfShapeFusion::CheckBoxValid(std::vector<PointD>& box) {
    int errocout1 = 0, errocout2 = 0, errocout3 = 0, errocout4 = 0;
    for (int i = 0; i < box.size(); ++i) {
        //检查一下合理性
        if (box[i].x > 0 && box[i].y > 0) {
            errocout1++;
        }
        if (box[i].x < 0 && box[i].y > 0) {
            errocout2++;
        }
        if (box[i].x < 0 && box[i].y < 0) {
            errocout3++;
        }
        if (box[i].x > 0 && box[i].y < 0) {
            errocout4++;
        }
    }
    if (errocout1 > 1 || errocout2 > 1 || errocout3 > 1 || errocout4 > 1) {
        return false;
    } else {
        return true;
    }
}

bool PbfShapeFusion::ComputeNearestPt(const PointCloud<PointD> &pts,
                                      const Eigen::Vector3d &originpt,
                                      PointD *nesrestpt) {
  float mindist = INT_MAX;
  for (auto pt : pts) {
    float distance = (pt.x - originpt[0]) * (pt.x - originpt[0]) +
                     (pt.y - originpt[1]) * (pt.y - originpt[1]);
    if (mindist > distance) {
      mindist = distance;
      *nesrestpt = pt;
    }
  }
  return true;
}

bool PbfShapeFusion::ConvertfromUtm2Other(float otheryawutm,
                                          Eigen::Vector3d otherpositionutm,
                                          PointD srcpt, PointD *dstpt) {
  double tcos_host = std::cos(otheryawutm);
  double tsin_host = std::sin(otheryawutm);
  double devx = srcpt.x - otherpositionutm[0];
  double devy = srcpt.y - otherpositionutm[1];
  dstpt->x = devx * tcos_host + devy * tsin_host;
  dstpt->y = -devx * tsin_host + devy * tcos_host;
  dstpt->z = srcpt.z;
  return true;
}
bool PbfShapeFusion::ConvertfromOther2Utm(float otheryawutm,
                                          Eigen::Vector3d otherpositionutm,
                                          PointD srcpt, PointD *dstpt) {
  float cos_host = std::cos(otheryawutm);
  float sin_host = std::sin(otheryawutm);
  dstpt->x = (srcpt.x * cos_host - srcpt.y * sin_host) + otherpositionutm[0];
  dstpt->y = (srcpt.x * sin_host + srcpt.y * cos_host) + otherpositionutm[1];
  dstpt->z = srcpt.z;
  return true;
}
bool PbfShapeFusion::UpdatePedestrianYaw(
    const SensorObjectConstPtr &measurement) {
  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();
  if (obj->type != ObjectType::PEDESTRIAN) {
    return false;
  }
  if (history_positions_.empty()) {
    StoreHistoryPositions();
    return false;
  }
  double delt_px =
      history_positions_.back().x() - history_positions_.front().x();
  double delt_py =
      history_positions_.back().y() - history_positions_.front().y();
  double yaw_temp = atan2(delt_py, delt_px);
  yaw_temp = yaw_temp < 0 ? yaw_temp + 2 * M_PI : yaw_temp;

  obj->yaw =
      yaw_temp;  // fabs(yaw_temp - obj->yaw) <= 0.4 ? obj->yaw : yaw_temp;

  auto converttheta = [](float yaw, float host_yaw) -> float {
    float yaw_baselink = yaw - host_yaw;
    yaw_baselink = yaw_baselink < 0 ? (yaw_baselink + 2 * M_PI) : yaw_baselink;
    return yaw_baselink > M_PI ? (yaw_baselink - 2 * M_PI) : yaw_baselink;
  };
  obj->theta = converttheta(obj->yaw, obj->host_yaw);
  StoreHistoryPositions();
  return true;
}
void PbfShapeFusion::StoreHistoryPositions() {
  if (history_positions_.size() >= historypositionnum_) {
    history_positions_.erase(history_positions_.begin());
  }
  history_positions_.emplace_back(
      track_ref_->GetFusedObject()->GetBaseObject()->position);
}

void PbfShapeFusion::InitBayes() {
  bayes_x_prob_ = 0.6;
  bayes_anti_x_prob_ = 1.0 - 0.6;
  x_drec_ = true;
}

void PbfShapeFusion::InitKalman(const float &srcyaw) {
  yaw_kf_ = srcyaw;
  P_ = .0;
  if (!ConfigManager::Instance()->get_value("yaw_kalman_q", &Q_)) {
    Q_ = M_PI_2;
  }
  if (!ConfigManager::Instance()->get_value("yaw_kalman_r", &R_)) {
    R_ = M_PI;
  }
  Gain_ = .0;
  ncoutbad_ = 0;
  kalmantimes_ = 0;
}

bool PbfShapeFusion::CheckBagYaw(const float &dstyaw, const float &srcyaw) {
  double diff_theta = std::fabs(dstyaw - srcyaw);
  if (diff_theta > M_PI) {  // 归一化到0-pi
    diff_theta = 2 * M_PI - diff_theta;
  }
  if (diff_theta > M_PI_2) {  // 归一化到0-pi/2
    diff_theta = M_PI - diff_theta;
  }
  if (diff_theta > 30.0 / 90.0 * M_PI_2) {
    return true;
  } else {
    return false;
  }
}

void PbfShapeFusion::BayesFilter(fusion::ObjectConstPtr src_obj) {
  // 计算本次的概率，概率先分组
  auto sigmoid = [this](double x) { return 1.0f / (1.0f + expf(-x)); };
  float bayes_prob[2];
  float diffthresh_ = 15 * M_PI / 180;
  double speed = std::sqrt(std::pow(src_obj->velocity[0], 2) +
                           std::pow(src_obj->velocity[1], 2));
  double diff_yaw = std::fabs(yaw_kf_ - src_obj->yaw);
  if (diff_yaw > M_PI) {  // 归一化到0-pi
    diff_yaw = 2 * M_PI - diff_yaw;
  }
  if (diff_yaw > 150.0 / 180.0 * M_PI) {
    bayes_prob[1] = sigmoid(speed);
        if (bayes_prob[1] < 0.1) {
            bayes_prob[1] = 0.1;
        }
        if (bayes_prob[1] > 0.9) {
            bayes_prob[1] = 0.9;
        }
    bayes_prob[0] = 1.0 - sigmoid(speed);
  } else {
    bayes_prob[0] = sigmoid(speed);
        if (bayes_prob[0] < 0.1) {
            bayes_prob[0] = 0.1;
        }
        if (bayes_prob[0] > 0.9) {
            bayes_prob[0] = 0.9;
        }
    bayes_prob[1] = 1.0 - sigmoid(speed);
  }
  // 贝叶斯滤波概率，判断航向
  bayes_x_prob_ *= bayes_prob[0];
  bayes_anti_x_prob_ *= bayes_prob[1];
  double sum = bayes_x_prob_ + bayes_anti_x_prob_;
  bayes_x_prob_ /= sum;
  bayes_anti_x_prob_ /= sum;
  if (bayes_x_prob_ > bayes_anti_x_prob_) {
    x_drec_ = true;
  } else {
    x_drec_ = false;
  }
}

void PbfShapeFusion::KalmanFilter(const float &srcyaw) {
  // 让滤波的输入值连续起来
  float mear_yaw = srcyaw;
  float diff_yaw = std::fabs(yaw_kf_ - srcyaw);
  if (diff_yaw > M_PI) {
    diff_yaw = 2 * M_PI - diff_yaw;
  }
  if (diff_yaw > 150.0 / 180.0 * M_PI) {
    mear_yaw = srcyaw + M_PI;
    mear_yaw = mear_yaw > 2 * M_PI ? (mear_yaw - 2 * M_PI) : mear_yaw;
  }
  // 4.卡尔曼滤波，对航向进行滤波
  kalmantimes_++;
  if (kalmantimes_ > 10) {
    if (!ConfigManager::Instance()->get_value("yaw_kalman_q", &Q_)) {
      Q_ = M_PI_2;
    }
    // Q_ = M_PI_2;
    if (!ConfigManager::Instance()->get_value("yaw_kalman_r", &R_)) {
      R_ = M_PI;
    }
  }
  P_ += Q_;
  Gain_ = P_ / (P_ + R_);
  float dy = remainder((mear_yaw - yaw_kf_),
                       (2 * M_PI));  // remainder(mear_yaw - yaw_kf_,2*M_PI);
  yaw_kf_ = fmod((yaw_kf_ + Gain_ * dy), (2 * M_PI));
  yaw_kf_ = yaw_kf_ > 0 ? yaw_kf_ : (2 * M_PI + yaw_kf_);
  P_ = (1 - Gain_) * P_;
}

void PbfShapeFusion::UpdateYawByKF(const SensorObjectConstPtr &measurement) {
  fusion::ObjectConstPtr src_obj = measurement->GetBaseObject();
  fusion::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();

  if (src_obj->type == ObjectType::UNKNOWN) {
    SensorObjectConstPtr latest_falcon_lidar =
        track_ref_->GetLatestFalconLidarObject();
    SensorObjectConstPtr latest_vidar = track_ref_->GetLatestVidarObject();
    SensorObjectConstPtr latest_lidar = track_ref_->GetLatestLidarObject();
    bool flag = false;
    if (latest_lidar != nullptr) {
      if (latest_lidar->GetBaseObject()->type != ObjectType::UNKNOWN) {
        flag = true;
      }
    }
    if (latest_falcon_lidar != nullptr || latest_vidar != nullptr || flag) {
      return;
    }
  }
  // 1.初始化
  if (!init_) {
    InitBayes();
    InitKalman(src_obj->yaw);
    dst_obj->yaw = yaw_kf_ = src_obj->yaw;
    init_ = true;
    return;
  }

  if (CheckBagYaw(dst_obj->yaw, src_obj->yaw)) {
    ncoutbad_++;
    if (ncoutbad_ > 6) {
      InitKalman(src_obj->yaw);
      dst_obj->yaw = src_obj->yaw;
    }
    return;
  }
  ncoutbad_ = 0;
  BayesFilter(src_obj);
  KalmanFilter(src_obj->yaw);
  dst_obj->yaw = x_drec_
                     ? yaw_kf_
                     : (yaw_kf_ > M_PI ? (yaw_kf_ - M_PI) : (yaw_kf_ + M_PI));
  auto converttheta = [](float yaw, float host_yaw) -> float {
    float yaw_baselink = yaw - host_yaw;
    yaw_baselink = yaw_baselink < 0 ? (yaw_baselink + 2 * M_PI) : yaw_baselink;
    return yaw_baselink > M_PI ? (yaw_baselink - 2 * M_PI) : yaw_baselink;
  };

  dst_obj->theta = converttheta(dst_obj->yaw, src_obj->host_yaw);
}
void PbfShapeFusion::InitKalmanSizeFilter(
    const SensorObjectConstPtr &measurement) {
  p_matrix_.setIdentity(3, 3);  
  p_matrix_(0, 0) = 1;
  p_matrix_(1, 1) = 1;
  p_matrix_(2, 2) = 1;

  opt_states_.setZero(3, 1); 
  opt_states_(0) = measurement->GetBaseObject()->size[0];
  opt_states_(1) = measurement->GetBaseObject()->size[1];
  opt_states_(2) = measurement->GetBaseObject()->size[2];

  transform_matrix_.setIdentity(3, 3); 
  transform_matrix_(0, 0) = 1;
  transform_matrix_(1, 1) = 1;
  transform_matrix_(2, 2) = 1;

  q_matrix_.setZero(3, 3);
  float t_q;
  if (!ConfigManager::Instance()->get_value("sizel_kalman_q", &t_q)) {
    t_q = 0.01;
  }
  q_matrix_(0, 0) = t_q;
  if (!ConfigManager::Instance()->get_value("sizew_kalman_q", &t_q)) {
    t_q = 0.1;
  }
  q_matrix_(1, 1) = t_q;
  if (!ConfigManager::Instance()->get_value("sizeh_kalman_q", &t_q)) {
    t_q = 1;
  }
  q_matrix_(2, 2) = t_q;

  r_matrix_.setIdentity(3, 3);
  if (isConfirmedDetection_) {
    if (!ConfigManager::Instance()->get_value("sizel_kalman_r", &t_q)) {
      t_q = 100;
    }
    r_matrix_(0, 0) = t_q;
    if (!ConfigManager::Instance()->get_value("sizew_kalman_r", &t_q)) {
      t_q = 100;
    }
    r_matrix_(1, 1) = t_q;
    if (!ConfigManager::Instance()->get_value("sizeh_kalman_r", &t_q)) {
      t_q = 1;
    }
    r_matrix_(2, 2) = t_q;
  } else {
    r_matrix_(0, 0) = 1;
    r_matrix_(1, 1) = 1;
    r_matrix_(2, 2) = 1;
  }

  if (!kalman_filter_.Init(opt_states_, p_matrix_)) {
    // return false;
  }
}
void PbfShapeFusion::UpdateSizeKF(const SensorObjectConstPtr &measurement) {
    // 1.RB的测量值的size不参与更新
    if (measurement->GetBaseObject()->type == ObjectType::UNKNOWN ||
            measurement->GetBaseObject()->type == ObjectType::VEGETATION ||
            IsRadar(measurement))
            return;
    //判断测量值的size与融合的差距，若差距较大则重置
    Eigen::Vector3f m_size = measurement->GetBaseObject()->size;
    Eigen::Vector3f f_size = track_ref_->GetFusedObject()->GetBaseObject()->size;
    float diff = std::max(abs(m_size[0] - f_size[0]), abs(m_size[1] - f_size[1]));
    if(diff > 2.0){
        initsizekm_ = false;
    }
  // 2.初始化滤波器
  if (!initsizekm_) {
    InitKalmanSizeFilter(measurement);
    initsizekm_ = true;
  }

  // 3.卡尔曼更新
  kalman_filter_.Predict(transform_matrix_, q_matrix_);
  Eigen::Vector3d observation;
  observation(0) = measurement->GetBaseObject()->size[0];
  observation(1) = measurement->GetBaseObject()->size[1];
  observation(2) = measurement->GetBaseObject()->size[2];
  kalman_filter_.Correct(observation, r_matrix_);
  // 4.获取
  float length = kalman_filter_.GetStates()(0);
  float width = kalman_filter_.GetStates()(1);
  float height = kalman_filter_.GetStates()(2);
  track_ref_->GetFusedObject()->GetBaseObject()->size[0] = length;
  track_ref_->GetFusedObject()->GetBaseObject()->size[1] = width;
  track_ref_->GetFusedObject()->GetBaseObject()->size[2] = height;
}
bool PbfShapeFusion::ComputeTrackConfirmedState(
    const SensorObjectConstPtr &measurement) {
  bool ret = false;
  if (lostAI_age_ > 20) {
    lostAI_age_ = 0;
    matchAI_times_ = 0;
    matchTotal_times_ = 0;
  }
  matchTotal_times_++;
  if (measurement->GetBaseObject()->type != ObjectType::UNKNOWN) {
    matchAI_times_++;
    lostAI_age_ = 0;
  } else {
    lostAI_age_++;
  }

  float corver = (float)(matchAI_times_ / (float)matchTotal_times_);
  if ((matchTotal_times_ > 20 && corver >= 0.5) || matchAI_times_ > 40) {
    ret = true;
  }
  return ret;
}
void PbfShapeFusion::UpdatePolygon(const SensorObjectConstPtr& measurement) {
    PointCloud<PointD> new_polygon;
    fusion::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
    // 1.check type
    if (measurement->GetBaseObject()->type == ObjectType::UNKNOWN ||
        measurement->GetBaseObject()->type == ObjectType::VEGETATION ||
        IsRadar(measurement)) {
        new_polygon = measurement->GetBaseObject()->polygon_utm;
    } else {
        // 2.create box
        PointCloud<PointD> temp_polygon;
        ComputeBox(measurement, dst_obj->size, new_polygon);
    }
    // 3.select falcon
    dst_obj->polygon_utm = new_polygon;
    if (IsLidar(measurement) || IsRadar(measurement)) {
        SensorObjectConstPtr falcon_sensor_object = track_ref_->GetLatestFalconLidarObject();
        if (falcon_sensor_object != nullptr) {
            //计算一下时差
            double ftime = falcon_sensor_object->GetTimestamp();
            double mtime = measurement->GetTimestamp();
            //计算一下图达通的距离
            double centerx = falcon_sensor_object->GetBaseObject()->center_ego[0];
            if(abs(mtime-ftime)<0.4 && centerx > 20){
                dst_obj->polygon_utm = sensorpolygon_map_["falcon"];
            }
        }
    }
    // 4.store polygon
    StoreSensorPolygon(measurement, new_polygon);
    dst_obj->polygon = measurement->GetBaseObject()->polygon;
}
void PbfShapeFusion::StoreSensorPolygon(const SensorObjectConstPtr& measurement, PointCloud<PointD> polygon) {
    if (IsLidar(measurement)) {
        sensorpolygon_map_["lidar"] = polygon;
    } else if (IsFalconLidar(measurement)) {
        sensorpolygon_map_["falcon"] = polygon;
    } else if (IsVidar(measurement)) {
        sensorpolygon_map_["vidar"] = polygon;
    } else if (IsRadar(measurement)) {
        sensorpolygon_map_["radar"] = polygon;
    } else if (IsObu(measurement)) {
        sensorpolygon_map_["obu"] = polygon;
    } else {
        // do nothing
    }
}
void PbfShapeFusion::UpdatePolygon() {
  fusion::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
  Eigen::Vector3d position_diff = dst_obj->position - lastposition_;
  for (size_t i = 0; i < dst_obj->polygon_utm.size(); ++i) {
    dst_obj->polygon_utm[i].x = dst_obj->polygon_utm[i].x + position_diff[0];
    dst_obj->polygon_utm[i].y = dst_obj->polygon_utm[i].y + position_diff[1];
  }
}

void PbfShapeFusion::UpdateCenter(const SensorObjectConstPtr &measurement) {
  fusion::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
  fusion::ObjectConstPtr src_obj = measurement->GetBaseObject();
  double host_yaw = measurement->GetBaseObject()->host_yaw;
  double cos_host = std::cos(host_yaw);
  double sin_host = std::sin(host_yaw);
  // Modify by jiangnan : coordinate transformation from UTM down ego
  double center_x = track_ref_->GetFusedObject()->GetBaseObject()->center.x();
  double center_y = track_ref_->GetFusedObject()->GetBaseObject()->center.y();
  dst_obj->center_ego[0] = center_x * cos_host + center_y * sin_host;
  dst_obj->center_ego[1] = -center_x * sin_host + center_y * cos_host;
  dst_obj->center_ego[2] = src_obj->center_ego.z();
}

// FUSION_REGISTER_SHAPEFUSION(PbfShapeFusion)

}  // namespace fusion
}  // namespace perception
