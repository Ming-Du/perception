
#include "pbf_gatekeeper.h"

#include "base/base_init_options.h"
#include "fusion/base/utils.h"
#include "base/configmanager.h"
#include "base/object_types.h"
#include "common/file.h"
#include "perception/lib/config_manager/config_manager.h"
#include "common/proto/pbf_gatekeeper_config.pb.h"

namespace perception {
namespace fusion {

using perception::fusion::GetAbsolutePath;

PbfGatekeeper::PbfGatekeeper() {}

PbfGatekeeper::~PbfGatekeeper() {}

bool PbfGatekeeper::Init() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfGatekeeper", &options)) {
    return false;
  }

  std::string woork_root_config =
      GetAbsolutePath(lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  PbfGatekeeperConfig params;

  if (!perception::fusion::GetProtoFromFile(config, &params)) {
    ROS_ERROR_STREAM("Init: Read config failed: " << config);
    return false;
  }
  params_.publish_if_has_lidar = params.publish_if_has_lidar();
  params_.publish_if_has_radar = params.publish_if_has_radar();
  params_.publish_if_has_camera = params.publish_if_has_camera();
  params_.use_camera_3d = params.use_camera_3d();
  params_.min_radar_confident_distance = params.min_radar_confident_distance();
  params_.max_radar_confident_angle = params.max_radar_confident_angle();
  params_.min_camera_publish_distance = params.min_camera_publish_distance();
  params_.invisible_period_threshold = params.invisible_period_threshold();
  params_.existence_threshold = params.existence_threshold();
  params_.radar_existence_threshold = params.radar_existence_threshold();
  params_.toic_threshold = params.toic_threshold();
  params_.use_track_time_pub_strategy = params.use_track_time_pub_strategy();
  params_.pub_track_time_thresh = params.pub_track_time_thresh();
  // Modify(@liuxinyu): obu_test
  params_.publish_if_has_obu = params.publish_if_has_obu();
  params_.min_obu_publish_distance = params.min_obu_publish_distance();
  ConfigManager::Instance()->get_value("vehicle", &vehicle_);

  bool ret = ConfigManager::Instance()->get_value("frustum_front60",&params_.frustum_front60);
  if(!ret){
    params_.frustum_front60.emplace_back(5);
    params_.frustum_front60.emplace_back(0);
    params_.frustum_front60.emplace_back(40);
    params_.frustum_front60.emplace_back(20);
    params_.frustum_front60.emplace_back(40);
    params_.frustum_front60.emplace_back(-18);
  }
  return true;
}

std::string PbfGatekeeper::Name() const {
  return "PbfGatekeeper";
}

bool PbfGatekeeper::AbleToPublish(const TrackPtr& track) {
  bool invisible_in_lidar = !(track->IsLidarVisible());
  bool invisible_in_radar = !(track->IsRadarVisible());
  bool invisible_in_camera = !(track->IsCameraVisible());
  bool invisible_in_falcon_lidar = !(track->IsFalconLidarVisible());
  bool invisible_in_obu = !(track->IsObuVisible());
  bool invisible_in_vidar =  !(track->IsVidarVisible());

  if (invisible_in_lidar && invisible_in_radar && invisible_in_falcon_lidar &&
      (!params_.use_camera_3d || invisible_in_camera) && invisible_in_obu && invisible_in_vidar) {
    auto sensor_obj = track->GetFusedObject();
    if (sensor_obj != nullptr &&
        sensor_obj->GetBaseObject()->sub_type != fusion::ObjectSubType::TRIANGLEROADBLOCK) {
      ROS_DEBUG("PbfGatekeeper::AbleToPublish return is false1!");
      return false;
    }
  }
  time_t rawtime = static_cast<time_t>(track->GetFusedObject()->GetTimestamp());
  // use thread-safe localtime_r instead of localtime
  struct tm timeinfo;
  localtime_r(&rawtime, &timeinfo);
  bool is_night = (timeinfo.tm_hour >= 23);
  if (!LidarAbleToPublish(track) && !RadarAbleToPublish(track, is_night) &&
      !CameraAbleToPublish(track, is_night) && !ObuAbleToPublish(track, is_night) &&
      !VidarAbleToPublish(track) && !FalconLidarAbleToPublish(track)) {
    ROS_DEBUG("PbfGatekeeper::AbleToPublish return is false2!");
    return false;
  }

  if (params_.use_track_time_pub_strategy &&
      track->GetTrackedTimes() <= static_cast<size_t>(params_.pub_track_time_thresh)) {
    ROS_DEBUG("PbfGatekeeper::AbleToPublish return is false3!");
    return false;
  }
  return !CheckNoise(track);
}

bool PbfGatekeeper::CheckNoise(const TrackPtr &track) {
  //判断noise的数据是否匹配了camera
  bool is_noise = false;
  if (vehicle_ == (int)perception::fusion::VEHICLE::BUS) {
    SensorObjectConstPtr lidar_ptr = track->GetLatestLidarObject();
    //判断点是否在相机fron60的可视范围内
    Eigen::Vector3d pt = track->GetFusedObject()->GetBaseObject()->center_ego;
    perception::fusion::PointCloud<perception::fusion::PointD> ptslist;
    perception::fusion::PointD origin,ptleft,ptright;
    origin.x = params_.frustum_front60[0];
    origin.y = params_.frustum_front60[1];
    ptleft.x = params_.frustum_front60[2];
    ptleft.y = params_.frustum_front60[3];
    ptright.x = params_.frustum_front60[4];
    ptright.y = params_.frustum_front60[5];
    ptslist.push_back(origin);
    ptslist.push_back(ptleft);
    ptslist.push_back(ptright);
    int ret = perception::fusion::pointInRegion(pt.x(),pt.y(),ptslist);
    if(lidar_ptr != nullptr && ret){
      if(lidar_ptr->GetBaseObject()->noise_state == (int)NoiseState::NOISE_NOISE||
      lidar_ptr->GetBaseObject()->noise_state == (int)NoiseState::NOISE_SUSPECTED){
          if(track->GetMatchTimes() < 1){
          is_noise = true;
        }
      }
    }
  }
  return is_noise;
}

bool PbfGatekeeper::LidarAbleToPublish(const TrackPtr& track) {
  bool visible_in_lidar = track->IsLidarVisible();
  if (params_.publish_if_has_lidar && visible_in_lidar) {
    return true;
  }
  return false;
}

bool PbfGatekeeper::FalconLidarAbleToPublish(const TrackPtr& track) {
  bool visible_in_falcon_lidar = track->IsFalconLidarVisible();
  if (params_.publish_if_has_falcon_lidar && visible_in_falcon_lidar) {
    return true;
  }
  return false;
}

bool PbfGatekeeper::VidarAbleToPublish(const TrackPtr& track) {
  bool visible_in_vidar = track->IsVidarVisible();
  if (params_.use_camera_3d && visible_in_vidar) {
    return true;
  }
  return false;
}

bool PbfGatekeeper::RadarAbleToPublish(const TrackPtr& track, bool is_night) {
  bool visible_in_radar = track->IsRadarVisible();
  SensorObjectConstPtr radar_object = track->GetLatestRadarObject();
  if (params_.publish_if_has_radar && visible_in_radar && radar_object != nullptr) {
    base::SensorManager* sensor_manager = base::SensorManager::Instance();
    ROS_DEBUG("\033[31m RadarAbleToPublish: is_rear: %d \033[0m\n",
              radar_object->GetBaseObject()->radar_supplement.is_rear);
    // if (sensor_manager->IsRadar(radar_object->GetSensorId())) {
    if (!radar_object->GetBaseObject()->radar_supplement.is_rear) {
      // TODO(henjiahao): enable radar front
      return false;
      // if (radar_object->GetBaseObject()->radar_supplement.range >
      //         params_.min_radar_confident_distance &&
      //     radar_object->GetBaseObject()->radar_supplement.angle <
      //         params_.max_radar_confident_angle) {
      //   double heading_v =
      //       std::abs(track->GetFusedObject()->GetBaseObject()->velocity.dot(
      //           track->GetFusedObject()->GetBaseObject()->direction));
      //   double toic_p = track->GetToicProb();
      //   auto set_velocity_to_zero = [heading_v, track]() {
      //     if (heading_v < 0.3) {
      //       track->GetFusedObject()->GetBaseObject()->velocity.setZero();
      //     }
      //   };
      //   if (!is_night) {
      //     if (toic_p > params_.toic_threshold) {
      //       set_velocity_to_zero();
      //       return true;
      //     }
      //   } else {
      //     // the velocity buffer is [-3, +3] m/s
      //     double v_ct = 4.0;
      //     double v_slope = 1.0;
      //     auto heading_v_decision = [](double x, double c, double k) {
      //       x = x - c;
      //       return 0.5 + 0.5 * x * k / std::sqrt(1 + x * x * k * k);
      //     };
      //     auto fuse_two_probabilities = [](double p1, double p2) {
      //       double p = (p1 * p2) / (2 * p1 * p2 + 1 - p1 - p2);
      //       p = std::min(1.0 - std::numeric_limits<float>::epsilon(), p);
      //       return p;
      //     };

      //     double min_toic_p = 0.2;
      //     toic_p = std::max(min_toic_p, toic_p);
      //     double v_p = heading_v_decision(heading_v, v_ct, v_slope);
      //     double p = fuse_two_probabilities(toic_p, v_p);
      //     if (p > 0.5) {
      //       set_velocity_to_zero();
      //       return true;
      //     }
      //   }
      // }
      // } else if (radar_object->GetSensorId() == "radar_rear") {
    } else if (radar_object->GetBaseObject()->radar_supplement.is_rear) {
      ROS_DEBUG_STREAM("ComputeRadarRadar: radar_rear: min_dis: "
                       << params_.min_radar_confident_distance
                       << " obj dist: " << radar_object->GetBaseObject()->radar_supplement.range
                       << " speed: " << radar_object->GetBaseObject()->velocity.norm()
                       << " radar_id " << radar_object->GetBaseObject()->id
                       << " track_id: " << track->GetTrackId()
                       << " exist_prob: " << track->GetExistenceProb()
                       << " exist_thresold: " << params_.radar_existence_threshold
                       << " tracking_time: " << track->GetTrackedTimes());
      if (((radar_object->GetBaseObject()->radar_supplement.range >
              params_.min_radar_confident_distance && track->GetTrackedTimes() > 15) &&
          ((radar_object->GetBaseObject()->velocity.norm() > 3.0))) || (track->GetIsPublish())) {
        track->SetIsPublish();
        return true;
      }
    }
  }
  return false;
}

bool PbfGatekeeper::CameraAbleToPublish(const TrackPtr& track, bool is_night) {
  bool visible_in_camera = track->IsCameraVisible();
  SensorId2ObjectMap& camera_objects = track->GetCameraObjects();
  // auto iter = camera_objects.find("front_6mm");
  // auto iter_narrow = camera_objects.find("front_12mm");
  // iter = iter != camera_objects.end() ? iter : iter_narrow;  //org

  // Modify-guoxiaoxiao
  base::SensorManager* sensor_manager = base::SensorManager::Instance();
  base::SensorInfo camera_sensor_info_;
  std::map<base::SensorType, SensorObjectPtr> SensorType2ObjectMap;
  for (auto it = camera_objects.begin(); it != camera_objects.end(); ++it) {
    auto sensor_name = it->first;
    if (!sensor_manager->GetSensorInfo(sensor_name, &camera_sensor_info_)) {
      ROS_ERROR_STREAM("CameraAbleToPublish: camera get sensor info failure : " << sensor_name);
      continue;
    } else {
      SensorType2ObjectMap.insert(make_pair(camera_sensor_info_.type(), it->second));
    }
  }
  auto iter = SensorType2ObjectMap.find(base::SensorType::SENSING_60);
  auto iter_narrow = SensorType2ObjectMap.find(base::SensorType::SENSING_120);
  iter = iter != SensorType2ObjectMap.end() ? iter : iter_narrow;
  // if (params_.publish_if_has_camera && visible_in_camera &&
  //     iter != camera_objects.end() && params_.use_camera_3d && !is_night) {  //org
  if (params_.publish_if_has_camera && visible_in_camera && iter != SensorType2ObjectMap.end() &&
      params_.use_camera_3d && !is_night) {
    SensorObjectConstPtr camera_object = iter->second;
    double range = camera_object->GetBaseObject()->camera_supplement.local_center.norm();
    // If sub_type of object is traffic cone publish it regardless of range
    if ((camera_object->GetBaseObject()->sub_type == fusion::ObjectSubType::TRIANGLEROADBLOCK) ||
        (range >= params_.min_camera_publish_distance ||
         ((camera_object->GetBaseObject()->type == fusion::ObjectType::UNKNOWN) &&
          (range >= params_.min_camera_publish_distance)))) {
      double exist_prob = track->GetExistenceProb();
      if (exist_prob > params_.existence_threshold) {
        static int cnt_cam = 1;
        ROS_DEBUG_STREAM("CameraAbleToPublish: publish camera only object : cnt =  " << cnt_cam);
        cnt_cam++;
        return true;
      }
    }
  }
  return false;
}

bool PbfGatekeeper::ObuAbleToPublish(const TrackPtr& track, bool is_night) {
  // TODO(@liuxinyu): 複用Lidar
  bool visible_in_obu = track->IsObuVisible();
  if (params_.publish_if_has_obu && visible_in_obu) {
    return true;
  }
  return false;
}

}  // namespace fusion
}  // namespace perception
