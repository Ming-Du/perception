// headers in STL
#include "fpnet_processing_unit.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <stdio.h>

// headers in ROS
#include <omp.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <unordered_set>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// headers in local files
#include <visualization_msgs/MarkerArray.h>

#include "common/include/pb_utils.h"
#include "mogo_track/gpu_track.h"
#include "mogo_track/mogo_track_logger.h"
#include "perception/base/object.h"
#include "perception/base/proto/perception_component.pb.h"
#include "perception/base/timer.h"
#include "perception/fusion_mid/common/define.h"
#include "perception/fusion_mid/common/display.h"
#include "perception/fusion_mid/common/object_type_convert.h"
#include "perception/fusion_mid/common/utils.h"
#include "mogo_track/mogo_track_logger.h"
#include "mogo_track/gpu_track.h"


namespace perception {
namespace mid_fusion {
static const double PI(3.141592653589793238462);
static const int ISUSECLOUDBASE(0);
bool perception::mid_fusion::TicToc::enable_timer_ = false;
constexpr int kMaxNumberOfPoint = 230000;
constexpr int kMinNumberOfPoint = 30000;
// vegetation infer const params
constexpr int kVegetationIndex = 12;
constexpr int kBackgroundIndex = 0;
constexpr double kVegetationExistProb = 0.50;
constexpr double kSmallObjMaxExistProb = 0.9;
constexpr int kHighVelocityCnt = 3;
// noise 
constexpr int kNoiseIndex = 10;

// road status params:
// road_status = 0 represent that the lidar obstacle on the road
// road status > 0 represent that the lidar obstacle is roadside or fenceside
constexpr int kOnRoadStatus = 0;
const std::unordered_set<perception::ObjectType> kCameraSmallObjectTypes = {
    perception::ObjectType::TYPE_PEDESTRIAN, perception::ObjectType::TYPE_TRIANGLEROADBLOCK,
    perception::ObjectType::TYPE_BICYCLE, perception::ObjectType::TYPE_MOTOR,
    perception::ObjectType::TYPE_WARNINGTRIANGLE};
const std::unordered_set<perception::ObjectType> kTrafficConeObjectTypes = {
    perception::ObjectType::TYPE_TRIANGLEROADBLOCK, perception::ObjectType::TYPE_WARNINGTRIANGLE};
const std::vector<float> kPersonSize = {0.5, 0.5, 1.7};
const std::vector<float> kBicycleSize = {1.5, 0.5, 1.7};
const std::vector<float> kConeSize = {0.2, 0.2, 0.5};
std::string getNowYmdhms() {
  std::chrono::time_point<std::chrono::system_clock> nowtime;
  nowtime = std::chrono::system_clock::now();
  std::time_t nowtime_t = std::chrono::system_clock::to_time_t(nowtime);
  char now[64];
  struct tm* ttime;
  // time(&tt);
  ttime = localtime(&nowtime_t);
  std::strftime(now, 64, "%Y-%m-%d %H:%M:%S", ttime);
  std::string str(now);
  return str;
}

FpnetProcessingUnit::FpnetProcessingUnit()
    : nh_(ros::NodeHandle()),
      private_nh_(ros::NodeHandle("~")),
      imageTransport_(ros::NodeHandle()) {
  cluster_interface_ptr_.reset(new perception::mid_fusion::ComponentCluster());
  dev_rotate_iou_ptr_.reset(new perception::mid_common::DevRotateIou());
  // ros related param
  private_nh_.param<bool>("baselink_support", baselink_support_, true);
  frcnn_obj_ptr_.reset(new frcnn::FrustumRCNN());
  tracker_.reset(new MidFusionTracker);
  trigger_manager_ptr_.reset(new TriggerManager);
  Init();
}

bool FpnetProcessingUnit::Init() {
  // read mid_fusion config 
  MidFusionInitOptions options;
  ROS_DEBUG_STREAM("Init: read config from: " << FLAGS_fusion_mid_conf_path);
  if (!::common::GetProtoFromFile(FLAGS_fusion_mid_conf_path, options)) {
    ROS_ERROR("Init: failed to load config file.");
    return false;
  }
  ROS_DEBUG_STREAM("Init: rec_topic_name_lidar_cloud: " << options.rec_topic_name_lidar_cloud());
  ROS_DEBUG_STREAM(
      "Init: rec_topic_name_lidar_obstacle: " << options.rec_topic_name_lidar_obstacle());
  ROS_DEBUG_STREAM("Init: rec_topic_name_lidar_obstacle: " << options.vehicle_type());
  // load lidar topic
  rec_topic_name_lidar_cloud_ = options.rec_topic_name_lidar_cloud();
  rec_topic_name_lidar_obstacle_ = options.rec_topic_name_lidar_obstacle();
  if (options.has_vehicle_type()) {
    vehicle_type_ = options.vehicle_type();
  }
  perception::base::SensorManager* sensor_manager = perception::base::SensorManager::Instance();
  /* load launch file conf */
  private_nh_.getParam("fusion_mid_config_path", config_file_);
  private_nh_.getParam("fusion_mid_models_path", models_file_);
  ROS_INFO_STREAM("Init: mid_config: " << config_file_);
  ROS_INFO_STREAM("Init: models: " << models_file_);

  config_file_str_ = config_file_ + "vehicle_type_conf/" + vehicle_type_ + "_mid_fusion_config.txt";
  ROS_INFO_STREAM("Init: mid_fusion_config:  " << config_file_str_);
  //  read Configuration items
  try {
    config_parser_ptr_.reset(
        new perception::mid_common::ConfigParser(config_file_str_));  // load conf file
  } catch (const char* e) {
    ROS_WARN_STREAM("Init: mid_fusion_config error!!");
  }
  yaml_file_str_ = config_file_ + "frustum_rcnn_conf/frcnn_cfg.yaml";
  ROS_INFO_STREAM("Init: frcnn_cfg:  " << yaml_file_str_);
  shuffle_index_file_str_ = config_file_ + "frustum_rcnn_conf/shuffle_index.txt";
  ROS_INFO_STREAM("Init: shuffle_index:  " << shuffle_index_file_str_);
  YAML::Node frcnn_cfg = YAML::LoadFile(yaml_file_str_);
  ROS_DEBUG_STREAM("Init: frcnn obj ptr init~~!!");
  const string& model_engine_name = config_parser_ptr_->getString("fpointnet_engine_name");
  frcnn_obj_ptr_->Init(frcnn_cfg["preprocess"], raw_camera_num_, model_engine_name, models_file_,
                       shuffle_index_file_str_);
  ROS_INFO_STREAM("Init: model init complete~!");
  // initialization camera external parameter and internal parameter  
  cam_extr_.resize(raw_camera_num_ * 4 * 4, 0);
  cam_intr_.resize(raw_camera_num_ * 3 * 3, 0);
  cam_hw_.resize(raw_camera_num_ * 2, 0);
  cam_extr_inv_.resize(raw_camera_num_ * 4 * 4, 0);
  cam_intr_inv_.resize(raw_camera_num_ * 3 * 3, 0);
  // load vehicle conf param
  LoadVehConfParam();
   
  lidarPoints_ptr_.reset(new std::vector<FrustumLidarPoint>());

  if (is_small_object_infer_) {
    tracker_->Init();
  }
  /* Init */
  // TODO:封装一波
  // load intr and extr params for each sensor
  sensorInfoMap_ = sensor_manager->GetSensorInfoMap();
  std::unordered_map<std::string, std::string> sensor_name_to_topic_map;
  int cam_id = 0;
  for (auto p : sensorInfoMap_) {
    Eigen::Matrix4d trans_matrix;
    trans_matrix.setIdentity();
    if (sensor_manager->IsCamera(p.second.type())) {
      if (p.second.has_intrinsic() && p.second.has_extrinsic()) {
        ExtraCamInterExterParam(p.first, p.second, trans_matrix);
        ROS_DEBUG_STREAM("Init: cam_id " << cam_id << "  " << p.second.topic());
        sensor_name_to_topic_map[p.first] = inextritopic_totopic_map_[p.second.topic()];
        id_map_[inextritopic_totopic_map_[p.second.topic()]] = cam_id;
        sensorMatrixMap_.insert(std::make_pair(p.first, trans_matrix));
        cam_id++;
      }
    } else {
      Eigen::Affine3d other_TtoWorld_base;
      perception::base::SetSensorExtrinsics(p.second.extrinsic(), other_TtoWorld_base);
      trans_matrix = other_TtoWorld_base.matrix().inverse();
      sensorMatrixMap_.insert(std::make_pair(p.first, trans_matrix));
    }
    ROS_DEBUG_STREAM(p.first);
    ROS_DEBUG_STREAM(trans_matrix);
  }
  // add value for extr and intr
  for (const auto autocamera : extrinsics_camera_map_) {
    int cam_id_extr_index = id_map_[sensor_name_to_topic_map[autocamera.first]];
    Eigen::Matrix4d cam_extr_temp;
    cam_extr_temp.setIdentity();
    cam_extr_temp = autocamera.second;
    Eigen::Matrix4d cam_extr_inv_temp;
    cam_extr_inv_temp.setIdentity();
    cam_extr_inv_temp.block(0, 0, 3, 3) = cam_extr_temp.block(0, 0, 3, 3).transpose();
    cam_extr_inv_temp.block(0, 3, 3, 1) =
        -cam_extr_inv_temp.block(0, 0, 3, 3) * cam_extr_temp.block(0, 3, 3, 1);
    for (int i = 0; i < 4 * 4; i++) {
      cam_extr_[cam_id_extr_index * 16 + i] = cam_extr_temp(i / 4, i % 4);
      cam_extr_inv_[cam_id_extr_index * 16 + i] = cam_extr_inv_temp(i / 4, i % 4);
    }
  }
  for (const auto autocamera : intrinsics_camera_map_) {
    int cam_id_intr_index = id_map_[sensor_name_to_topic_map[autocamera.first]];
    Eigen::Matrix4d cam_intr_temp;
    cam_intr_temp = autocamera.second;
    Eigen::Matrix3d cam_intr_inv_temp;
    cam_intr_inv_temp = cam_intr_temp.block(0, 0, 3, 3).inverse();
    for (int i = 0; i < 3 * 3; i++) {
      cam_intr_[cam_id_intr_index * 9 + i] = cam_intr_temp.block(0, 0, 3, 3)(i / 3, i % 3);
      cam_intr_inv_[cam_id_intr_index * 9 + i] = cam_intr_inv_temp(i / 3, i % 3);
    }
  }
  return true;
}

void FpnetProcessingUnit::Checkdata() {
  ROS_INFO_STREAM("check data:  start***********************************************");
  ROS_INFO_STREAM("check data:  end#################################################");
}

void FpnetProcessingUnit::Run() {
  static pcl::StopWatch Run_timer;
  ReadData();
  while (HasData()) {
    if (Run_timer.getTime() > 300) {
      ROS_ERROR_STREAM("Run: " << getNowYmdhms() << "  lidarobject_delay!!! delay_time="
                               << Run_timer.getTime() << "ms");
    }
    Run_timer.reset();
    perception::mid_fusion::TicToc stimer("Run");

    {
      perception::mid_fusion::TicToc timer("Run/ExtractValidData");
      ExtractValidData();
    }
    if (is_pub_cloudpointsImg_) {
      PointsProcess();
    }
    if (is_pub_camera_frustum_) {
      DebugVisualProcess();
    }
    if (is_fpnet_) {
      perception::mid_fusion::TicToc timer("Run/FpointnetProcess");
      FpointnetProcess();
    }
  }
  /* 控制故障上报发送 */
  double current_sys_time = ros::Time::now().toSec();
  static double latest_time = 0.0;
  if (current_sys_time - latest_time > 1.0) {
    latest_time = current_sys_time;
    if (output_fault_messages_.infos_size() > 0) {
      ROS_DEBUG_STREAM(output_fault_messages_.DebugString());
      fault_msg_publisher_.publish(fault_manager_report_, output_fault_messages_);
      output_fault_messages_.clear_infos();
    }
  }
}

void FpnetProcessingUnit::ExtraCamInterExterParam(std::string sensor_name,
                                           const perception::base::SensorInfo sensor_info,
                                           Eigen::Matrix4d& trans_matrix) {
  /*
  相机深度Z_c乘以像素坐标[u v 1] = 相机内参K*相机外参RT*世界坐标P_w
  */
  Eigen::Matrix4d base_T_camera;
  Eigen::Matrix4d T_K;
  Eigen::VectorXd Distcoeff(5);
  base_T_camera.setIdentity();
  T_K.setIdentity();

  Eigen::Affine3d camera_T_base;
  perception::base::SetSensorExtrinsics(sensor_info.extrinsic(), camera_T_base);
  base_T_camera = camera_T_base.matrix().inverse();
  extrinsics_camera_map_.insert(std::make_pair(sensor_name, base_T_camera));

  T_K(0, 0) = sensor_info.intrinsic().matrix(0);
  T_K(0, 1) = sensor_info.intrinsic().matrix(1);
  T_K(0, 2) = sensor_info.intrinsic().matrix(2);
  T_K(0, 3) = 0.0;
  T_K(1, 0) = sensor_info.intrinsic().matrix(3);
  T_K(1, 1) = sensor_info.intrinsic().matrix(4);
  T_K(1, 2) = sensor_info.intrinsic().matrix(5);
  T_K(1, 3) = 0.0;
  T_K(2, 0) = sensor_info.intrinsic().matrix(6);
  T_K(2, 1) = sensor_info.intrinsic().matrix(7);
  T_K(2, 2) = sensor_info.intrinsic().matrix(8);
  T_K.row(3) << 0.0, 0.0, 0.0, 1.0;  // 构成 4*4
  intrinsics_camera_map_.insert(std::make_pair(sensor_name, T_K));

  trans_matrix = T_K * base_T_camera;

  Distcoeff(0) = sensor_info.distcoeff().distort_matrix(0);
  Distcoeff(1) = sensor_info.distcoeff().distort_matrix(1);
  Distcoeff(2) = sensor_info.distcoeff().distort_matrix(2);
  Distcoeff(3) = sensor_info.distcoeff().distort_matrix(3);
  Distcoeff(4) = sensor_info.distcoeff().distort_matrix(4);
  distcoeff_camera_map_.insert(std::make_pair(sensor_name, Distcoeff));
}

void FpnetProcessingUnit::CreateROSPubSub() {
  ros::NodeHandle pn("~");
  // todo subscriber
  cloud_sub_ptr_ =
      std::make_shared<mid_fusion::CloudSubscriber>(nh_, rec_topic_name_lidar_cloud_, 1);
  radar_sub_ptr_ =
      std::make_shared<mid_fusion::RadarSubscriber>(nh_, rec_topic_name_radar_obstacle_, 2);
  lidarobjects_sub_ptr_ =
      std::make_shared<mid_fusion::LidarobjectSubscriber>(nh_, rec_topic_name_lidar_obstacle_, 1);
  if (is_pub_cloudpointsImg_) {
    sub_camera_detection_front60_ =
        nh_.subscribe<sensor_msgs::Image>(rec_topic_name_front_camera_detection_60_, 1,
                                          &FpnetProcessingUnit::Front60CameraDetectionCallback, this);
  }

  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front30_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_front_camera_obstacle_front30_, 1);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front60_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_front_camera_obstacle_front60_, 1);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front120_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_front_camera_obstacle_front120_, 1);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_left120_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_front_camera_obstacle_left120_, 1);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_right120_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_front_camera_obstacle_right120_, 1);
  camera2d_sub_ptr_map_[rec_topic_name_back_camera_obstacle_back120_] =
      std::make_shared<mid_fusion::Camera2dSubscriber>(
          nh_, rec_topic_name_back_camera_obstacle_back120_, 1);

  localization_subscriber_ =
      pn.subscribe(rec_topic_name_localization_, 1, &FpnetProcessingUnit::LocalizationCallback, this);

  hadmap_lane_subscriber = pn.subscribe("/hadmap_engine/lanes_msg", 1, &FpnetProcessingUnit::HadmapLaneCallback, this);
  // publisher
  pub_lidar_objects_ = nh_.advertise<std_msgs::String>("/perception/fusion_mid/lidar_obstacle", 10);
  pub_camera_objects_ =
      nh_.advertise<std_msgs::String>("/perception/fusion_mid/camera_obstacle", 10);
  fault_manager_report_ = nh_.advertise<autopilot_msgs::BinaryData>("/system/fm/fault_report", 10);
  if (is_pub_cloudpointsImg_) {
    pub_cloudpointsImg_ = imageTransport_.advertise("/perception/fusion_mid/cloudpoints_image", 10);
  }
  pub_trigger_ = nh_.advertise<autopilot_msgs::BinaryData>("/perception/fusion_mid/trigger_info", 1, true);
  if (is_addfpnet_visualization_pub_) {
    visualization_lidar_obstacle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/perception/fusion_mid/visualization_mid_obstacle", 10);
    visualization_lidar_obstacle_text_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/perception/fusion_mid/visualization_mid_obstacle_text", 10);
    visualization_lidar_obstacle_polygon_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/perception/fusion_mid/visualization_mid_obstacle_polygon", 10);
  }

  visualization_camera_frustum_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/perception/fusion_mid/visualization_camera_frustum", 10);
  visualization_2D_frustum_points_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/perception/fusion_mid/visualization_2D_frustum_points", 10);
}

bool FpnetProcessingUnit::ExtractValidData() {
  fusion_rawdata_.clearData();
  for (auto temp : map_current_camera2d_data_) {
    temp.second.Clear();
  }
  map_current_camera2d_data_.clear();
#if ISUSECLOUDBASE
  fusion_rawdata_.current_cloud_data_ = cloud_data_buff_.front();
  double curent_base_time = fusion_rawdata_.current_cloud_data_.time;
#else
  fusion_rawdata_.current_lidarobjects_data_ = lidarobjects_data_buff_.front();
  double curent_base_time =
      fusion_rawdata_.current_lidarobjects_data_.header().stamp().sec() +
      fusion_rawdata_.current_lidarobjects_data_.header().stamp().nsec() * 1e-9;
#endif
  // get camera 2D Obstacles
  for (auto camera_auto = camera2d_data_buff_map_.begin();
       camera_auto != camera2d_data_buff_map_.end(); camera_auto++) {
    double min_cameradiff_time = max_time_tr_;
    int image_obj_index = -1;
    for (int i = 0; i < camera_auto->second.size(); i++) {
      double camera2d_curent_time = camera_auto->second[i].header().stamp().sec() +
                                    camera_auto->second[i].header().stamp().nsec() * 1e-9;
      double diff_camera_time = std::fabs(curent_base_time - camera2d_curent_time);
      // ROS_DEBUG_STREAM("ExtractValidData: diff_camera_time
      // :"<<diff_camera_time<<"  "<<camera_auto->first<<"_curent_time:
      // "<<camera2d_curent_time);
      if (diff_camera_time < min_cameradiff_time) {
        min_cameradiff_time = diff_camera_time;
        image_obj_index = i;
      }
    }
    //Modify@liuxinyu: Camera & Lidar Unsynchronized FM
    if (min_cameradiff_time > min_lidar_camera_sync_time_thr_) {
      FaultManager("Perception_MidFusion_CameraLidarUnsynchronized");
    }
    if (image_obj_index != -1) {
      perception::VisualObjects camera_dec_temp = camera_auto->second[image_obj_index];
      map_current_camera2d_data_.insert(std::make_pair(camera_auto->first, camera_dec_temp));
      double matched_image_timestamp =
          camera_dec_temp.header().stamp().sec() + camera_dec_temp.header().stamp().nsec() * 1e-9;
      double final_time_diff = std::fabs(curent_base_time - matched_image_timestamp);
      ROS_INFO_STREAM("ExtractValidData: Matched Image Time: " << std::setprecision(18)
                       << matched_image_timestamp << " Lidar_obs_time: " << curent_base_time
                       << " Final Time Diff: " << final_time_diff);
    } else if (camera_auto->second.size() != 0) {
      ROS_ERROR_STREAM("ExtractValidData: " << getNowYmdhms() << " " << camera_auto->first
                                            << "   min_camera_lidar_difftime > "
                                            << min_cameradiff_time << " s");
    }
  }
  // get radar Obstacles
  double min_radarobjectdiff_time = max_time_tr_;
  int radar_obj_index = -1;
  for (int i = 0; i < radar_data_buff_.size(); i++) {
    double radar_obj_curent_time = radar_data_buff_[i].header().stamp().sec() +
                                   radar_data_buff_[i].header().stamp().nsec() * 1e-9;
    double diff_radar_time = std::fabs(curent_base_time - radar_obj_curent_time);
    // ROS_DEBUG_STREAM("ExtractValidData: diff_radar_time
    // :"<<diff_radar_time<<"   radar_obj_curent_time:
    // "<<radar_obj_curent_time);
    if (diff_radar_time < min_radarobjectdiff_time) {
      min_radarobjectdiff_time = diff_radar_time;
      radar_obj_index = i;
    }
  }
  if (radar_obj_index != -1) {
    fusion_rawdata_.current_radar_data_ = radar_data_buff_[radar_obj_index];
  } else if (radar_data_buff_.size() != 0) {
    ROS_ERROR_STREAM("ExtractValidData: " << getNowYmdhms()
                                          << "radarobject  min_radar_lidar_difftime > "
                                          << min_radarobjectdiff_time << " s");
  }
  // get Lidar Obstacles
#if ISUSECLOUDBASE
  double min_lidarobjectdiff_time = max_time_tr_;
  int lidarobject_index = -1;
  for (int i = 0; i < lidarobjects_data_buff_.size(); i++) {
    double lidar_obj_curent_time = lidarobjects_data_buff_[i].header().stamp().sec() +
                                   lidarobjects_data_buff_[i].header().stamp().nsec() * 1e-9;
    double diff_lidarobjects_time = std::fabs(curent_base_time - lidar_obj_curent_time);
    // ROS_DEBUG_STREAM("ExtractValidData: diff_lidarobjects_time
    // :"<<diff_lidarobjects_time<<"   lidar_obj_curent_time:
    // "<<lidar_obj_curent_time);
    if (diff_lidarobjects_time <= min_lidarobjectdiff_time) {
      min_lidarobjectdiff_time = diff_lidarobjects_time;
      lidarobject_index = i;
    }
  }
  if (lidarobject_index != -1) {
    fusion_rawdata_.current_lidarobjects_data_ = lidarobjects_data_buff_[lidarobject_index];
  }
  cloud_data_buff_.pop_front();
#else
  double min_lidardiff_time = max_time_tr_;
  int lidar_obj_index = -1;
  for (int i = 0; i < cloud_data_buff_.size(); i++) {
    double lidar_curent_time = cloud_data_buff_[i].time;
    double diff_lidar_time = std::fabs(curent_base_time - lidar_curent_time);
    ROS_INFO_STREAM("ExtractValidData: "
                    << "diff_lidar_time  :" << diff_lidar_time
                    << "   lidar_curent_time: " << lidar_curent_time);
    if (diff_lidar_time <= min_lidardiff_time) {
      min_lidardiff_time = diff_lidar_time;
      lidar_obj_index = i;
    }
  }
  if (lidar_obj_index != -1) {
    fusion_rawdata_.current_cloud_data_ = cloud_data_buff_[lidar_obj_index];
  } else if (cloud_data_buff_.size() != 0) {
    ROS_ERROR_STREAM("ExtractValidData: " << getNowYmdhms()
                                          << " pointcloud   min_cloud_lidar_difftime > "
                                          << min_lidardiff_time << " s");
  }
  lidarobjects_data_buff_.pop_front();
#endif
  // ROS_INFO_STREAM("ExtractValidData: " << "lidarobject_index
  // "<<lidarobject_index<<" radarobject_index "<<radarobject_index<<"
  // cloudtime: "<<fusion_rawdata_.current_cloud_data_.time);
}

bool FpnetProcessingUnit::HasData() {
#if ISUSECLOUDBASE
  if (cloud_data_buff_.size() == 0) {
    ROS_WARN_THROTTLE(2, "HasData: cloud_data_buff is empty.");
    FaultManager("Perception_MidFusion_LackLidarPointCloud");
    return false;
  }
#else
  if (lidarobjects_data_buff_.size() == 0) {
    ROS_WARN_THROTTLE(2, "HasData: lidarobjects_data_buff is empty.");
    // FaultManager("Perception_MidFusion_LackLidar3dObject");
    return false;
  }
#endif
  return true;
}
bool FpnetProcessingUnit::ReadData() {
  lidarobjects_sub_ptr_->ParseData(lidarobjects_data_buff_);
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  radar_sub_ptr_->ParseData(radar_data_buff_);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front60_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_front_camera_obstacle_front60_]);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front30_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_front_camera_obstacle_front30_]);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_front120_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_front_camera_obstacle_front120_]);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_left120_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_front_camera_obstacle_left120_]);
  camera2d_sub_ptr_map_[rec_topic_name_front_camera_obstacle_right120_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_front_camera_obstacle_right120_]);
  camera2d_sub_ptr_map_[rec_topic_name_back_camera_obstacle_back120_]->ParseData(
      camera2d_data_buff_map_[rec_topic_name_back_camera_obstacle_back120_]);
  return true;
}

void FpnetProcessingUnit::Front60CameraDetectionCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr cam_image_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!cam_image_ptr->image.empty()) {
      if (detection_image_buffer_map_["/perception/camera/object_detection_front60"].size() >= 10) {
        detection_image_buffer_map_["/perception/camera/object_detection_front60"].front().reset();
        detection_image_buffer_map_["/perception/camera/object_detection_front60"].pop_front();
      }

      detection_image_buffer_map_["/perception/camera/object_detection_front60"].push_back(
          cam_image_ptr);
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void FpnetProcessingUnit::FpointnetProcess() {
  PreprocessFpntInput pre_res;
  double pub_timestamp = 0.0;
  {
    perception::mid_fusion::TicToc timer("FpointnetProcess/ExtractPcdAndBox");
    pre_res = ExtractPcdAndBox();
  }
  std::vector<perception::TrackedObject*> candidates_list = GetCandidatesList();
  if (pre_res.empty() && candidates_list.empty()) {
    if (is_addfpnet_visualization_pub_) {
      DisplayLidarobject(fusion_rawdata_.current_lidarobjects_data_);
    }
    std_msgs::String middle_fusion_frame;
    fusion_rawdata_.current_lidarobjects_data_.SerializeToString(&middle_fusion_frame.data);
    pub_lidar_objects_.publish(middle_fusion_frame);
    ROS_DEBUG_STREAM("FpointnetProcess: No small obejct or vegetation or noise");
    return;
  }
  /* model infer */
  {
    perception::mid_fusion::TicToc timer("FpointnetProcess/inference");
    mogo_track::GpuTrackGuard guard("frcnn_object_det", "perception");
    frcnn_obj_ptr_->inference(pre_res.pcd_xyzi, pre_res.box_det2d, cam_extr_, cam_intr_, cam_hw_,
                              cam_extr_inv_, cam_intr_inv_, candidates_list,
                              is_pub_frustum_points_);
  }
  // TODO(liuxinyu): 封装一波
  // refine unknown->vegetation && noise_state
  {
    perception::mid_fusion::TicToc timer("FpointnetProcess/refine_unknown");
    pub_timestamp = fusion_rawdata_.current_lidarobjects_data_.header().stamp().sec() +
                    fusion_rawdata_.current_lidarobjects_data_.header().stamp().nsec() * 1e-9;
    if (is_vegetation_infer_ || is_noise_infer_) {
      std::vector<float>& lidar_object_pred_type = frcnn_obj_ptr_->lidar_object_pred_type_;
      ROS_DEBUG_STREAM(
          "FpointnetProcess: lidar object pred type size: " << lidar_object_pred_type.size() / 2);
      for (int i = 0; i < lidar_object_pred_type.size(); i++) {
        if (i % 2 == 0) {
          int index = i / 2;
          ROS_DEBUG_STREAM("FpointnetProcess:### lidar id is: "
                           << candidates_list[index]->obj().id()
                           << "type is: " << lidar_object_pred_type[i + 1]
                           << " score is: " << lidar_object_pred_type[i]);
          // lidar obstacle pred type == -1 means that there are no lidar points in the lidar
          // obstacle. If it is roadside obstacle(road status > 0), still set it as vegetation.
          perception::Object* mutable_object_ptr = candidates_list[index]->mutable_obj();
          if (int(lidar_object_pred_type[i + 1]) == kVegetationIndex &&
              lidar_object_pred_type[i] > vegetation_threshold_) {
            mutable_object_ptr->set_type(perception::ObjectType::TYPE_VEGETATION);
            mutable_object_ptr->set_exist_confidence(kVegetationExistProb);
            // Set vegetation type obstacle velocity as zero.
            mutable_object_ptr->mutable_velocity()->set_x(0);
            mutable_object_ptr->mutable_velocity()->set_y(0);
            mutable_object_ptr->mutable_velocity()->set_z(0);
            ROS_DEBUG_STREAM(H_DEBUG_R << " Refine Type for " << mutable_object_ptr->id());
          }
          // refine noise state
          if (mutable_object_ptr->noise_state() == perception::NoiseState::NOISE_SUSPECTED &&
              int(lidar_object_pred_type[i + 1]) != kNoiseIndex &&
              lidar_object_pred_type[i] > fixed_noise_minimum_score_thr_) {
            mutable_object_ptr->set_noise_state(perception::NoiseState::NOISE_OBJECT);
            ROS_DEBUG_STREAM(H_DEBUG_B << " Refine Noise State for " << mutable_object_ptr->id()
                                       << " , Current Noise State is "
                                       << mutable_object_ptr->noise_state());
          }
        }
      }
      // TODO : Pub
      // Avoid duplicated publish.
      if (!is_small_object_infer_) {
        // TODO(liuxinyu): 封装一波 UnInferSmallObjPub
        perception::mid_fusion::TicToc timer("FpointnetProcess/UnInferSmallObjPub");
        if (is_addfpnet_visualization_pub_) {
          DisplayLidarobject(fusion_rawdata_.current_lidarobjects_data_);
        }
        std_msgs::String middle_fusion_frame;
        fusion_rawdata_.current_lidarobjects_data_.SerializeToString(&middle_fusion_frame.data);
        ROS_INFO_THROTTLE(1, "FpointnetProcess:DataCheck size: %d time: %lf\n",
                          fusion_rawdata_.current_lidarobjects_data_.objs_size(), pub_timestamp);
        pub_lidar_objects_.publish(middle_fusion_frame);
      }
    }
  }

  // when is_small_object_infer_ is open, deal with this part
  if (is_small_object_infer_) {
    // TODO(liuxinyu): 封装一波 InferSmallObjPub
    perception::mid_fusion::TicToc timer("FpointnetProcess/InferSmallObjPub");
    std::vector<float> pred_box3d_temp = frcnn_obj_ptr_->pred_box3d_;
    ROS_DEBUG_STREAM("FpointnetProcess:pred_box3d_temp size is: " << pred_box3d_temp.size());
    // TODO: PublishFpntObjects 改为PostProcess如何？
    PublishFpntObjects(pred_box3d_temp, pre_res);
    if (is_pub_frustum_points_) {
      visualization_msgs::MarkerArray marker_array;
      mid_visualization_.PointsDisplay(frcnn_obj_ptr_->frustum_2d_points_, pre_res.box_det2d,
                                       pub_timestamp, marker_array);
      if (!marker_array.markers.empty()) {
        visualization_2D_frustum_points_.publish(marker_array);
      }
    }
  }
}

bool FpnetProcessingUnit::IsVegetationCandidate(const perception::TrackedObject& lidar_object) {
  const int lidar_object_id = lidar_object.obj().id();
  ROS_DEBUG_STREAM(H_DEBUG_R << " Lidar object id: " << lidar_object_id);
  if (lidar_object.obj().center().x() >= vegetation_roi_x_max_ ||
      lidar_object.obj().center().x() < vegetation_roi_x_min_ ||
      lidar_object.obj().center().y() >= vegetation_roi_y_max_ ||
      lidar_object.obj().center().y() < vegetation_roi_y_min_) {
    ROS_DEBUG_STREAM(H_DEBUG_R << " ROI filter");
    return false;
  }
  // Skip objects have high velocity.
  if (object_id_to_velocity_cnt_[lidar_object_id] >= kHighVelocityCnt) {
    ROS_DEBUG_STREAM(H_DEBUG_R << " Velocity filter "
                               << object_id_to_velocity_cnt_[lidar_object_id]);
    return false;
  }
  // Only handle TYPE_UNKNOWN type(exclude TYPE_UNKNOWN_DYNAMIC).
  ROS_DEBUG_STREAM(H_DEBUG_B << " Noise status is: "
                            << lidar_object.obj().noise_state()
                            << " id: " << lidar_object.obj().id());
  if ((lidar_object.obj().type() == perception::ObjectType::TYPE_UNKNOWN ||
       (lidar_object.obj().type() >= perception::ObjectType::TYPE_UNKNOWN_SMALL &&
        lidar_object.obj().type() <= perception::ObjectType::TYPE_UNKNOWN_STATIC)) ||
      (lidar_object.obj().type() == perception::ObjectType::TYPE_CAR &&
       lidar_object.obj().noise_state() == perception::NoiseState::NOISE_FLOWERBEDS)) {
    perception::mid_fusion::Point2d candidate_pnt;
    candidate_pnt.x = lidar_object.obj().center().x();
    candidate_pnt.y = lidar_object.obj().center().y();
    // Handle unknown candidates which in lanemarkers.
    if (!lanemarks_.empty() && IsBoundaryLane(lanemarks_.back(), candidate_pnt)) {
      ROS_INFO_STREAM(H_DEBUG_R << " Real vege candidate and road status is: "
                                << lidar_object.obj().road_status()
                                << " id: " << lidar_object.obj().id());
      return true;
    }
  }
  return false;
}

bool FpnetProcessingUnit::IsNoiseCandidate(const perception::TrackedObject& lidar_object) {
  if (lidar_object.obj().center().x() >= noise_roi_x_max_ ||
      lidar_object.obj().center().x() < noise_roi_x_min_ ||
      lidar_object.obj().center().y() >= noise_roi_y_max_ ||
      lidar_object.obj().center().y() < noise_roi_y_min_) {
    return false;
  }
  // (lidar_object.obj().type() == perception::ObjectType::TYPE_UNKNOWN ||
  //      (lidar_object.obj().type() >= perception::ObjectType::TYPE_UNKNOWN_SMALL &&
  //       lidar_object.obj().type() <= perception::ObjectType::TYPE_UNKNOWN_STATIC)) &&
  if (lidar_object.obj().noise_state() == perception::NoiseState::NOISE_SUSPECTED ||
       lidar_object.obj().noise_state() == perception::NoiseState::NOISE_NOISE) {
    ROS_DEBUG_STREAM(H_DEBUG_R << " Lidar object id: " << lidar_object.obj().id());
    return true;
  }
  return false;
}

std::vector<perception::TrackedObject*> FpnetProcessingUnit::GetCandidatesList() {
  const int lidar_object_cnt = fusion_rawdata_.current_lidarobjects_data_.objs_size();
  #if EXTRA_VEG
  double time = fusion_rawdata_.current_lidarobjects_data_.header().stamp().sec() +
                fusion_rawdata_.current_lidarobjects_data_.header().stamp().nsec() * 1e-9;
  stringstream temp_ss;
  std::stringstream ss;
  ss << "//home//mogo//data//liuxinyu//catkin_ws//data//" << std::setprecision(19) << time << ".txt";
  std::string file_name = ss.str();
  std::cout << " ===== " << file_name << std::endl;
  std::ofstream vegetation_anno(file_name,std::ios::app);
  std::string tmp_type = "vegetation";
  int id = 1;
  for (int i = 0; i < lidar_object_cnt; i++) {
    perception::TrackedObject* lidar_object =
    fusion_rawdata_.current_lidarobjects_data_.mutable_objs(i);
    if(IsVegetationCandidate(*lidar_object)) {
      // 大理都是低矮杂草
      // if (lidar_object->obj().center().z() <= 0.6 || lidar_object->obj().size().z() <= 0.5) {
      //   continue;
      // }
      // if (lidar_object->obj().road_status() == 0 && lidar_object->obj().size().z() <= 1.2) {
      //   continue;
      // }
      // 通过ID可控制0kb文件生成
      // if (lidar_object->obj().id()!= 17237 && lidar_object->obj().id()!= 17243 
      //   && lidar_object->obj().id()!= 17263 && lidar_object->obj().id()!= 17260
      //   && lidar_object->obj().id()!= 17286 && lidar_object->obj().id()!= 17306
      //   && lidar_object->obj().id()!= 17322 ) {
      //   continue;
      // }
      std::ofstream vegetation_anno(file_name,std::ios::app);

      ROS_DEBUG_STREAM("GetVegetationCandidate: Lidar object id: " << lidar_object->obj().id());
      ROS_DEBUG_STREAM("GetVegetationCandidate: Lidar object road status: "
                        << lidar_object->obj().road_status());
      std::cout << tmp_type << " " << id << " 0 0 0 0 0 0 0 " << lidar_object->obj().center().x()
                << " " << lidar_object->obj().center().y() << " "
                << lidar_object->obj().center().z() << " " << lidar_object->obj().size().x() << " "
                << lidar_object->obj().size().y() << " " << lidar_object->obj().size().z() << " 0"
                << "\n";
      vegetation_anno << tmp_type << " " << id << " 0 0 0 0 0 0 0 "
                      << lidar_object->obj().center().x() << " " << lidar_object->obj().center().y()
                      << " " << lidar_object->obj().center().z() << " "
                      << lidar_object->obj().size().x() << " " << lidar_object->obj().size().y()
                      << " " << lidar_object->obj().size().z() << " 0"
                      << "\n";
      id++;
    }
  }
  #endif
  std::vector<perception::TrackedObject*> candidates_list;
  std::unordered_set<int> current_frame_objects;
  for (int i = 0; i < lidar_object_cnt; i++) {
    perception::TrackedObject* lidar_object =
        fusion_rawdata_.current_lidarobjects_data_.mutable_objs(i);
    const int lidar_object_id = lidar_object->obj().id();
    current_frame_objects.insert(lidar_object_id);
    const double speed = 3.6 * std::sqrt(std::pow(lidar_object->velocity().x(), 2) +
                                         std::pow(lidar_object->velocity().y(), 2));
    if (speed > max_vegetation_velocity_) {
      object_id_to_velocity_cnt_[lidar_object_id]++;
    }
    if ((is_vegetation_infer_ && IsVegetationCandidate(*lidar_object)) ||
        (is_noise_infer_ && IsNoiseCandidate(*lidar_object))) {
      candidates_list.push_back(lidar_object);
    }
  }
  // Clear not current object infos.
  std::vector<int> not_current_frame_object;
  for (auto item : object_id_to_velocity_cnt_) {
    if (!current_frame_objects.count(item.first)) {
      not_current_frame_object.push_back(item.first);
    }
  }
  for (auto lidar_object_id : not_current_frame_object) {
    object_id_to_velocity_cnt_.erase(lidar_object_id);
  }
  return candidates_list;
}

PreprocessFpntInput FpnetProcessingUnit::ExtractPcdAndBox() {
  PreprocessFpntInput res;
  int cam_num = 0;
  constexpr int k2DBoxFeatureDim = 7;
  {
    perception::mid_fusion::TicToc timer("ExtractPcdAndBox/GetAllPoints");
    res.ClearData();
    int N = fusion_rawdata_.current_cloud_data_.cloud_ptr->size();
    cam_num = map_current_camera2d_data_.size();
    ROS_INFO_STREAM("ExtractPcdAndBox: cloud data N is " << N);
    if (N < kMinNumberOfPoint || N > kMaxNumberOfPoint) {
      ROS_ERROR_STREAM("ExtractPcdAndBox: The number of cloud point are invalid! "
                       << "N = " << N);
      return res;
    }
    // Extract pcd_xyzi.
    res.pcd_xyzi.resize(N * 4, 0);
    omp_set_num_threads(8);
#pragma omp parallel for
    for (int i = 0; i < N; ++i) {
      const float intensity = fusion_rawdata_.current_cloud_data_.cloud_ptr->points[i].intensity;
      if (intensity > 0.0) {
        res.pcd_xyzi[i * 4] = fusion_rawdata_.current_cloud_data_.cloud_ptr->points[i].x;
        res.pcd_xyzi[i * 4 + 1] = fusion_rawdata_.current_cloud_data_.cloud_ptr->points[i].y;
        res.pcd_xyzi[i * 4 + 2] = fusion_rawdata_.current_cloud_data_.cloud_ptr->points[i].z;
        res.pcd_xyzi[i * 4 + 3] = intensity;
      }
    }
  }

  // If turn off the small object detetion, we skip the camera 2d box collection.
  if (!is_small_object_infer_) {
    return res;
  }
  // Extract camera 2D box.
  int box_num = 0;
  for (const auto& autocamera : map_current_camera2d_data_) {
    int box_size_temp = autocamera.second.objs_size();
    box_num += box_size_temp;
  }
  ROS_INFO_STREAM("ExtractPcdAndBox:box_num: " << box_num);
  if (cam_num == 0 || box_num == 0) {
    FaultManager("Perception_MidFusion_LackCamre2dDetection");
    return res;
  }
  std::vector<float> box_det2d(box_num * 7, 0);
  std::vector<uint32_t> box_ID(box_num, 1);
  cam_hw_.resize(raw_camera_num_ * 2, 0);

  int index = 0;
  for (const auto& autocamera : map_current_camera2d_data_) {
    const std::string& camera_name = autocamera.first;
    const perception::VisualObjects& camera_objects = autocamera.second;
    if (camera_name != "/perception/camera/camera_obstacle_front30" &&
        camera_name != "/perception/camera/camera_obstacle_front60" &&
        camera_name != "/perception/camera/camera_obstacle_right120") {
      continue;
    }
    Eigen::VectorXd distcoeff(5);
    Eigen::Matrix4d intrinsics;
    distcoeff = distcoeff_camera_map_[camera_objects.sensor_name()];
    intrinsics = intrinsics_camera_map_[camera_objects.sensor_name()];
    cam_hw_[id_map_[camera_name] * 2] = camera_objects.height();
    cam_hw_[id_map_[camera_name] * 2 + 1] = camera_objects.width();
    vector<int> margin_vec(camera_objects.objs_size(), -1);
    const int box2d_size_temp = MarginProcess(camera_objects, margin_vec, margin_fpnet_);
    for (int j = 0; j < box2d_size_temp; ++j) {
      int margin_index = margin_vec[j];
      if (kCameraSmallObjectTypes.count(camera_objects.objs(margin_index).obj().type())) {
        if (camera_objects.objs(margin_index).obj().id() == 4294967295) {
          continue;
        }
        if (is_only_pub_cone_ &&
            !kTrafficConeObjectTypes.count(camera_objects.objs(margin_index).obj().type())) {
          continue;
        }
        if (camera_name == "/perception/camera/camera_obstacle_front30" &&
            CameraTypeConvertToFpnetIndex(camera_objects.objs(margin_index).obj().type()) == 9) {
          continue;
        }
        const perception::VisualObject& camera_object = camera_objects.objs(margin_index);
        perception::VisualObject camera_object_undistort;
        UndisortBox(camera_object, intrinsics, distcoeff, camera_object_undistort);
        // std::cout << "* * * undistorted * * * " << camera_object_undistort.x() << " " << camera_object_undistort.y() << " " << camera_object_undistort.width() << " " << camera_object_undistort.height() << std::endl;
        box_det2d[index * 7] = id_map_[camera_name];
        box_det2d[index * 7 + 1] = camera_object_undistort.x() + camera_object_undistort.width() / 2;
        box_det2d[index * 7 + 2] = camera_object_undistort.y() + camera_object_undistort.height() / 2;
        box_det2d[index * 7 + 3] = camera_object_undistort.width();
        box_det2d[index * 7 + 4] = camera_object_undistort.height();
        box_det2d[index * 7 + 5] = camera_object.obj().confidence();
        box_det2d[index * 7 + 6] = CameraTypeConvertToFpnetIndex(camera_object.obj().type());
        box_ID[index] = camera_object.obj().id() + 2000000000;
        // std::cout << "box raw id: " << camera_object.obj().id() << "\n";
        // std::cout << "box raw type: " << box_det2d[index * 7 + 6] << "\n";
        // std::cout << "box camera name: " << box_det2d[index * 7] << "\n";
        // std::cout << "box camera raw name: " << camera_name << "\n";
        res.index_to_camtopic_map[index] = camera_name;
        index++;
      }
    }
  }
  // test
  // index = 1;
  // Todo
  res.box_det2d.assign(box_det2d.begin(), box_det2d.begin() + k2DBoxFeatureDim * index);
  res.box_ID.assign(box_ID.begin(), box_ID.begin() + index);
  res.box_count = index;
  return res;
}

void FpnetProcessingUnit::PublishFpntObjects(const std::vector<float>& pred_box3d_temp,
                                             const PreprocessFpntInput& pre_res) {
  // TODO: 重新细拆该接口功能
  pcl::StopWatch Run_timer1;
  const std::vector<uint32_t>& box_ID = pre_res.box_ID;
  const int box_2d_count = pred_box3d_temp.size() / 9;
  perception::TrackedObjects lidar_objects_fpnet;
  TrackObjectInfoCopy(fusion_rawdata_.current_lidarobjects_data_, lidar_objects_fpnet);
  // update local_current
  localization::Localization local_current;
  double timestamp = lidar_objects_fpnet.header().stamp().sec() +
                     lidar_objects_fpnet.header().stamp().nsec() * 1e-9;
  if (!QueryNearestLocalization(timestamp, local_current)) {
    ROS_ERROR_STREAM(
        "PublishFpntObjects: Fail to get localization for lidar_objects_fpnet measurement.");
    return;
  }
  double host_yaw_global = local_current.yaw();
  double cos_host = std::cos(host_yaw_global);
  double sin_host = std::sin(host_yaw_global);
  ROS_INFO_STREAM("PublishFpntObjects: ################ after predict ##############.");
  for (int i = 0; i < box_2d_count; ++i) {
    if (pred_box3d_temp[i * 9] > 1.5 && box_ID[i] != 4294967295) {
      // 对所有模型预测的目标计数[id, tracked_times]
      std::map<uint32_t, double>::iterator iter;
      iter = multi_frame_infer_obj_count_.find(box_ID[i]);
      ROS_DEBUG_STREAM(H_DEBUG_R << "id: " << box_ID[i]);
      if (iter != multi_frame_infer_obj_count_.end()) {
        ROS_DEBUG_STREAM(H_DEBUG_R << "id: " << box_ID[i] << " add 1");
        double tmp_tracked_times = iter->second + 1;
        multi_frame_infer_obj_count_[box_ID[i]] = tmp_tracked_times;
      } else {
        ROS_DEBUG_STREAM(H_DEBUG_R << "i am init.");
        multi_frame_infer_obj_count_.emplace(box_ID[i], 1);
      }
      const int fpnet_model_pred = static_cast<int>(pred_box3d_temp[i * 9 + 8] + 0.001f);
      const int box_cls = pre_res.box_det2d[i * 7 + 6];  // no priori
      // If frustum points is empty, set it pred as -1; also skip background pred(0).
      if (fpnet_model_pred <= 0) {
        ROS_WARN_STREAM("PublishFpntObjects: empty frustum points" << i);
        continue;
      }
      if (is_only_pub_cone_) {
        if (FpnetIndexConvertToLidarType(box_cls) !=
            perception::ObjectType::TYPE_TRIANGLEROADBLOCK) {
          ROS_WARN_STREAM("PublishFpntObjects: this is not cone! " << i);
          continue;
        }
      }
      perception::TrackedObject lidar_obj_temp;
      // TODO(liuxinyu): set timestamp
      // mutableobject_ptr->set_time_stamp(timestamp);
      ConvertFpnetResultToObject(pred_box3d_temp, pre_res, local_current, i, lidar_obj_temp);
      perception::TrackedObject* result_object = lidar_objects_fpnet.add_objs();
      result_object->CopyFrom(lidar_obj_temp);
    }
  }
  ROS_DEBUG_STREAM("======fpnet_result CheckContinuity Before======"
                   << "\n"
                   << lidar_objects_fpnet.DebugString());
  CheckContinuity();
  last_infer_obj_count_ = multi_frame_infer_obj_count_;
  ROS_DEBUG_STREAM("lidar_objects_fpnet size:   " << lidar_objects_fpnet.objs_size());
  std::unordered_map<int, std::vector<int>> lidar_intersect_fpnet;
  std::unordered_set<int> fpnet_match_big;
  perception::TrackedObjects lidar_objects_fpnet_result;
  TrackObjectInfoCopy(lidar_objects_fpnet, lidar_objects_fpnet_result);
  FpnetobjectMatch(lidar_objects_fpnet.objs(), lidar_objects_fpnet_result.mutable_objs());
  bool is_tracker = false;
  if (past_frames_fpnet_obstacles_.size() == MODEL_INFER_HISTORY_BUFF_SIZE) {
    is_tracker = true;
    ComputeInitialVelocity(past_frames_fpnet_obstacles_.front(), lidar_objects_fpnet_result,
                           multi_frame_infer_obj_count_, local_current);
    ReasonablenessCheck(past_frames_fpnet_obstacles_.back(), lidar_objects_fpnet_result,
                        local_current);
  }
  // Save past frames fpnet obstacles
  past_frames_fpnet_obstacles_.push_back(lidar_objects_fpnet_result);
  if (past_frames_fpnet_obstacles_.size() > MODEL_INFER_HISTORY_BUFF_SIZE) {
    past_frames_fpnet_obstacles_.pop_front();
  }

  if (is_tracker) {
    ROS_DEBUG_STREAM("======fpnet_result CheckContinuity After======"
                     << "\n"
                     << lidar_objects_fpnet_result.DebugString());
    /* tracking begin */
    mid_fusion::Frame detected_frame;
    TrackerOptions options;
    // Mogo2Tracker
    detected_frame.timestamp = lidar_objects_fpnet_result.header().stamp().sec() +
                               lidar_objects_fpnet_result.header().stamp().nsec() * 1e-9;
    for (size_t i = 0; i < lidar_objects_fpnet_result.objs_size(); i++) {
      TrackedObject* det_obj = lidar_objects_fpnet_result.mutable_objs(i);
      mid_fusion::ObjectPtr candidate_tracker_obj(new mid_fusion::Object());
      Mogo2Tracker(*det_obj, candidate_tracker_obj);
      detected_frame.objects.emplace_back(candidate_tracker_obj);
    }
    mid_fusion::FramePtr tracked_frame(new mid_fusion::Frame);
    tracker_->Track(detected_frame, options, tracked_frame);
    perception::TrackedObjects fpnet_tracked_result;
    // TODO: confidence type // UTM positon velociety yaw? // Veh center velociety size polygon
    // angle
    Tracker2Mogo(detected_frame, tracked_frame, fpnet_tracked_result, local_current);
    /* tracking end */
    ROS_DEBUG_STREAM("======fpnet_tracked_result======"
                     << "\n"
                     << fpnet_tracked_result.DebugString());
    /* 去重 begin */
    {
      perception::mid_fusion::TicToc timer("PublishFpntObjects/DuplicateRemove");
      ObjectMatch(fpnet_tracked_result, fusion_rawdata_.current_lidarobjects_data_,
                  lidar_intersect_fpnet, fpnet_match_big);
      perception::mid_fusion::TicToc del_timer("PublishFpntObjects/DelCandidate");
      std::vector<int> candidate_del_obj;
      candidate_del_obj.clear();
      for (size_t i = 0; i < lidar_intersect_fpnet.size(); i++) {
        std::vector<int> match_idx_vec = lidar_intersect_fpnet[i];
        for (size_t match_idx = 0; match_idx < match_idx_vec.size(); match_idx++) {
          if (match_idx_vec[match_idx] != -1) {
            candidate_del_obj.push_back(match_idx_vec[match_idx]);
          }
        }
      }
      // 从大到小删Lidar Obj，避免索引位置变化
      sort(candidate_del_obj.rbegin(), candidate_del_obj.rend());
      candidate_del_obj.erase(unique(candidate_del_obj.begin(), candidate_del_obj.end()),
                              candidate_del_obj.end());
      for (std::vector<int>::iterator it = candidate_del_obj.begin(); it != candidate_del_obj.end();
           it++) {
        fusion_rawdata_.current_lidarobjects_data_.mutable_objs()->DeleteSubrange(*it, 1);
      }
    }
    /* 去重 end */
    // 添加新检测的小目标
    for (int i = 0; i < fpnet_tracked_result.objs_size(); ++i) {
      if (fpnet_match_big.count(fpnet_tracked_result.objs(i).obj().id()) == 0) {
        const perception::TrackedObject& lidar_fpobj = fpnet_tracked_result.objs(i);
        perception::TrackedObject* result_object1 =
            fusion_rawdata_.current_lidarobjects_data_.add_objs();
        result_object1->CopyFrom(lidar_fpobj);
      }
    }
    ROS_INFO_STREAM(
        "PublishFpntObjects: fpnet_tracked_result.size: " << fpnet_tracked_result.objs_size());
  }
  ROS_INFO_STREAM("PublishFpntObjects: f-current_lidarobjects_data.size: "
                  << fusion_rawdata_.current_lidarobjects_data_.objs_size());
  // 发布与可视化
  if (is_addfpnet_visualization_pub_) {
    perception::mid_fusion::TicToc timer("PublishFpntObjects/Rviz");
    DisplayLidarobject(fusion_rawdata_.current_lidarobjects_data_);
  }
  // triger
  std::vector<std::string> trigger_info;
  if (enable_trigger_) {
    if (trigger_manager_ptr_ != nullptr) {
        TicToc timer("perception/trigger");
        trigger_manager_ptr_->perception(fusion_rawdata_.current_lidarobjects_data_, trigger_info);
    }
    ParserTriggerFromRes(fusion_rawdata_.current_lidarobjects_data_, trigger_info);
  }

  std_msgs::String middle_fusion_frame;
  fusion_rawdata_.current_lidarobjects_data_.SerializeToString(&middle_fusion_frame.data);
  pub_lidar_objects_.publish(middle_fusion_frame);
  double pub_timestamp = fusion_rawdata_.current_lidarobjects_data_.header().stamp().sec() +
                         fusion_rawdata_.current_lidarobjects_data_.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(1, "PublishFpntObjects:DataCheck size: %d time: %lf\n",
                    fusion_rawdata_.current_lidarobjects_data_.objs_size(), pub_timestamp);
  // DisplayLidarobject(fpnet_tracked_result);
}

void FpnetProcessingUnit::PointsProcess() {
  // fine the newest camera detection objects
  perception::VisualObjects camera_detector_obj;
  camera_detector_obj = map_current_camera2d_data_[rec_topic_name_front_camera_obstacle_front60_];
  DebugLidarCameraSync(detection_image_buffer_map_, intrinsics_camera_map_, extrinsics_camera_map_,
                       camera_detector_obj, fusion_rawdata_.current_cloud_data_,
                       pub_cloudpointsImg_);
}

void FpnetProcessingUnit::DebugVisualProcess() {
  std::vector<perception::VisualObjects> camera_frame_vec;
  for (const auto& camera_item : map_current_camera2d_data_) {
    const std::string& camera_id = camera_item.first;
    const perception::VisualObjects& camera_objects = camera_item.second;
    camera_frame_vec.push_back(camera_objects);
    ROS_DEBUG_STREAM("LidarObjectsProcess: " << camera_objects.sensor_name() << camera_id << "   : "
                                             << camera_frame_vec.back().objs_size());
  }
  std::unordered_map<std::string, Eigen::MatrixXd> world_points_map;
  GetImageFrustum(camera_frame_vec, intrinsics_camera_map_, extrinsics_camera_map_,
                  world_points_map);
  DisplayImageFrustum(world_points_map);
}

void FpnetProcessingUnit::DisplayImageFrustum(
    std::unordered_map<std::string, Eigen::MatrixXd>& world_points_map) {
  visualization_msgs::MarkerArray marker_array;
  mid_visualization_.lineMarker(world_points_map, marker_array);
  if (!marker_array.markers.empty()) {
    visualization_camera_frustum_.publish(marker_array);
  }
}

void FpnetProcessingUnit::DisplayLidarobject(perception::TrackedObjects& lidar_measurement) {
  DisplayObject(lidar_measurement, mid_visualization_, visualization_lidar_obstacle_pub_,
                visualization_lidar_obstacle_text_pub_, visualization_lidar_obstacle_polygon_pub_);
}

void FpnetProcessingUnit::LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  static double last_ts = 0.0;
  double ts_dis = msg->header.stamp.toSec() - last_ts;
  if (last_ts != 0 && ts_dis > 1.0)
    ROS_WARN("LocalizationCallback: Localization message time out!");
  last_ts = msg->header.stamp.toSec();

  while (global_localizations_.size() >= 200) global_localizations_.pop_front();

  localization::Localization global_localization;
  RosToProto(*msg, global_localization);
  if (global_localization.utm_zone() != 0)
    std::string UTM_ZONE = std::to_string(global_localization.utm_zone());
  else
    ROS_ERROR("LocalizationCallback: Localization utm_zone zero error!");
  global_localizations_.push_back(global_localization);
}

void FpnetProcessingUnit::HadmapLaneCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  static double latest_ts = 0.0;
  double system_time = ros::Time::now().toSec();
  double ts_dis = msg->header.stamp.toSec() - latest_ts;
  if (latest_ts != 0 && ts_dis > 1.0) ROS_WARN("HadmapLaneCallback: HadmapLane message time out!");
  if (system_time - latest_ts > 10.0) {
    ROS_DEBUG_STREAM("HadmapLaneCallback: system time_diff = "
                     << std::setprecision(18) << system_time - latest_ts
                     << ", between system_time:" << system_time << " and latest_ts:" << latest_ts);
    lanemarks_.clear();
  }
  latest_ts = msg->header.stamp.toSec();
  while (lanemarks_.size() >= 10) {
    lanemarks_.pop_front();
  }

  if (msg) {
    hadmap::MapMsg map;
    perception::mid_fusion::LaneMarkerDataRecord lanemark;
    lanemark.clear();
    RosToProto<hadmap::MapMsg>(*msg, map);
    localization::Localization local_current;
    double timestamp = map.header().stamp().sec() + map.header().stamp().nsec() * 1e-9;
    if (!QueryNearestLocalization(timestamp, local_current)) {
      ROS_ERROR_STREAM("HadmapLaneCallback: Fail to get localization for HadmapLaneCallback.");
      return;
    }
    ConvertMapMsg2LaneData(map, lanemark, local_current);
    lanemarks_.emplace_back(lanemark);
  }
}

bool FpnetProcessingUnit::QueryNearestLocalization(const double& timestamp,
                                            localization::Localization& localization) {
  if (global_localizations_.empty()) {
    ROS_ERROR_STREAM("QueryNearestLocalization: Localization message NOT received.");
    return false;
  }
  localization = global_localizations_.front();
  // reduce timestamp by time delay of sensor data transmission and perception consuming. now 0.0
  double stamp = timestamp + 0.05;
  double loc_ts;
  localization::Localization global_localization;
  for (auto it = global_localizations_.rbegin(); it != global_localizations_.rend(); it++) {
    global_localization = *it;
    loc_ts = global_localization.header().stamp().sec() +
             global_localization.header().stamp().nsec() * 1e-9;
    if (loc_ts > stamp) {
      continue;
    } else {
      localization = global_localization;
      break;
    }
  }
  if (abs(stamp - loc_ts) > 0.3) {
    ROS_ERROR_STREAM("QueryNearestLocalization: Time distance "
                     << std::setprecision(3) << abs(stamp - loc_ts)
                     << " between  InputData:: " << std::setprecision(18) << stamp
                     << " and localization: " << std::setprecision(18) << loc_ts
                     << " is too long.");
    return false;
  }
  return true;
}

void FpnetProcessingUnit::CheckContinuity() {
  if (!last_infer_obj_count_.empty() && !multi_frame_infer_obj_count_.empty()) {
    std::map<uint32_t, double>::iterator iter;
    for (iter = multi_frame_infer_obj_count_.begin(); iter != multi_frame_infer_obj_count_.end();) {
      auto res = last_infer_obj_count_.find(iter->first);
      if (res != last_infer_obj_count_.end()) {
        ROS_DEBUG_STREAM(H_DEBUG_R << "CheckContinuity: current: " << iter->second
                                   << " last:" << res->second);
        if (iter->second != (res->second + 1)) {
          uint32_t del_id = iter->first;
          iter++;
          multi_frame_infer_obj_count_.erase(del_id);
        } else {
          iter++;
        }
      } else {
        iter++;
      }
    }
  }
}

void FpnetProcessingUnit::LoadVehConfParam() {
  //TODO:增加保护机制？
  perception::mid_fusion::TicToc::enable_timer_ = config_parser_ptr_->getBool("is_print_timecost");
  is_pub_cloudpointsImg_ = config_parser_ptr_->getBool("is_pub_cloudpointsImg");
  is_addfpnet_visualization_pub_ = config_parser_ptr_->getBool("is_addfpnet_visualization_pub");
  is_pub_camera_frustum_ = config_parser_ptr_->getBool("is_pub_camera_frustum");
  is_pub_frustum_points_ = config_parser_ptr_->getBool("is_pub_frustum_points");
  max_time_tr_ = config_parser_ptr_->getDouble("max_time_tr");
  margin_fpnet_ = config_parser_ptr_->getInt("margin_fpnet");
  is_fpnet_ = config_parser_ptr_->getBool("is_fpnet");
  // Load pub param
  is_only_pub_cone_ = config_parser_ptr_->getBool("is_only_pub_cone");
  // Load vegetation and small object infer flags
  is_vegetation_infer_ = config_parser_ptr_->getBool("is_vegetation_infer");
  is_small_object_infer_ = config_parser_ptr_->getBool("is_small_object_infer");
  is_noise_infer_ = config_parser_ptr_->getBool("is_noise_infer");
  // Load vegetation roi params
  vegetation_roi_x_max_ = config_parser_ptr_->getDouble("vegetation_roi_x_max");
  vegetation_roi_x_min_ = config_parser_ptr_->getDouble("vegetation_roi_x_min");
  vegetation_roi_y_max_ = config_parser_ptr_->getDouble("vegetation_roi_y_max");
  vegetation_roi_y_min_ = config_parser_ptr_->getDouble("vegetation_roi_y_min");
  vegetation_threshold_ = config_parser_ptr_->getDouble("vegetation_threshold");
  max_vegetation_velocity_ = config_parser_ptr_->getDouble("max_vegetation_velocity");
  // Load Noise roi params 
  noise_roi_x_max_ = config_parser_ptr_->getDouble("noise_roi_x_max");
  noise_roi_x_min_ = config_parser_ptr_->getDouble("noise_roi_x_min");
  noise_roi_y_max_ = config_parser_ptr_->getDouble("noise_roi_y_max");
  noise_roi_y_min_ = config_parser_ptr_->getDouble("noise_roi_y_min");
  fixed_noise_minimum_score_thr_ = config_parser_ptr_->getDouble("fixed_noise_minimum_score_thr");
  // load camere topic
  rec_topic_name_front_camera_obstacle_front30_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_front30");
  rec_topic_name_front_camera_obstacle_front60_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_front60");
  rec_topic_name_front_camera_obstacle_front120_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_front120");
  rec_topic_name_front_camera_obstacle_left120_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_left120");
  rec_topic_name_front_camera_obstacle_right120_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_right120");
  rec_topic_name_back_camera_obstacle_back120_ =
      config_parser_ptr_->getString("rec_topic_name_back_camera_obstacle_back120");
  rec_topic_name_front_camera_detection_60_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_detection_60");

  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_60"] =
      rec_topic_name_front_camera_obstacle_front60_;
  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_30"] =
      rec_topic_name_front_camera_obstacle_front30_;
  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_120"] =
      rec_topic_name_front_camera_obstacle_front120_;
  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_back"] =
      rec_topic_name_back_camera_obstacle_back120_;
  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_left"] =
      rec_topic_name_front_camera_obstacle_left120_;
  inextritopic_totopic_map_["/sensor/camera/sensing/image_raw_right"] =
      rec_topic_name_front_camera_obstacle_right120_;
  // load radar topic
  rec_topic_name_radar_obstacle_ = config_parser_ptr_->getString("rec_topic_name_radar_obstacle");
  // load localization topic
  rec_topic_name_localization_ = config_parser_ptr_->getString("rec_topic_name_localization");
  // FM param 
  min_lidar_camera_sync_time_thr_ = config_parser_ptr_->getDouble("min_lidar_camera_sync_time_thr");
  enable_trigger_ = config_parser_ptr_->getBool("enable_trigger");
}

void FpnetProcessingUnit::FaultManager(std::string fault_id) {
  bool is_existence = false;
  for (size_t i = 0; i < output_fault_messages_.infos_size(); i++) {
    if (output_fault_messages_.infos(i).fault_id() == fault_id) {
      is_existence = true;
      break;
    }
  }
  if (!is_existence) {
    output_fault_messages_.set_src("perception_fusion_mid_node");
    fault_management::FaultInfo* fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id(fault_id);
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
  }
}

void FpnetProcessingUnit::ParserTriggerFromRes(perception::TrackedObjects& lidar_measurement,
                                           std::vector<std::string>& trigger_infos) {
  if (trigger_infos.empty()) {
    return;
  }
  static int seq = 0;
  ros::Time ts;
  double timestamp =
      lidar_measurement.header().stamp().sec() + lidar_measurement.header().stamp().nsec() * 1e-9;
  ts.fromSec(timestamp);
  ros::Time ts_start;
  ts_start.fromSec(timestamp - 3.0);
  ros::Time ts_end;
  ts_end.fromSec(timestamp + 5.0);
  trigger_info::TriggerInfo trigger_msg;
  trigger_msg.mutable_header()->set_seq(seq++);
  trigger_msg.mutable_header()->mutable_stamp()->set_sec(ts.sec);
  trigger_msg.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
  trigger_msg.mutable_header()->set_frame_id("base_link");
  trigger_msg.mutable_header()->set_module_name("Bus-MidFusion");
  trigger_msg.mutable_timestamp_start()->set_sec(ts_start.sec);
  trigger_msg.mutable_timestamp_start()->set_nsec(ts_start.nsec);
  trigger_msg.mutable_timestamp_end()->set_sec(ts_end.sec);
  trigger_msg.mutable_timestamp_end()->set_nsec(ts_end.nsec);
  for (size_t i = 0; i < trigger_infos.size() / 2; i++) {
    trigger_info::Label* info;
    info = trigger_msg.add_trigger_label();
    info->set_primary_label(trigger_infos[i * 2 + 0]);
    info->set_secondary_label(trigger_infos[i * 2 + 1]);
  }
  trigger_msg.set_case_num(20);
  trigger_msg.set_level(1);
  proto_trigger_publisher_.publish(pub_trigger_, trigger_msg);
}

}  // namespace mid_fusion
}  // namespace perception
