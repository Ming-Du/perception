#include "perception_fusion_component.h"
namespace perception {
namespace fusion {
extern std::string UTM_ZONE;
static const double PI(3.141592653589793238462);
static map::V_Points fov_c60_bev_;

bool PerceptionFusionComponent::Init() {
  ros::NodeHandle pn("~");
  node_ptr_.reset(new ros::NodeHandle);
  // read fusion config
  FusionComponentInitOptions options;
  ROS_DEBUG_STREAM("Init: read config from: " << FLAGS_multi_sensor_fusion_conf_path);
  if (!::common::GetProtoFromFile(FLAGS_multi_sensor_fusion_conf_path, options)) {
    ROS_ERROR("Init: failed to load config file.");
    return false;
  }

  pub_obu_ = options.pub_obu();
  pub_vidar_ = options.pub_vidar();
  pub_radar_ = options.pub_radar();
  falcon_lidar_filter_distance_ = options.falcon_lidar_filter_distance();
  enable_publish_planning_ = options.enable_publish_fusion_topic();
  enable_publish_app_ = options.enable_publish_fusion_topic_app();
  use_ground_map_ = options.use_ground_map();

  // to load component configs
  fusion_method_ = options.fusion_method();
  for (size_t indx = 0; indx < options.fusion_main_sensors_size(); ++indx) {
    fusion_main_sensors_.push_back(options.fusion_main_sensors(indx));
  }
  fusion_main_sensors_bk_ = fusion_main_sensors_;
  object_in_roi_check_ = options.object_in_roi_check();
  radius_for_roi_object_check_ = options.radius_for_roi_object_check();

  // init algorithm plugin
  if (!(InitAlgorithmPlugin())) {
    ROS_ERROR("Init: Failed to init algorithm plugin.");
  }
  InitSubTopics(options);
  InitPubTopics(options);
  ConfigManager::Instance()->get_value("radar_filter_yaw_thr", &radar_filter_yaw_thr_);
  ConfigManager::Instance()->get_value("release_mode", &release_mode_);
  ConfigManager::Instance()->get_value("vehicle", &vehicle_);

//Modify by jiangnan : add virtual_object
  perception::VirtualObject virtual_object;
  virtual_objects.clear();
  for (size_t index = 0; index < options.virtual_object_size(); index++) {
      virtual_object.CopyFrom(options.virtual_object(index));
      virtual_objects.push_back(virtual_object);
  };
  output_fault_messages_.set_src("perception_fusion2_node");
  InitV2xParam();
  int queueSize = 1;
  map::Init(options.semantic_map_name());//加载语义地图

  display_ptr.reset(new RvizDisplay(node_ptr_));
  return true;
}

void PerceptionFusionComponent::InitSubTopics(FusionComponentInitOptions& options){
  base::SensorManager* sensor_manager = base::SensorManager::Instance();
  int queueSize =1;
  for (size_t indx = 0; indx < options.input_sensor_size(); ++indx) {
    perception::InputSensor input_sensor = options.input_sensor(indx);
    if(options.use_lidar() && sensor_manager->IsLidar(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame =node_ptr_->subscribe(input_sensor.topic(), queueSize,&PerceptionFusionComponent::LidarObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber lidar topic: " << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        continue;
    }
    if(options.use_falcon_lidar() && sensor_manager->IsFalconLidar(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame =node_ptr_->subscribe(input_sensor.topic(), queueSize,&PerceptionFusionComponent::FalconLidarObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber falcon  lidar topic: " << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        continue;
    }
    if(options.use_camera()  && sensor_manager->IsCamera(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame =node_ptr_->subscribe(input_sensor.topic(), queueSize,&PerceptionFusionComponent::CameraObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber camera topic: " << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        continue;
    }
    if(options.use_vidar() && sensor_manager->IsVidar(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame =node_ptr_->subscribe(input_sensor.topic(), queueSize,&PerceptionFusionComponent::VidarObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber vidar topic:" << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        continue;
    }
    if(options.use_radar() && sensor_manager->IsRadar(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame =node_ptr_->subscribe(input_sensor.topic(), queueSize,&PerceptionFusionComponent::RadarObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber radar topic: " << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        continue;
    }
    if(options.use_obu() && sensor_manager->IsObu(input_sensor.sensor_name())){
        ros::Subscriber sub_sensor_frame = node_ptr_->subscribe(input_sensor.topic(), queueSize, &PerceptionFusionComponent::ObuObjectsCallback, this);
        ROS_DEBUG_STREAM("Init: subscriber obu topic: " << input_sensor.topic());
        subscribers_map_[input_sensor.sensor_name()] = sub_sensor_frame;
        // Get v2x ros param from app
        GetV2XParamThread();
        continue;
    }
  }
  ros::Subscriber localization_subscriber = node_ptr_->subscribe(options.localization_topic(), queueSize,
                                          &PerceptionFusionComponent::LocalizationCallback, this);
  subscribers_map_["localization"] = localization_subscriber;
  ROS_DEBUG_STREAM("Init: subscriber location topic: " << options.localization_topic());

  ros::Subscriber param_set_cmd_subscriber = node_ptr_->subscribe("/telematics/param_set_cmd", 10,
                                           &PerceptionFusionComponent::ParamSetCmdCallback, this);
  subscribers_map_["paramsetcmd"] = param_set_cmd_subscriber;
  ROS_DEBUG_STREAM("Init: subscriber CMDParamSet topic: /telematics/param_set_cmd");
  ros::Subscriber hadmap_lane_subscriber = node_ptr_->subscribe("/hadmap_engine/lanes_msg", 1, 
                                         &PerceptionFusionComponent::HadmapLaneCallback, this);
  subscribers_map_["hadmap_lane"] = hadmap_lane_subscriber;
  ROS_DEBUG_STREAM("Init: subscriber hadmap_lane topic: " << "/hadmap_engine/lanes_msg");

  ros::Subscriber camera_detection_f60_subscriber = node_ptr_->subscribe("/perception/camera/object_detection_front60", 1, 
                                                  &PerceptionFusionComponent::Front60CameraDetectionCallback, this);
  subscribers_map_["camera_detection_f60"] = camera_detection_f60_subscriber;
  ROS_DEBUG_STREAM("Init: subscriber camera_detection_f60 topic: " << "/perception/camera/object_detection_front60");

  ros::Subscriber falcon_ground_subscriber = node_ptr_->subscribe("/perception/lidar/ground_fitting_info", 1, 
                                           &PerceptionFusionComponent::GroundFittingInfoCallback, this);
  subscribers_map_["falcon_ground"] = falcon_ground_subscriber;
  ROS_DEBUG_STREAM("Init: subscriber falcon_ground topic: " << "/perception/lidar/ground_fitting_info");


}
void PerceptionFusionComponent::InitPubTopics(FusionComponentInitOptions& options){
  int queueSize =1;
  if (options.enable_publish_fusion_topic()){
    enable_publish_planning_ = true;
    ros::Publisher fusion_obstacles_pub_planning = node_ptr_->advertise<autopilot_msgs::BinaryData>(options.output_obstacles_topic_name(), queueSize);
    publishers_map_["toplanning"] = fusion_obstacles_pub_planning;
    ROS_DEBUG_STREAM("Init: publisher obstacle to planning topic: " << options.output_obstacles_topic_name());
  }
  if(options.enable_publish_fusion_topic_app()){
    enable_publish_app_ = true;
    ros::Publisher fusion_obstacles_pub_app = node_ptr_->advertise<autopilot_msgs::BinaryData>(
        options.output_obstacles_topic_name_app(), queueSize);
    publishers_map_["toapp"] = fusion_obstacles_pub_app;
    ROS_DEBUG_STREAM("Init: publisher obstacle to app topic: " << options.output_obstacles_topic_name_app());
  }
  if(options.pub_obu()){
    ros::Publisher obu_rviz_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/viz_obu", queueSize);
    ros::Publisher obu_rte_rviz_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/rviz_obu_rte_pass", queueSize);
    ros::Publisher obu_tmp_rviz_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/viz_obu_pass", queueSize);
    publishers_map_["torviz_obu"] = obu_rviz_pub;
    publishers_map_["torviz_oburte"] = obu_rte_rviz_pub;
    publishers_map_["torviz_obutmp"] = obu_tmp_rviz_pub;
    // ROS_DEBUG_STREAM("Init: publisher obu to rviz topic: " << options.output_obstacles_topic_name_app());
  }

  ros::Publisher fusion_obstacles_pub_obu = node_ptr_->advertise<autopilot_msgs::BinaryData>("/perception/fusion/obstacles_obu", queueSize);
  publishers_map_["toall_obu"] = fusion_obstacles_pub_obu;
  ROS_DEBUG_STREAM("Init: publisher obuobstacles topic:/perception/fusion/obstacles_obu ");

  ros::Publisher fault_manger_report = node_ptr_->advertise<autopilot_msgs::BinaryData>("/system/fm/fault_report", queueSize);
  publishers_map_["faultmanger"] = fault_manger_report;
  ROS_DEBUG_STREAM("Init: publisher faultmanger topic: /system/fm/fault_report");
  
  ros::Publisher vidar_rviz_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/vidar", queueSize);
  publishers_map_["vidarrviz"] = vidar_rviz_pub;
  ROS_DEBUG_STREAM("Init: publisher faultmanger topic: /perception/fusion/vidar");
  
  ros::Publisher radar_filtered_rviz = node_ptr_->advertise<visualization_msgs::MarkerArray>(
      "/perception/fusion/viz_radar_filtered", queueSize);
  publishers_map_["radarfilteredrviz"] = radar_filtered_rviz;
  
  ros::Publisher lane_img_pub = node_ptr_->advertise<sensor_msgs::Image>("/perception/fusion/lane_img", queueSize);
  publishers_map_["lane_img"] = lane_img_pub;

  ros::Publisher ground_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/ground", queueSize);
  publishers_map_["ground"] = ground_pub;

  ros::Publisher camera3d_pub = node_ptr_->advertise<visualization_msgs::MarkerArray>("/perception/fusion/camera3d", queueSize);
  publishers_map_["camera3d"] = camera3d_pub;



  display_ptr_.reset(new RvizDisplay(node_ptr_));
  map_display_ptr_.reset(new RvizDisplay(node_ptr_, ROS_VISUALIZATION_MARKER::LINE_STRIP));
  lane_display_ptr_.reset(new RvizDisplay(node_ptr_, ROS_VISUALIZATION_MARKER::LINE_STRIP, "lane"));
  fov_display_ptr_.reset(new RvizDisplay(node_ptr_, ROS_VISUALIZATION_MARKER::LINE_STRIP, "camera_fov"));
}

bool PerceptionFusionComponent::InitAlgorithmPlugin() {
  fusion_.reset(new fusion::ObstacleMultiSensorFusion());
  fusion::ObstacleMultiSensorFusionParam param;
  param.main_sensors = fusion_main_sensors_;
  param.fusion_method = fusion_method_;
  if (!(fusion_->Init(param))) {
    ROS_ERROR("InitAlgorithmPlugin: Failed to init ObstacleMultiSensorFusion!");
  }

  //  ROS_DEBUG_STREAM("Init algorithm successfully, onboard fusion: " << fusion_method_);
  return true;
}

bool PerceptionFusionComponent::PreProcessLidar(const TrackedObject *lidar_obj, const std::string &sensor_name, const double radius_roi_object_check) {
//  bool is_processed = false;
  double roi_object_check = radius_roi_object_check;
  if (lidar_obj->obj().detect_state() == 1 || lidar_obj->obj().contour_size() < 3) {
    return true;
  }
  double obstacle_dis = std::sqrt(std::pow(lidar_obj->obj().center().x(), 2) +
                                  std::pow(lidar_obj->obj().center().y(), 2));
  if (object_in_roi_check_ && (obstacle_dis > radius_roi_object_check)) {
    return true;
  }
  if (vehicle_ == (int)perception::fusion::VEHICLE::SWEEPER) {
    if (sensor_name == "lidar_zvision") {
      if (lidar_obj->obj().type() != perception::ObjectType::TYPE_UNKNOWN && lidar_obj->obj().center().x() > 20) {
        return true;
      }
    }
  }
  return false;
}

void PerceptionFusionComponent::LidarObjectsCallback(const std_msgs::StringConstPtr& msg) {
  if (v2x_param_.v2x_process_mode == 5) {
    return;
  }  
  TrackedObjects lidar_measurement;
  lidar_measurement.ParseFromString(msg->data);
  std::string lidar_name_measurement = lidar_measurement.sensor_name();
  if(!base::SensorManager::Instance()->GetSensorInfo(lidar_name_measurement, &lidar_sensor_info_)){
    ROS_WARN_STREAM("LidarObjectsCallback: measurement sensor name: "
                    << lidar_name_measurement
                    << " , and get configured sensorinfo Failed: ");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_LidarSensorInfoFlt");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
  }
  fusion::FramePtr frame(new fusion::Frame());
  fusion::PointFCloudPtr attribute_ptr(new fusion::PointFCloud());  // Modify-guoxiaoxiao
  frame->timestamp =
      lidar_measurement.header().stamp().sec() + lidar_measurement.header().stamp().nsec() * 1e-9;
  // Modify(@liuxinyu)
  if (lidar_measurement.objs_size() == 0) {
    ROS_WARN_STREAM("LidarObjectsCallback: Receive lidar objects number is 0.");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_FusedObjctEmptyFlt");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
  }
  ROS_INFO_THROTTLE(1,"LidarObjectsCallback: Receive lidar objects number is %d, time stamp is %lf"
                   ,lidar_measurement.objs_size(),frame->timestamp);

  if (output_fault_messages_.infos_size() > 0) {
    // publish fault message 
    fault_msg_publisher_.publish(publishers_map_["faultmanger"], output_fault_messages_);
    output_fault_messages_.clear_infos();
  }

  // get localization which stamp similiar to lidar measurement-syf
  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR("LidarObjectsCallback: Fail to get localization for lidar  measurement.");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_FusionUnSynchronized");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
    return;
  }
  frame->sensor_info.set_type(lidar_sensor_info_.type());
  frame->sensor_info.set_name(lidar_name_measurement);
  auto lidar_convertor_ = new LidarConvertor();
  std::vector<fusion::ObjectPtr> fused_objects;
  for (size_t i = 0; i < lidar_measurement.objs_size(); ++i) {
    TrackedObject* lidar_obj = lidar_measurement.mutable_objs(i);
    // Modify@jiangnan:filter  falcon_lidar  predict object :
    // 1、prediction object  (STATE_DETECT = 0 ;STATE_PREDICT = 1);
    // 2、 the number of contour points is less than 3;
    if (PreProcessLidar(lidar_obj, lidar_name_measurement, radius_for_roi_object_check_)) continue;
    fusion::ObjectPtr apollo_object(new fusion::Object());
    if (!(lidar_convertor_->Mogo2Fusion(*lidar_obj, localization, apollo_object, BaseConvertor::input_sensor_::use_lidar))) {
      continue;
    }
    perception::Object* object_src = lidar_obj->mutable_obj();
    PointFCloud cloud = ProtoArrayToPointCloud(object_src->lidar_supplement().cloud());
    *attribute_ptr += cloud;
    apollo_object->lidar_supplement.cloud = cloud;
    // Modify @jiangnan : pass through lidar object (the attribute of group)
    if (apollo_object->group_type == 1) {
      fused_objects.emplace_back(apollo_object);
    } else {
      frame->objects.emplace_back(apollo_object);
    }
  }
  ROS_DEBUG_STREAM("LidarObjectsCallback: Convert lidar objects to apollo objects number is "
                   << frame->objects.size());
  frame->lidar_frame_supplement.on_use = true;
  frame->lidar_frame_supplement.cloud_ptr = attribute_ptr;

  Process(frame, &fused_objects);
}

// process similar to lidar
void PerceptionFusionComponent::VidarObjectsCallback(const std_msgs::StringConstPtr& msg) {
  if (v2x_param_.v2x_process_mode == 5) {
    return;
  }
  TrackedObjects vidar_measurement;
  vidar_measurement.ParseFromString(msg->data);
  std::string vidar_name_measurement = vidar_measurement.sensor_name();
  if(!base::SensorManager::Instance()->GetSensorInfo(vidar_name_measurement, &vidar_sensor_info_)){
    ROS_WARN_STREAM("VidarObjectsCallback: measurement sensor name: "
                     << vidar_name_measurement
                    << " , and get configured sensorinfo Failed: ");
    return;
  }
  fusion::FramePtr frame(new fusion::Frame());
  frame->timestamp =
      vidar_measurement.header().stamp().sec() + vidar_measurement.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(5,"VidarObjectsCallback: Receive vidar objects number is %d, time stamp is  %lf"
                   ,vidar_measurement.objs_size(),frame->timestamp);
  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR(
        "VidarObjectsCallback: Fail to get localization for vidar "
        "measurement.");
    return;
  }
  frame->sensor_info.set_type(vidar_sensor_info_.type());
  frame->sensor_info.set_name(vidar_name_measurement);

  perception::base::SetSensorExtrinsics(vidar_sensor_info_.extrinsic(), frame->sensor2world_pose);
  Eigen::Matrix4d world2camera_pose = frame->sensor2world_pose.matrix().inverse();
  auto vidar_convertor_ = new VisualConvertor();

  for (size_t i = 0; i < vidar_measurement.objs_size(); ++i) {
    TrackedObject* vidar_obj = vidar_measurement.mutable_objs(i);
    double obstacle_dis = std::sqrt(std::pow(vidar_obj->obj().center().x(), 2) +
                                    std::pow(vidar_obj->obj().center().y(), 2));
    if (object_in_roi_check_ && (obstacle_dis > radius_for_roi_object_check_))
      continue;

    fusion::ObjectPtr apollo_object(new fusion::Object());
    if (!vidar_convertor_->Mogo2Fusion(*vidar_obj, localization, apollo_object, BaseConvertor::input_sensor_::use_vidar))
      continue;
    frame->objects.emplace_back(apollo_object);
  }
  std::vector<fusion::ObjectPtr> fused_objects;
  if (pub_vidar_)
    visualization_.VidarObjDisplay(frame, publishers_map_["vidarrviz"]);
  Process(frame, &fused_objects);
}

void PerceptionFusionComponent::FalconLidarObjectsCallback(const std_msgs::StringConstPtr& msg) {
  if (v2x_param_.v2x_process_mode == 5) {
    return;
  }
  // add by jiangnan ::Innovusion falcon lidar callback
  TrackedObjects falcon_lidar_measurement;
  falcon_lidar_measurement.ParseFromString(msg->data);
  std::string falcon_lidar_name_measurement = falcon_lidar_measurement.sensor_name();
  if(!base::SensorManager::Instance()->GetSensorInfo(falcon_lidar_name_measurement, &falcon_lidar_info_)){
    ROS_WARN_STREAM("FalconObjectsCallback: measurement sensor name: "
                     << falcon_lidar_name_measurement
                     << " , and get configured sensorinfo Failed: ");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_LidarSensorInfoFlt");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
  }
  fusion::FramePtr frame(new fusion::Frame());
  fusion::PointFCloudPtr attribute_ptr(new fusion::PointFCloud());
  frame->timestamp = falcon_lidar_measurement.header().stamp().sec() +
                     falcon_lidar_measurement.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(1,"FalconLidarObjectsCallback: Receive falcon lidar objects number is %d, time stamp is %lf"
                   ,falcon_lidar_measurement.objs_size(),frame->timestamp);
  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR(
        "FalconLidarObjectsCallback: Fail to get localization for falcon lidar "
        "measurement.");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_FusionUnSynchronized");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
    return;
  }
  frame->sensor_info.set_type(falcon_lidar_info_.type());
  frame->sensor_info.set_name(falcon_lidar_name_measurement);
  auto falcon_convertor_ = new LidarConvertor();

  for (size_t i = 0; i < falcon_lidar_measurement.objs_size(); ++i) {
    TrackedObject* falcon_lidar_obj = falcon_lidar_measurement.mutable_objs(i);
      // Modify@jiangnan :    filter  lidar  predict object   (STATE_DETECT = 0 ;STATE_PREDICT = 1;)
    if (PreProcessLidar(falcon_lidar_obj, falcon_lidar_name_measurement, radius_faclon_for_roi_object_check_)) continue;
    fusion::ObjectPtr apollo_object(new fusion::Object());
 
    if (!(falcon_convertor_->Mogo2Fusion(*falcon_lidar_obj, localization, apollo_object, BaseConvertor::input_sensor_::use_falcon))) {
      continue;
    }

    perception::Object* object_src = falcon_lidar_obj->mutable_obj();
    PointFCloud cloud = ProtoArrayToPointCloud(object_src->falcon_lidar_supplement().cloud());
    *attribute_ptr += cloud;
    apollo_object->falcon_lidar_supplement.cloud = cloud;
    frame->objects.emplace_back(apollo_object);
  }
  ROS_DEBUG_STREAM(
      "FalconLidarObjectsCallback: Convert falcon lidar objects to apollo "
      "objects number is "
      << frame->objects.size());

  //  frame->falcon_lidar_frame_supplement.on_use = true;
  frame->falcon_lidar_frame_supplement.cloud_ptr = attribute_ptr;
  std::vector<fusion::ObjectPtr> fused_objects;
  Process(frame, &fused_objects);
}

void PerceptionFusionComponent::CameraObjectsCallback(const std_msgs::StringConstPtr& msg) {
  if (v2x_param_.v2x_process_mode == 5) {
    return;
  }  
  VisualObjects camera_measurement;
  camera_measurement.ParseFromString(msg->data);
  std::string camera_name_measurement = camera_measurement.sensor_name();
  if(!base::SensorManager::Instance()->GetSensorInfo(camera_name_measurement, &camera_sensor_info_)){
    ROS_WARN_STREAM("CameraObjectsCallback: measurement sensor name: "
                    << camera_name_measurement
                    << " is not Exist! return");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_CameraSensorInfoFlt");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
    return;
  }
  fusion::FramePtr frame(new fusion::Frame());
  frame->timestamp =
      camera_measurement.header().stamp().sec() + camera_measurement.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(5,"CameraObjectsCallback: Receive camera objects number is %d, time stamp is %lf",camera_measurement.objs_size(),frame->timestamp);
  frame->sensor_info.set_type(camera_sensor_info_.type());
  frame->sensor_info.set_name(camera_name_measurement);
  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR(
        "CameraObjectsCallback: Fail to get localization for falcon lidar "
        "measurement.");
    return;
  }
  perception::base::SetSensorExtrinsics(camera_sensor_info_.extrinsic(), frame->sensor2world_pose);
  // Modify-guoxiaoxiao
  Eigen::Matrix4d world2camera_pose = frame->sensor2world_pose.matrix().inverse();
  static bool first_time = true;
  if (first_time) {
    camera602baselink_pose_ = frame->sensor2world_pose.matrix();
    baselink2camera60_pose_ = frame->sensor2world_pose.matrix().inverse();
    c60_model_ = SensorDataManager::Instance()->GetCameraIntrinsicDistortion(camera_sensor_info_.name());
    first_time = false;
    calculateCameraFov();
  }
  auto camera_convertor_ = new  VisualConvertor();

  for (size_t i = 0; i < camera_measurement.objs_size(); ++i) {
    VisualObject camera_obj = camera_measurement.objs(i);
    if (camera_obj.obj().type() == perception::ObjectType::TYPE_SIGN ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_LIGHT ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_RED ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_GREEN ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_YELLOW ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_BLACK ||
        // camera_obj.obj().type() == perception::ObjectType::TYPE_TRIANGLEROADBLOCK ||
        camera_obj.obj().type() == perception::ObjectType::TYPE_WARNINGTRIANGLE) {
      continue;
    }
    fusion::ObjectPtr apollo_object(new fusion::Object());
    if (!camera_convertor_->Mogo2Fusion(camera_obj, localization, apollo_object)) {
      continue;
    }
    frame->objects.emplace_back(apollo_object);
  }
   //使用车道线信息计算计算2dbox深度,并填充
  calculateBoxDepthByLane(localization, frame);
  ROS_DEBUG_STREAM("CameraObjectsCallback: Convert camera objects to apollo objects number is "
                   << frame->objects.size());
  frame->camera_frame_supplement.on_use = true;
    std::vector<fusion::ObjectPtr> fused_objects;
    Process(frame, &fused_objects);
}

void PerceptionFusionComponent::calculateCameraFov() {
  // calculate camera fov
  double angle_horizon = 60/2.0/180.0*PI;
  double angle_upright = 32/2.0/180.0*PI;
  double range_max_camera = 100.0;
  map::V_V_Points fov_vec;
  map::V_Points fov_c60;

  //camera frame fov fit
  map::Point p0(0.0, 0.0, 0.0);
  fov_c60.push_back(p0);

  map::Point p1;
  p1.z = range_max_camera*cos(angle_upright)*cos(angle_horizon);
  p1.x = -range_max_camera*cos(angle_upright)*sin(angle_horizon);
  p1.y = -range_max_camera*sin(angle_upright);
  fov_c60.push_back(p1);

  map::Point p2;
  p2.z = range_max_camera*cos(-angle_upright)*cos(angle_horizon);
  p2.x = -range_max_camera*cos(-angle_upright)*sin(angle_horizon);
  p2.y = -range_max_camera*sin(-angle_upright);
  fov_c60.push_back(p2);

  map::Point p3;
  p3.z = range_max_camera*cos(-angle_upright)*cos(-angle_horizon);
  p3.x = -range_max_camera*cos(-angle_upright)*sin(-angle_horizon);
  p3.y = -range_max_camera*sin(-angle_upright);
  fov_c60.push_back(p3);

  map::Point p4;
  p4.z = range_max_camera*cos(angle_upright)*cos(-angle_horizon);
  p4.x = -range_max_camera*cos(angle_upright)*sin(-angle_horizon);
  p4.y = -range_max_camera*sin(angle_upright);
  fov_c60.push_back(p4);

  //transform to base_link frame下
  auto camera2baselink = [=](map::V_Points& points) {
    for (auto& p : points) {
      Eigen::Vector4d p_camera(p.x, p.y, p.z, 1.0);
      Eigen::Vector4d p_baselink = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(camera602baselink_pose_ * p_camera);//camera frame to base_link
      p.x = p_baselink[0];
      p.y = p_baselink[1];
      p.z = p_baselink[2];
    }
  };
  camera2baselink(fov_c60);

  /*calculate base on base_link
    1           4
    '           '
    '           '
    '           '
    2-----------3
  */

  //calculate between p0p2 and ground
  map::Point p02;
  p02.z = .0;
  p02.y = fov_c60[0].y - fov_c60[0].z*(fov_c60[0].y - fov_c60[2].y)/(fov_c60[0].z - fov_c60[2].z);
  p02.x = fov_c60[0].x - (fov_c60[0].x - fov_c60[2].x)*(fov_c60[0].y - p02.y)/(fov_c60[0].y - fov_c60[2].y);

  //calculate between p0p3 and ground
  map::Point p03;
  p03.z = .0;
  p03.y = fov_c60[0].y - fov_c60[0].z*(fov_c60[0].y - fov_c60[3].y)/(fov_c60[0].z - fov_c60[3].z);
  p03.x = fov_c60[0].x - (fov_c60[0].x - fov_c60[3].x)*(fov_c60[0].y - p03.y)/(fov_c60[0].y - fov_c60[3].y);
  //计算直线p0p1 p0p4在地面的投影, 分别与直线p02p03的交点(不算也可以，直接和p0 p4在xoy上的投影连接起来也可以)

  //组成一个梯形框
  map::V_Points fov_c60_bev;
  fov_c60_bev.push_back(p02);
  fov_c60_bev.push_back(p03);
  fov_c60_bev.push_back(map::Point(fov_c60[4].x, fov_c60[4].y, .0));
  fov_c60_bev.push_back(map::Point(fov_c60[1].x, fov_c60[1].y, .0));
  fov_c60_bev_ = fov_c60_bev;

  //display
  fov_vec.push_back(fov_c60);
  fov_vec.push_back(fov_c60_bev);
  fov_display_ptr_->displayPolyline(fov_vec);
}

void PerceptionFusionComponent::calculateBoxDepthByLane(const localization::Localization& cur_local,
                                                        fusion::FramePtr frame) {
  // 将车道线从utm坐标，转换到camera box当前时刻的base_link坐标系下, 并发布lane marker、lane img、以及计算出来的camera 2dbox的3d信息
  // step1: 将车道线从utm坐标，转换到camera box当前时刻的base_link坐标系下
  if (hadmap_msg_.mutable_map()->roads_size() == 0 ||
      hadmap_msg_.mutable_map()->mutable_roads(0)->sections_size() == 0) {
    ROS_ERROR("PerceptionFusionComponent::calculateBoxDepthByLane: can not get hadmap_msg_.");
    return;
  }
  LaneMarkerDataRecord lanemark;
  ConvertMapMsg2LaneData(hadmap_msg_, lanemark, cur_local);
  lane_display_ptr_->displayPolyline(lanemark);

  // step2:计算车道线投影到img上的像素坐标
  if (c60_model_ == nullptr) {
    ROS_ERROR("PerceptionFusionComponent::calculateBoxDepthByLane: can not get model.");
    return;
  }

  auto baselink2camera = [=](const std::vector<Point2DD>& lane_points, std::vector<cv::Point2d>& lane_img) {
    for (const auto& p : lane_points) {
      Eigen::Vector4d p_baselink(p.x, p.y, 0.0, 1.0);
      // 添加z坐标
      if (use_ground_map_) {
        for(const auto& ground : ground_map_) {//注意这里使用前应该判断下时间戳，防止使用的是过期的地面数据！！！
          if (p.x >= ground.start_ring && p.x < ground.end_ring) {
            p_baselink[2] = (-ground.ground_coe[3] - ground.ground_coe[0]*p.x - ground.ground_coe[1]*p.y) / ground.ground_coe[2];
            break;
          }
        }
      }

      Eigen::Vector4d p_camera = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(baselink2camera60_pose_ * p_baselink);//base_link到camera frame
      Eigen::Vector2f pt_ct = c60_model_->Project(Eigen::Vector3f(p_camera[0], p_camera[1], p_camera[2]));//投影成像素坐标
      cv::Point2d p_img(pt_ct[0], pt_ct[1]);
      lane_img.push_back(p_img);
    }

  };
  // 将所有的点转换到图像坐标系
  std::vector<std::vector<cv::Point2d>> lanes_on_img;
  lanes_on_img.resize(6);
  std::vector<std::vector<Point2DD>*> vector_ptr;
  vector_ptr.resize(6);
  vector_ptr[0] = &lanemark.left.points;
  vector_ptr[1] = &lanemark.left_right.points;
  vector_ptr[2] = &lanemark.left2.points;
  vector_ptr[3] = &lanemark.right.points;
  vector_ptr[4] = &lanemark.right_left.points;
  vector_ptr[5] = &lanemark.right2.points;

  baselink2camera(*vector_ptr[0], lanes_on_img[0]);
  baselink2camera(*vector_ptr[1], lanes_on_img[1]);
  baselink2camera(*vector_ptr[2], lanes_on_img[2]);
  baselink2camera(*vector_ptr[3], lanes_on_img[3]);
  baselink2camera(*vector_ptr[4], lanes_on_img[4]);
  baselink2camera(*vector_ptr[5], lanes_on_img[5]);


  // step3:计算2dbox下边沿角点水平距离最近的车道线对应x值插值
  std::vector<std::vector<double>> dis_x_img;
  double x_min, x_max, y_min, y_max;
  double z_min = .0;
  double img_max_x, img_min_x, img_x_dis, lane_max_y, lane_min_y, lane_y_dis;
  double base_link_x, base_link_y, size_y, size_z;
  double host_yaw_global = cur_local.yaw();
  double cos_host = std::cos(host_yaw_global);
  double sin_host = std::sin(host_yaw_global);

  for (auto& obj : frame->objects) {
    std::vector<std::pair<int, int>> obj_lane_index;
    y_max = obj->camera_supplement.box.ymax;
    x_min = obj->camera_supplement.box.xmin;
    // 3.1先根据角点像素的y坐标，筛选所有车道线的两个点的index，保存起来
    for (int k = 0; k < 6; k++) {
      for (int i = 0; i < lanes_on_img[k].size(); i++) { //从下到上，车道线的y是从大到小的
        if (lanes_on_img[k][i].y < 0) continue;
        if (lanes_on_img[k][i].y < y_max) {
          if (i - 1 >= 0) {
            obj_lane_index.push_back(std::pair<int, int>(k, i-1));
          }
          break;
        }
      }
    }
    if (obj_lane_index.empty() || obj_lane_index.size() == 1) {// size为1时候无法估计出y坐标和宽度高度
      continue;
    }
    // 3.2再依次比较角点像素的x坐标与车道线的关系，可以得到距离最近车道上(像素x坐标)
    double min_diss_x = std::numeric_limits<double>::max();
    std::pair<int, int> cur_lane_index;
    for (const auto& lane_index : obj_lane_index) {
      double dis_x = fabs(lanes_on_img[lane_index.first][lane_index.second].x - x_min);
      if (dis_x < min_diss_x) {
        min_diss_x = dis_x;
        cur_lane_index = lane_index;
      }
    }
    // 3.3得到(x_min, y_max)对应的base_link坐标
    base_link_x = (lanes_on_img[cur_lane_index.first][cur_lane_index.second].y - y_max) /
                  (lanes_on_img[cur_lane_index.first][cur_lane_index.second].y - lanes_on_img[cur_lane_index.first][cur_lane_index.second+1].y) *
                  ((*vector_ptr[cur_lane_index.first])[cur_lane_index.second].x - (*vector_ptr[cur_lane_index.first])[cur_lane_index.second+1].x) +
                  (*vector_ptr[cur_lane_index.first])[cur_lane_index.second].x;

    // 3.4计算base_link_y坐标
    x_max = obj->camera_supplement.box.xmax;
    y_min = obj->camera_supplement.box.ymin;
    img_max_x = lanes_on_img[(obj_lane_index.end()-1)->first][(obj_lane_index.end()-1)->second].x;
    img_min_x = lanes_on_img[obj_lane_index.begin()->first][obj_lane_index.begin()->second].x;
    img_x_dis = img_max_x - img_min_x;
    lane_max_y = (*vector_ptr[obj_lane_index.begin()->first])[obj_lane_index.begin()->second].y;
    lane_min_y = (*vector_ptr[(obj_lane_index.end()-1)->first])[(obj_lane_index.end()-1)->second].y;
    lane_y_dis = lane_max_y - lane_min_y;

    base_link_y = lane_max_y - (x_min - img_min_x) / img_x_dis * lane_y_dis;

    // 3.5计算宽度W和高度H
    size_y = (x_max - x_min) / img_x_dis * lane_y_dis;
    size_z = (y_max - y_min) / (x_max - x_min) * size_y;

    // 3.6计算obj最小z坐标
    if (use_ground_map_) {
      for(const auto& ground : ground_map_) {//注意这里使用前应该判断下时间戳，防止使用的是过期的地面数据！！！
        if (base_link_x >= ground.start_ring && base_link_x < ground.end_ring) {
          z_min = (-ground.ground_coe[3] - ground.ground_coe[0]*base_link_x - ground.ground_coe[1]*base_link_y) / ground.ground_coe[2];
          break;
        }
      }
    }

    // 3.7填充center_ego、size、center、position、polygon
    {
      obj->center_ego << base_link_x, base_link_y-size_y/2.0, z_min+size_z/2.0;
      obj->size << 0.0, size_y, size_z;
      obj->center[0] = obj->center_ego[0] * cos_host - obj->center_ego[1] * sin_host;
      obj->center[1] = obj->center_ego[0] * sin_host + obj->center_ego[1] * cos_host;
      obj->center[2] = obj->center_ego[2];
      obj->position[0] = obj->center[0] + cur_local.position().x();
      obj->position[1] = obj->center[1] + cur_local.position().y();
      obj->position[2] = obj->center[2] + cur_local.position().z();
      obj->camera_supplement.has_3d = true;
    }

    // 3.8结果单独保存用于后续可视化
    std::vector<double> temp = {x_min, y_max, base_link_x, base_link_y, z_min, size_y, size_z, obj->id};
    dis_x_img.push_back(temp);

  }

  // 3d display
  visualization_.Camera3DDisplay(dis_x_img, publishers_map_["camera3d"]);

#if (DEBUG_ROS)
  // display
if ((new_detection_image_ != nullptr) && (!new_detection_image_->image.empty())) {

  // 在图片上画出车道线
  visualization_.LaneImageDisplay(new_detection_image_->image, lanes_on_img, publishers_map_["lane_img"], false);

  // 遍历车道线，将在base_link下x坐标相等的点连线
  std::vector<std::pair<double, std::vector<cv::Point2d>>> lines_on_img;
  for (int i = 0; i < lanemark.left.points.size(); i++) {
    if (i%10 == 0) {
      for (int j = 0; j < lanemark.left2.points.size(); j++) {
        if (fabs(lanemark.left.points[i].x - lanemark.left2.points[j].x) < 0.1) {
          std::vector<cv::Point2d> line;
          line.resize(2);
          line[0] = lanes_on_img[0][i];
          line[1] = lanes_on_img[2][j];

          lines_on_img.push_back(std::pair<double, std::vector<cv::Point2d>>(lanemark.left.points[i].x, line));
          break;
        }
      }
    }
  }
  visualization_.LineImageDisplay(new_detection_image_->image, lines_on_img, publishers_map_["lane_img"], false);

  // 将每个obj的xy坐标画在图片上
  visualization_.PointsImageDisplay(new_detection_image_->image, dis_x_img, publishers_map_["lane_img"], true);
}
#endif
}

void PerceptionFusionComponent::GroundFittingInfoCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  if (!use_ground_map_) {
    return;
  }

  double timestamp;
  if (msg) {
    RosToProto<ground_map::GroundMap>(*msg, ground_map_msg_);//将地面信息保存下来
    ground_map_.clear();
    // 保存下来，后续还需要根据时间戳判断是否保存
    for(int i=0; i< ground_map_msg_.coefficients_size(); i++){
      ground_map::GroundCoeff *tmp_coeff = ground_map_msg_.mutable_coefficients(i);
      GroundCoeff result;
      result.start_ring = static_cast<int>(tmp_coeff->start_ring());
      if (result.start_ring == -1) {
        continue;
      }
      result.end_ring = static_cast<int>(tmp_coeff->end_ring());
      result.ground_coe << tmp_coeff->a(), tmp_coeff->b(), tmp_coeff->c(), tmp_coeff->d();
      result.ground_normal << tmp_coeff->normal_x(), tmp_coeff->normal_y(), tmp_coeff->normal_z();
      ground_map_.push_back(result);
    }

#if (DEBUG_ROS)
    // rviz可视化
  // 每个平面方程是10m*60m的区域，x方向选2m的精度，所以每个平面上选取12个点
  auto addZ = [](std::vector<Point3DD>& points, const GroundCoeff& ground) {
    float a = ground.ground_coe[0];
    float b = ground.ground_coe[1];
    float c = ground.ground_coe[2];
    float d = ground.ground_coe[3];

    for (auto& p : points) {
      double z = (-d - a*p.x - b*p.y) / c;
      p.z = z;
    }

  };

  std::vector<std::vector<Point3DD>> ground_planes;
  int plane_size = ground_map_.size();
  ground_planes.resize(plane_size);
  for (int i = 0; i < plane_size; i++) {
    std::vector<Point3DD> points;
    int start_ring = ground_map_[i].start_ring;
    points.emplace_back(Point3DD(start_ring, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring, 30.0, 0.0));

    points.emplace_back(Point3DD(start_ring + 2.0, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring + 2.0, 30.0, 0.0));

    points.emplace_back(Point3DD(start_ring + 4.0, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring + 4.0, 30.0, 0.0));

    points.emplace_back(Point3DD(start_ring + 6.0, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring + 6.0, 30.0, 0.0));

    points.emplace_back(Point3DD(start_ring + 8.0, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring + 8.0, 30.0, 0.0));

    points.emplace_back(Point3DD(start_ring + 10.0, -30.0, 0.0));
    points.emplace_back(Point3DD(start_ring + 10.0, 30.0, 0.0));

    addZ(points, ground_map_[i]);
    ground_planes.push_back(points);
  }
  visualization_.GroundPlaneDisplay(ground_planes, publishers_map_["ground"]);
#endif
  }
}

void PerceptionFusionComponent::TranformPolygonMap2Baselink(map::V_V_Points& polygons, localization::Localization& localization) {
  Eigen::Matrix4d cur_host_mat;
  transGlobal2VehicleMat(localization, cur_host_mat);
  for(auto& polygon : polygons) {
    for(auto& p : polygon) {
      Eigen::Vector4d map_pose(p.x, p.y, p.z, 1);
      Eigen::Vector4d base_link_pose;
      base_link_pose = cur_host_mat*map_pose;
      p.x = base_link_pose[0];
      p.y = base_link_pose[1];
      p.z = base_link_pose[2];
    }
  }
}

void PerceptionFusionComponent::FilteredBySemanticMap(std::vector<fusion::ObjectPtr>& fused_objects) {

  // 植被类型retype
  for (const fusion::ObjectPtr& fusion_object_ptr : fused_objects) {
    if (fusion_object_ptr->type == perception::fusion::ObjectType::UNKNOWN ||
        fusion_object_ptr->type == perception::fusion::ObjectType::UNKNOWN_MOVABLE ||
        fusion_object_ptr->type == perception::fusion::ObjectType::UNKNOWN_UNMOVABLE) {
      if (IsInVegPolygons(fusion_object_ptr)) {
        fusion_object_ptr->type = perception::fusion::ObjectType::VEGETATION;
        fusion_object_ptr->filtered_by_map = 1;
      }
    }
  }

  // 添加虚拟障碍物-building
  if (!map::GetCurrentBuildingPolygons().empty()) {
    // car bus truck的AI错检纠正，以及unknown retype
    for(auto it=fused_objects.begin(); it!=fused_objects.end();) {
      if ((
              (*it)->type == perception::fusion::ObjectType::UNKNOWN ||
              (*it)->type == perception::fusion::ObjectType::UNKNOWN_MOVABLE ||
              (*it)->type == perception::fusion::ObjectType::UNKNOWN_UNMOVABLE ||
              (*it)->type == perception::fusion::ObjectType::CAR ||
              (*it)->type == perception::fusion::ObjectType::BUS ||
              (*it)->type == perception::fusion::ObjectType::TRUCK) &&
          (IsBuildingObj(*it))) {
        // erase
        it = fused_objects.erase(it);
      } else {
        it++;
      }

    }
    // add, 这个部分放在后面publish地方

  }
}

void PerceptionFusionComponent::HadmapLaneCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {

  double timestamp;
  if (msg) {
    RosToProto<hadmap::MapMsg>(*msg, hadmap_msg_);//将车道线信息保存起来
  }
}

void PerceptionFusionComponent::Front60CameraDetectionCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr cam_image_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!cam_image_ptr->image.empty()) {
      new_detection_image_ = cam_image_ptr;
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

bool PerceptionFusionComponent::IsBuildingObj(const fusion::ObjectPtr& obj_ptr) {
  auto&& polygons = map::GetCurrentBuildingPolygons();
  if (polygons.empty()) {
    return false;
  }
  IOU iou;
  double iou_core = .0;
  for (const auto& polygon : polygons) {
    iou_core += iou.Compute(obj_ptr->polygon_utm, polygon, 4);
    if (iou_core > 0.5) {
      return true;
    }
  }
  return false;
}

bool PerceptionFusionComponent::IsInVegPolygons(const fusion::ObjectPtr& obj_ptr) {
  auto&& veg_polygons = map::GetCurrentVegPolygons();
  if (veg_polygons.empty()) {
    return false;
  }
  IOU iou;
  double iou_core = .0;
  for (const auto& polygon : veg_polygons) {
    iou_core += iou.Compute(obj_ptr->polygon_utm, polygon);
    if (iou_core > 0.5) {
      return true;
    }
  }
  return false;
}

bool IsInCameraFov(PointCloud<PointD> polygon_ego) {
  // 默认在
  if (fov_c60_bev_.empty()) {
    return true;
  }
  IOU iou;
  double iou_core = iou.Compute(polygon_ego, fov_c60_bev_);
  if (iou_core > 0.5) {
    return true;
  }
  return false;
}


bool PerceptionFusionComponent::PostProcessRadar(const RadarObject &radar_object,
                                                 const fusion::ObjectPtr apollo_object) {
  double radar_obj_speed = std::sqrt(std::pow(apollo_object->velocity[0], 2) +
                                     std::pow(apollo_object->velocity[1], 2));
  //process vehicle
  if (radar_object.obj().center().x() > 0) {
    if (radar_obj_speed < 1.5) {
      return true;
    }
  } else {
    if ((radar_obj_speed < 0.5) || ((radar_object.probexist() < 0.95) && (radar_obj_speed < 1.5))) {
      return true;
    }
  }
  return false;
}

void PerceptionFusionComponent::RadarObjectsCallback(const std_msgs::StringConstPtr& msg) {
  if (v2x_param_.v2x_process_mode == 5) {
    return;
  }
  RadarObjects radar_measurement;
  radar_measurement.ParseFromString(msg->data);
  std::string radar_name_measurement = radar_measurement.sensor_name();
  if(!base::SensorManager::Instance()->GetSensorInfo(radar_name_measurement, &radar_sensor_info_)){
      ROS_WARN_STREAM("RadarObjectsCallback: measurement sensor name: "
                     << radar_name_measurement
                     << " , and get configured sensorinfo Failed: ");
    fault_management::FaultInfo *fault_info_ = output_fault_messages_.add_infos();
    fault_info_->set_fault_id("Perception_Fusion_RadarSensorInfoFlt");
    uint64_t fault_time = (uint64_t)(ros::Time::now().toSec() * 1e3);
    fault_info_->set_fault_time(fault_time);
  }
  fusion::FramePtr frame(new fusion::Frame());
  frame->timestamp =
      radar_measurement.header().stamp().sec() + radar_measurement.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(5,"RadarObjectsCallback: Receive radar objects number is %d, time stamp is%f"
                   ,radar_measurement.objs_size(), frame->timestamp);
  // get localization which stamp similiar to lidar measurement-lxy
  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR("RadarObjectsCallback: Fail to get localization for radar measurement.");
    return;
  }
  if (fabs(localization.yaw_v()) > radar_filter_yaw_thr_) {
    return;
  }
  frame->sensor_info.set_type(radar_sensor_info_.type());
  frame->sensor_info.set_name(radar_name_measurement);
  
  auto radar_convertor_ = new RadarConvertor();

  for (size_t i = 0; i < radar_measurement.objs_size(); ++i) {
    RadarObject radar_object = radar_measurement.objs(i);
    if (radar_object.meas_state() != 2 || radar_object.rcs() < -2) {
      continue;
    }
    if (std::fabs(radar_object.obj().velocity().x()) > 100) {
      continue;
    }
    if ((radar_object.probexist() < 0.8) || ((radar_object.probexist() < 0.91) && (radar_object.rcs() < -0.5))) {
      continue;
    }
    double obstacle_dis = std::sqrt(std::pow(radar_object.obj().center().x(), 2) +
                                    std::pow(radar_object.obj().center().y(), 2));
    if (object_in_roi_check_ && (obstacle_dis > radius_for_roi_object_check_)) {
      continue;
    }
    fusion::ObjectPtr apollo_object(new fusion::Object());
    if (!radar_convertor_->Mogo2Fusion(radar_object, localization, apollo_object)) {
      continue;
    }
    // Modify @ jiangnan : filter low_speed radar_obj
    if (PostProcessRadar(radar_object, apollo_object)) continue;

    frame->objects.emplace_back(apollo_object);
  }
  ROS_DEBUG_STREAM("RadarObjectsCallback: Convert radar objects to apollo objects number is "
                   << frame->objects.size());
  if (pub_radar_) {
    visualization_.RadarObjDisplay(frame, publishers_map_["radarfilteredrviz"]);
  }
  frame->radar_frame_supplement.on_use = true;
  std::vector<fusion::ObjectPtr> fused_objects;
  Process(frame, &fused_objects);
}

void PerceptionFusionComponent::ObuObjectsCallback(const std_msgs::StringConstPtr& msg) {
  ObuObjects obu_measurement;
  obu_measurement.ParseFromString(msg->data);
  std::string obu_name_measurement = obu_measurement.sensor_name();
  if (!base::SensorManager::Instance()->GetSensorInfo(obu_name_measurement, &obu_sensor_info_)) {
      ROS_WARN_STREAM("ObuObjectsCallback: measurement sensor name: "
                     << obu_name_measurement
                     << " , and get configured sensorinfo Failed: ");
  }
  fusion::FramePtr frame(new fusion::Frame());
  frame->timestamp =
      obu_measurement.header().stamp().sec() + obu_measurement.header().stamp().nsec() * 1e-9;
  ROS_DEBUG_STREAM("ObuObjectsCallback: Receive obu objects number is "
                   << obu_measurement.objs_size() << ", time stamp is " << std::setprecision(18)
                   << frame->timestamp);

  localization::Localization localization;
  if (!QueryNearestLocalization(frame->timestamp, localization)) {
    ROS_ERROR("ObuObjectsCallback: Fail to get localization for obu message timestamp.");
    return;
  }
  while (obu_objects_v2n_.size() >= 40) {
    obu_objects_v2n_.pop_front();
  }  // 20hz obu_objects , buffer for 2 seconds
  while (obu_objects_v2i_.size() >= 40) {
    obu_objects_v2i_.pop_front();
  }  // 20hz obu_objects , buffer for 2 seconds
  auto obu_convertor_ = new ObuConvertor();
  // 缓存v2n和v2i数据，支持不同模式
  obu_convertor_->StoreV2xObjs(obu_measurement, localization, global_localizations_, obu_objects_v2n_, obu_objects_v2i_);
  if (pub_obu_) {
    visualization_.ObuTmpObjDisplay(obu_measurement, publishers_map_["torviz_obutmp"]);
    visualization_.ObuRTEDisplay(obu_measurement, localization, publishers_map_["torviz_oburte"]);
  }
  // 全融合模式 || 纯路侧控车模式 
  if (v2x_param_.v2x_process_mode == 1 || v2x_param_.v2x_process_mode == 5) {
    frame->sensor_info.set_type(obu_sensor_info_.type());
    frame->sensor_info.set_name(obu_name_measurement);
    for (size_t i = 0; i < obu_measurement.objs_size(); ++i) {
      ObuObject obu_object = obu_measurement.objs(i);
      if (IsPassThrough(obu_object)) {
        continue;
      }
      if (v2x_param_.v2x_process_mode == 1 &&
          obu_object.obj().type() != perception::TYPE_PEDESTRIAN &&
          obu_object.obj().type() != perception::TYPE_CAR) {
        continue;
      }
      // 2023.8 poscon<= 5（越小越好，0无效）spd ped <= 2 car <= 3 （越小越好，0无效）
      //  (obu_object.obj().type() == perception::TYPE_PEDESTRIAN &&
      //      obu_object.obj().obu_supplement().velocity_confidence() > 2) ||
      //     (obu_object.obj().type() == perception::TYPE_CAR &&
      //      obu_object.obj().obu_supplement().velocity_confidence() > 3) ||
      if (v2x_param_.v2x_process_mode == 1 &&
              obu_object.obj().obu_supplement().position_confidence() > 5 ||
          obu_object.obj().obu_supplement().position_confidence() - 0 < EPSILON ||
          obu_object.obj().obu_supplement().velocity_confidence() - 0 < EPSILON) {
        ROS_WARN_STREAM(
            "ObuObjectsCallback: The confidence level is not satisfied. "
            << "vel confidence: " << obu_object.obj().obu_supplement().velocity_confidence()
            << " pos confidence: " << obu_object.obj().obu_supplement().position_confidence());
        continue;
      }
      if (v2x_param_.enable_only_pub_zombie && obu_object.status_duration() == DURATION_DEFAULT) {
        continue;
      }
      fusion::ObjectPtr apollo_object(new fusion::Object());
      if (!obu_convertor_->Mogo2Fusion(obu_object, localization, apollo_object)) continue;
      frame->objects.emplace_back(apollo_object);
    }
    ROS_INFO_STREAM("ObuObjectsCallback: Convert obu objects to apollo objects number is "
                     << frame->objects.size());
    if (pub_obu_) {
      visualization_.ObuObjDisplay(frame, publishers_map_["torviz_obu"]);
    }
    frame->obu_frame_supplement.on_use = true;
    std::vector<fusion::ObjectPtr> fused_objects;
    Process(frame, &fused_objects);
  }
}

// transform from radar to vehicle frame temporarily (later moved to radar)- syf
void PerceptionFusionComponent::TransformToBaseLink(perception::RadarObject& obj_in,
                                                    perception::RadarObject& obj_out,
                                                    const Eigen::Affine3d radar_T_base) {
  // obj_out.CopyFrom(obj_in);
  obj_out = obj_in;

  Eigen::Matrix3d radar_r_base = radar_T_base.rotation();
  Eigen::Vector3d radar_t_base = radar_T_base.translation();
  // position transform
  Eigen::Vector3d pos_radar;
  pos_radar << obj_in.obj().center().x(), obj_in.obj().center().y(), obj_in.obj().center().z();

  Eigen::Vector3d pos_FLU = radar_r_base * pos_radar + radar_t_base;
  obj_out.mutable_obj()->mutable_center()->set_x(pos_FLU(0));
  obj_out.mutable_obj()->mutable_center()->set_y(pos_FLU(1));
  obj_out.mutable_obj()->mutable_center()->set_z(pos_FLU(2));

  // angle-yaw transform
  //  Eigen::Vector3d eulerAngle = radar_r_base.eulerAngles(2, 1, 0);
  //  obj_in_FLU->set_angle(obj_in.angle() + eulerAngle(2));

  // velocity transform
  Eigen::Vector3d vel_radar;
  vel_radar << obj_in.obj().velocity().x(), obj_in.obj().velocity().y(),
      obj_in.obj().velocity().z();
  Eigen::Vector3d vel_FLU = radar_r_base * vel_radar;
  obj_out.mutable_obj()->mutable_velocity()->set_x(vel_FLU(0));
  obj_out.mutable_obj()->mutable_velocity()->set_y(vel_FLU(1));
  obj_out.mutable_obj()->mutable_velocity()->set_z(vel_FLU(2));

  // acceleration transform
  Eigen::Vector3d acc_radar;
  acc_radar << obj_in.obj().acceleration().x(), obj_in.obj().acceleration().y(),
      obj_in.obj().acceleration().z();
  Eigen::Vector3d acc_FLU = radar_r_base * acc_radar;
  obj_out.mutable_obj()->mutable_acceleration()->set_x(acc_FLU(0));
  obj_out.mutable_obj()->mutable_acceleration()->set_y(acc_FLU(1));
  obj_out.mutable_obj()->mutable_acceleration()->set_z(acc_FLU(2));
}

bool PerceptionFusionComponent::IsPassThrough(ObuObject obu_object) {
  pub_v2n_mutex_.lock();
  bool pub_v2n_to_pnc = pub_v2n_to_pnc_;
  pub_v2n_mutex_.unlock();

  pub_v2i_mutex_.lock();
  bool pub_v2i_to_pnc_app = pub_v2i_to_pnc_app_;
  pub_v2i_mutex_.unlock();
  /* 1，V2N_RSI 不进全融合 return true
     2，V2N_RSM 若不给PNC，则无需进融合处理，后续添加给鹰眼显示即可 return true
     3，V2I 所有若不给PNC+APP，则无需进融合处理 return true
  */
  if (obu_object.source() == perception::ObjectSource::V2N_RSI) {
    return true;
  } else if (obu_object.source() == perception::ObjectSource::V2N_RSM) {
    if (!pub_v2n_to_pnc) {
      return true;
    }
  } else {
    if (!pub_v2i_to_pnc_app) {
      return true;
    }
  }
  if (obu_object.obj().center().x() == 0 && obu_object.obj().center().y() == 0) {
    return true;
  }
  if (abs(obu_object.obj().center().x()) < v2x_param_.radius_for_fusion_object &&
      abs(obu_object.obj().center().y()) < v2x_param_.radius_for_fusion_object) {
    return true;
  }
  return false;
}

void PerceptionFusionComponent::InitV2xParam() {
  ConfigManager::Instance()->get_value("v2x_process_mode", &v2x_param_.v2x_process_mode);
  ConfigManager::Instance()->get_value("pass_through_mode", &v2x_param_.pass_through_mode);
  ConfigManager::Instance()->get_value("beyond_vis_range", &v2x_param_.beyond_vis_range);
  ConfigManager::Instance()->get_value("radius_for_fusion_object", &v2x_param_.radius_for_fusion_object);
  ConfigManager::Instance()->get_value("zombie_thr", &v2x_param_.zombie_thr); 
  ConfigManager::Instance()->get_value("enable_only_pub_zombie", &v2x_param_.enable_only_pub_zombie);
}

void PerceptionFusionComponent::Process(const fusion::FrameConstPtr& frame,
                                        std::vector<fusion::ObjectPtr>* objects) {
  fusion_->Process(frame, objects);
  if (objects == nullptr) {
    ROS_ERROR("Process: Fused objects pointer ERROR.");
    return;
  }
  if (objects->size() == 0) {
    ROS_DEBUG("Process: Fused objects Empty!");
  }
  // 纯路侧控车模式，主传感器已改为obu，故不需要判断当前帧是不是Lidar Frame
  if (v2x_param_.v2x_process_mode != 5) {
    if (SensorDataManager::Instance()->IsLidar(frame)) {
      PublishFusedObjects(frame, *objects);
    }
  } else {
    PublishFusedObjects(frame, *objects);
    ROS_DEBUG_STREAM("Process: Publish Object Size: " << objects->size());
  }
}

void PerceptionFusionComponent::PublishFusedObjects(
    const fusion::FrameConstPtr& frame,
    const std::vector<fusion::ObjectPtr>& fused_objects) {
  TrackedObjects output_objects;
  ros::Time ts;
  localization::Localization localization;
  ts.fromSec(frame->timestamp);
  double ts_fus = ts.sec + ts.nsec * 1e-9;
  if (!QueryNearestLocalization(ts_fus, localization)) {
    ROS_ERROR("PublishFusedObjects: Fail to get localization for Fused Objects.");
    return;
  }
  pub_v2n_mutex_.lock();
  bool pub_v2n_to_pnc = pub_v2n_to_pnc_;
  pub_v2n_mutex_.unlock();
  pub_v2i_mutex_.lock();
  bool pub_v2i_to_pnc_app = pub_v2i_to_pnc_app_;
  pub_v2i_mutex_.unlock();
  auto output_convertor_ = new OutputConvertor();
  auto obu_convertor_ = new ObuConvertor();
  output_convertor_->Fusion2Mogo(frame, fused_objects, output_objects, localization);
  global_postprocessor_.ComputeGlobalState(output_objects, localization);
  // obu_test
  if (!obu_objects_v2n_.empty() && pub_v2n_to_pnc) {
    obu_convertor_->AddObuToPnc(obu_objects_v2n_, localization, output_objects, v2x_param_);
  }
  /* v2x_param中参数控制v2i处理模式：2-盲区/3-超视距 */
  if (!obu_objects_v2i_.empty() &&
      (v2x_param_.v2x_process_mode == 2 || v2x_param_.v2x_process_mode == 3) &&
      pub_v2i_to_pnc_app) {
    obu_convertor_->AddObuToPnc(obu_objects_v2i_, localization, output_objects, v2x_param_);
  }
  // Modify(@liuxinyu): 增加僵尸车处理
  event_processor_.ZombieCarProcessor(output_objects, v2x_param_.zombie_thr);
  event_processor_.V2nEventProcessor(output_objects);

  output_convertor_->AddVirtualObj(output_objects, virtual_objects, localization);
  if (enable_publish_planning_) {
    proto_msg_publisher_.publish(publishers_map_["toplanning"] , output_objects);
  }
  /* pub_v2n_to_pnc控制是否将v2n障碍物发给PNC */
  if (!obu_objects_v2n_.empty() && !pub_v2n_to_pnc) {
    obu_convertor_->AddObuToPnc(obu_objects_v2n_, localization, output_objects, v2x_param_);
    event_processor_.V2nEventProcessor(output_objects);
  }
  /* v2x_param中参数控制v2i处理模式：4-纯透传模式 */
  if (!obu_objects_v2i_.empty() && v2x_param_.v2x_process_mode == 4 && pub_v2i_to_pnc_app) {
    obu_convertor_->AddObuToPnc(obu_objects_v2i_, localization, output_objects, v2x_param_);
  }

  TrackedObjects output_objects_obu = output_objects;
  mogo::telematics::pad::TrackedObjects output_objects_app;
  std::vector<fusion::ObjectPtr> filter_objects_app;
  obu_convertor_->Mogo2OBU(output_objects, output_objects_obu);
  output_convertor_->OverlapObjectFilter(fused_objects, filter_objects_app);
  output_convertor_->Fusion2App(frame, filter_objects_app, output_objects_app, localization,
                                v2x_param_.zombie_thr);
  // Modify(@liuxinyu):  obu obs add to app
  int done_num = fused_objects.size();
  object_filter_.ObjectsToApp(output_objects, output_objects_app, ts_fus, done_num);
  if (!release_mode_) {
    display_ptr->display(output_objects, output_objects, localization);
  }
  proto_msg_publisher_app.publish(publishers_map_["toapp"], output_objects_app);
  proto_msg_publisher_.publish(publishers_map_["toall_obu"], output_objects_obu);
  ROS_INFO_THROTTLE(1,"Publish fused object:%d,timestamp: %lf",output_objects.objs_size(),frame->timestamp);
}

void PerceptionFusionComponent::AddEgoCarFromLocalization(
    const localization::Localization localization,
    TrackedObjects& tracked_objects) {
  // TrackedObjects output_objects_with_ego = output_objects_;
  perception::TrackedObject* tracked_object_ptr = tracked_objects.add_objs();
  // track_object fill
  {
    tracked_object_ptr->set_longitude(localization.longitude());
    tracked_object_ptr->set_latitude(localization.latitude());
    tracked_object_ptr->set_alt(localization.altitude());

    tracked_object_ptr->set_longitude_p(localization.position().x());
    tracked_object_ptr->set_latitude_p(localization.position().y());

    tracked_object_ptr->mutable_velocity()->set_x(localization.longitudinal_v());
    tracked_object_ptr->mutable_velocity()->set_y(localization.lateral_v());
    tracked_object_ptr->set_absolute_longitude_v(localization.longitudinal_v());
    tracked_object_ptr->set_absolute_lateral_v(localization.lateral_v());
    tracked_object_ptr->set_yaw(localization.yaw());
  }

  perception::Object* obj_ptr = tracked_object_ptr->mutable_obj();
  {
    obj_ptr->set_id(-1);  // set id -1 to uint32 obj id

    obj_ptr->mutable_size()->set_x(5.0);
    obj_ptr->mutable_size()->set_y(2.0);
    obj_ptr->mutable_size()->set_z(2.0);

    // not add for app display - syf
    //  AddContours(obj_ptr);

    obj_ptr->set_type(perception::ObjectType::TYPE_CAR);
    obj_ptr->set_confidence(1.0);
    obj_ptr->set_tracking_time(999.0);

    obj_ptr->set_x_distance(0.0);
    obj_ptr->set_y_distance(0.0);
    obj_ptr->mutable_center()->set_x(0.0);
    obj_ptr->mutable_center()->set_y(0.0);
    obj_ptr->mutable_center()->set_z(0.0);
  }
}

void PerceptionFusionComponent::LocalizationCallback(
    const autopilot_msgs::BinaryDataConstPtr& msg) {
  localization::Localization global_localization;
  RosToProto(*msg, global_localization);
  static double last_ts = 0.0;
  double localization_ts = global_localization.header().stamp().sec() +
                           global_localization.header().stamp().nsec() * 1e-9;
  double ts_dis = localization_ts - last_ts;
  if (last_ts != 0 && ts_dis > 1.0)
    ROS_WARN("Localization message time out!");
  last_ts = localization_ts;

  while (global_localizations_.size() >= 200)
    global_localizations_.pop_front();

  if (global_localization.utm_zone() != 0)
    UTM_ZONE = std::to_string(global_localization.utm_zone());
  else
    ROS_ERROR("LocalizationCallback: Localization utm_zone zero error!");
  global_localizations_.push_back(global_localization);
}

bool PerceptionFusionComponent::QueryNearestLocalization(const double& timestamp,
                                                  localization::Localization& localization) {
  if (global_localizations_.empty()) {
    ROS_ERROR("QueryNearestLocalization: Localization message NOT received.");
    return false;
  }
  // reduce timestamp by time delay of sensor data transmission and perception
  // consuming. now 0.0
  double stamp = timestamp - 0.0;
  double loc_ts;
  for (auto it = global_localizations_.begin(); it != global_localizations_.end(); it++) {
    localization = *it;
    loc_ts = localization.header().stamp().sec() + localization.header().stamp().nsec() * 1e-9;
    if (loc_ts < stamp)
      continue;
    else
      break;
  }

  if (abs(stamp - loc_ts) > 0.3) {
    ROS_ERROR_STREAM("QueryNearestLocalization: Time distance "
                     << std::setprecision(3) << abs(stamp - loc_ts)
                     << " between fusion objects: " << std::setprecision(18) << stamp
                     << " and localization: " << std::setprecision(18) << loc_ts
                     << " is too long.");
    return false;
  }
  return true;
}

// Modify-baihaijiang
void PerceptionFusionComponent::MapConvexhullCallback(
    const autopilot_msgs::BinaryDataConstPtr& msg) {
  static double last_ts = 0.0;
  double ts_dis = msg->header.stamp.toSec() - last_ts;
  if (last_ts != 0 && ts_dis > 1.0)
    ROS_WARN("MapConvexhullCallback: map convexhull message time out!");
  last_ts = msg->header.stamp.toSec();

  while (map_convexhulls_.size() >= 20)  // 10hz map_convexhull , buffer for 2 seconds
    map_convexhulls_.pop_front();

  hadmap::MapConvexhull map_convexhull;
  RosToProto<hadmap::MapConvexhull>(*msg, map_convexhull);
  map_convexhulls_.push_back(map_convexhull);
}

bool PerceptionFusionComponent::updateObjToMapConvexhull(const double& timestamp,
                                                         hadmap::MapConvexhull& map_convexhull) {
  if (map_convexhulls_.empty()) {
    ROS_ERROR("updateObjToMapConvexhull: map convexhull message NOT received.");
    return false;
  }
  // reduce timestamp by time delay of sensor data transmission and perception
  // consuming. now 0.0
  double stamp = timestamp - 0.0;
  double convexhull_ts;
  for (auto it = map_convexhulls_.begin(); it != map_convexhulls_.end(); it++) {
    map_convexhull = *it;
    convexhull_ts =
        map_convexhull.header().stamp().sec() + map_convexhull.header().stamp().nsec() * 1e-9;
    if (convexhull_ts < stamp)
      continue;
    else
      break;
  }
  if (abs(stamp - convexhull_ts) > 0.3) {
    // ROS_ERROR_STREAM("Time distance " << std::setprecision(3) << abs(stamp -
    // convexhull_ts) << " between objects: " << std::setprecision(18)
    //     << stamp << " and map convexhull: " << std::setprecision(18) <<
    //     convexhull_ts << " is too long.");
    return false;
  }
  return true;
}

void PerceptionFusionComponent::ParamSetCmdCallback(const autopilot_msgs::BinaryDataConstPtr &msg) {
  if (msg == nullptr)
    return;
  mogo::telematics::ParamSetCmd param_set_cmd;
  common::DeserializeProto(param_set_cmd, msg->data);
  // V2N
  if (param_set_cmd.type() == mogo::telematics::ParamSetTypeV2N &&
      pub_v2n_to_pnc_ != param_set_cmd.boolvalue()) {
    if (param_set_cmd.boolvalue()) {
      ROS_INFO("\033[32m =======>> Publish V2N to PnC! <<======= \033[0m");
    } else {
      ROS_INFO("\033[32m =======>> Donot Publish V2N to PnC! <<======= \033[0m");
    }
    pub_v2n_mutex_.lock();
    pub_v2n_to_pnc_ = param_set_cmd.boolvalue();
    pub_v2n_mutex_.unlock();
  }
  // V2I
  if (param_set_cmd.type() == mogo::telematics::ParamSetTypeV2I &&
      pub_v2i_to_pnc_app_ != param_set_cmd.boolvalue()) {
    if (param_set_cmd.boolvalue()) {
      ROS_INFO("\033[32m =======>> Publish V2I to PnC and APP! <<======= \033[0m");
    } else {
      ROS_INFO("\033[32m =======>> Donot Publish V2I to PnC and APP! <<======= \033[0m");
    }
    pub_v2i_mutex_.lock();
    pub_v2i_to_pnc_app_ = param_set_cmd.boolvalue();
    pub_v2i_mutex_.unlock();
  }
}

bool PerceptionFusionComponent::GetV2XParamThread() {
  ROS_INFO("GetV2XParamThread: Start v2x param thread!");
  param_thread_.reset(new std::thread(&PerceptionFusionComponent::GetV2XParam, this));
  return true;
}

void PerceptionFusionComponent::GetV2XParam() {
  while (ros::ok()) {
    // v2n to pnc
    bool v2n_to_pnc = false;
    if (ros::param::get("/telematics/fusion/v2n_flag", v2n_to_pnc)) {
      if (pub_v2n_to_pnc_ != v2n_to_pnc) {
        if (v2n_to_pnc) {
          ROS_INFO("\033[32m =======>> Publish V2N to PnC! <<======= \033[0m");
        } else {
          ROS_INFO("\033[32m =======>> Donot Publish V2N to PnC! <<======= \033[0m");
        }
        pub_v2n_mutex_.lock();
        pub_v2n_to_pnc_ = v2n_to_pnc;
        pub_v2n_mutex_.unlock();
      }
    }

    // v2i to pnc
    bool v2i_to_pnc_app = false;
    if (ros::param::get("/telematics/fusion/v2i_flag", v2i_to_pnc_app)) {
      if (pub_v2i_to_pnc_app_ != v2i_to_pnc_app) {
        if (v2i_to_pnc_app) {
          ROS_INFO("\033[32m =======>> Publish V2I to PnC and APP! <<======= \033[0m");
        } else {
          ROS_INFO("\033[32m =======>> Donot Publish V2I to PnC and APP! <<======= \033[0m");
        }
        pub_v2i_mutex_.lock();
        pub_v2i_to_pnc_app_ = v2i_to_pnc_app;
        pub_v2i_mutex_.unlock();
      }
    }

    // v2i fusion mode
    int v2x_fusion_mode = 1;
    if (ros::param::get("/telematics/fusion/fusion_mode", v2x_fusion_mode)) {
      if (v2x_param_.v2x_process_mode != v2x_fusion_mode) {
        v2x_param_.v2x_process_mode = v2x_fusion_mode;
        switch (v2x_param_.v2x_process_mode) {
          case 1:
            ROS_INFO("\033[32m =======>> 1: V2I Fusion! <<======= \033[0m");
            break;
          case 2:
            ROS_INFO("\033[32m =======>> 2: V2I Blind! <<======= \033[0m");
            break;
          case 3:
            ROS_INFO("\033[32m =======>> 3: V2I Beyond Range! <<======= \033[0m");
            break;
          case 4:
            ROS_INFO("\033[32m =======>> 4: V2I Pass Through! <<======= \033[0m");
            break;
          case 5:
            ROS_INFO("\033[32m =======>> 5: V2I Only! <<======= \033[0m");
            break;
        }
        if (v2x_param_.v2x_process_mode == 5) {
          fusion_main_sensors_.clear();
          fusion_main_sensors_.push_back("obu");
        } else {
          fusion_main_sensors_.clear();
          fusion_main_sensors_ = fusion_main_sensors_bk_;
        }
        // init algorithm plugin
        if (!(InitAlgorithmPlugin())) {
          ROS_ERROR("GetV2XParam: Failed to init algorithm plugin.");
        }
      }
    }
    sleep(5);
  }
}

}  // namespace fusion
}  // namespace perception
