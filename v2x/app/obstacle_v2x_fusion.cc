#include "obstacle_v2x_fusion.h"

#include <ctype.h>

#include <fstream>

#define DEBUG 0
namespace perception {
namespace v2x {

V2xFusion::V2xFusion(const std::shared_ptr<ros::NodeHandle>& node) : node_(node) {
  V2xConfigManager::Instance()->Init(FLAGS_v2x_fusion_config_manager_path);
  self_veh_ident_ptr_.reset(new perception::v2x::SelfVehIdentIOU());
  self_veh_ident_ptr_->SetParam();
  V2xConfigManager::Instance()->get_value("use_filter_ego_car", &use_filter_ego_car_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfStoreV2ISSM", &TimeCheckOfStoreV2ISSM_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfStoreV2IRSM", &TimeCheckOfStoreV2IRSM_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfStoreV2NRSM", &TimeCheckOfStoreV2NRSM_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfStoreV2NRSI", &TimeCheckOfStoreV2NRSI_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfStoreV2VBSM", &TimeCheckOfStoreV2VBSM_);
  V2xConfigManager::Instance()->get_value("TimeCheckOfLocalization", &TimeCheckOfLocalization_);
  V2xConfigManager::Instance()->get_value("pub_v2x_rviz", &pub_v2x_rviz_);
}

bool V2xFusion::Init() {
  ros::NodeHandle pn("~");
  V2xFusionInitOptions options;
  ROS_DEBUG_STREAM("read config from: " << FLAGS_v2x_fusion_conf_path);
  if (!::common::GetProtoFromFile(FLAGS_v2x_fusion_conf_path, options)) {
    ROS_ERROR("Init: failed to load config file.");
    return false;
  }

  pn.param("use_time_filter", use_time_filter_, true);

  ::common::Binary::SetName(Name());

  ROS_DEBUG_STREAM("use bsm: " << options.use_bsm());
  ROS_DEBUG_STREAM("use rsm: " << options.use_rsm());
  ROS_DEBUG_STREAM("use_v2n_rsm: " << options.use_v2n_rsm());
  ROS_DEBUG_STREAM("use_v2n_rsi: " << options.use_v2n_rsi());
  ROS_DEBUG_STREAM("use_ssm: " << options.use_ssm());
  ROS_DEBUG_STREAM("use_v2v_ssm: " << options.use_v2v_ssm());
  ROS_DEBUG_STREAM("input source size: " << options.input_source_size());
  ROS_DEBUG_STREAM("v2x obstacles topic name: " << options.output_topic());
  ROS_DEBUG_STREAM("localization topic name: " << options.localization_topic());

  int queueSize = 20;
  localization_subscriber_ =
      pn.subscribe(options.localization_topic(), queueSize, &V2xFusion::LocalizationCallback, this);
  v2x_objects_pub_ = node_->advertise<std_msgs::String>(options.output_topic(), queueSize);
  ros::Subscriber sub_v2x_frame;

  perception::v2x::InputSource input_source;
  for (size_t indx = 0; indx < options.input_source_size(); indx++) {
    input_source = options.input_source(indx);
    ROS_DEBUG_STREAM("input_source.source_name()=" << input_source.source_name());
    if (options.use_bsm() && "bsm" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::BsmCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_bsm_ = true;
    } else if (options.use_rsm() && "rsm" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::V2iRsmCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_rsm_ = true;
    } else if (options.use_v2n_rsm() && "v2n_rsm" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::V2nRsmCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_v2n_rsm_ = true;
    } else if (options.use_ssm() && "ssm" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::SsmCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_v2i_ssm_ = true;
    } else if (options.use_v2n_rsi() && "v2n_rsi" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::V2nRsiCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_v2n_rsi_ = true;
    } else if (options.use_v2v_ssm() && "v2v_ssm" == input_source.source_name()) {
      ros::Subscriber sub_v2x_frame =
          pn.subscribe(input_source.topic(), queueSize, &V2xFusion::SsmCallback, this);
      ROS_DEBUG_STREAM("subscriber topic: " << input_source.topic());
      v2x_subscribers_.push_back(sub_v2x_frame);
      pub_v2v_ssm_ = false;
    }
  }

  if (pub_v2x_rviz_) {
    v2x_rte_rviz_pub_ = pn.advertise<visualization_msgs::MarkerArray>(
        "/perception/v2x/v2x_rte_rviz", queueSize);
    v2x_pnt_rviz_pub_ = pn.advertise<visualization_msgs::MarkerArray>(
        "/perception/v2x/v2x_pnt_rviz", queueSize);
  }

  return true;
}

void V2xFusion::BsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  ::common::BSM_PB bsm_measurement;
  V2xRosToProto(*msg, bsm_measurement);
  ROS_DEBUG_STREAM(bsm_measurement.DebugString());
#if DEBUG
  printf("\033[31m vehid is %s.\033[0m\n", bsm_measurement.bsmframe().id().str_value().c_str());
  // bsm_measurement.bsmframe().secmark());
#endif
  const std::string zero_id = "00000000";
  if (bsm_measurement.bsmframe().id().str_value().c_str() == zero_id) {
    return;
  }
  if (!IsHex(bsm_measurement.bsmframe().id().str_value().c_str())) {
    ROS_WARN_STREAM(
        "BsmCallback: id is't hex: " << bsm_measurement.bsmframe().id().str_value().c_str());
    return;
  }
  const int bsm_id = stoi(bsm_measurement.bsmframe().id().str_value(), 0, 16);
#if DEBUG
  // printf("\033[31m bsm_id is %d.\033[0m\n", bsm_id);
#endif
  // update or add
  std::unordered_map<int, ::common::BSM_PB>::iterator iter = bsm_objects_.find(bsm_id);
  std::unique_lock<std::mutex> lock(bsm_mutex_, std::defer_lock);
  lock.lock();
  if (iter != bsm_objects_.end()) {
    bsm_objects_[bsm_id] = bsm_measurement;
  } else {
    bsm_objects_.insert(std::make_pair(bsm_id, bsm_measurement));
  }
  lock.unlock();
  ROS_INFO_STREAM("BsmCallback: bsm_objects_.size: " << bsm_objects_.size());
}

void V2xFusion::V2iRsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  ::common::RSM_PB rsm_measurement;
  V2xRosToProto(*msg, rsm_measurement);
  ROS_DEBUG_STREAM(rsm_measurement.DebugString());
  double rsu_time =
      rsm_measurement.header().stamp().sec() + rsm_measurement.header().stamp().nsec() * 1e-9;
  double system_time = ros::Time::now().toSec();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ROS_DEBUG_STREAM("V2iRsmCallback: system time_diff = "
                   << std::setprecision(18)
                   << system_time - rsu_time
                   << ", between system_time:" << system_time
                   << " and cloud_time:" << rsu_time);
  ROS_DEBUG_STREAM("V2iRsmCallback: now time = " << std::setprecision(18) << tv.tv_sec + tv.tv_usec * 1e-6);
  localization::Localization localization;
  // temporarity - current time diff threshold is 0.1
  if (!QueryNearestLoc(rsu_time, localization)) {
    ROS_WARN("V2iRsmCallback: Fail to get localization for obu message timestamp.");
    return;
  }
  const int obj_size = rsm_measurement.rsmframe().participants().participantdata_size();
  for (size_t i = 0; i < obj_size; ++i) {
    ::common::ParticipantData_PB Pnt_object =
        rsm_measurement.rsmframe().participants().participantdata(i);

    ::common::RSM_PNT_PB obj_tmp;
    ros::Time ts;
    ts.fromSec(rsu_time);
    obj_tmp.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    obj_tmp.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    ::common::ParticipantData_PB* pnt_obj = obj_tmp.mutable_participantdata();
    pnt_obj->CopyFrom(Pnt_object);

    const int rsm_id = Pnt_object.ptcid();
    std::unordered_map<int, ::common::RSM_PNT_PB>::iterator iter = rsm_objects_.find(rsm_id);
    std::unique_lock<std::mutex> lock(v2x_rsm_mutex_, std::defer_lock);
    lock.lock();
    if (iter != rsm_objects_.end()) {
      rsm_objects_[rsm_id] = obj_tmp;
    } else {
      rsm_objects_.insert(std::make_pair(rsm_id, obj_tmp));
    }
    lock.unlock();
  }
  ROS_INFO_STREAM("V2iRsmCallback: rsm_objects_.size: " << rsm_objects_.size());
}

void V2xFusion::SsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  ::common::SSM_PB ssm_measurement;
  V2xRosToProto(*msg, ssm_measurement);
  ROS_INFO_STREAM(ssm_measurement.DebugString());
  const double ssm_time =
      ssm_measurement.header().stamp().sec() + ssm_measurement.header().stamp().nsec() * 1e-9;
  double system_time = ros::Time::now().toSec();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ROS_DEBUG_STREAM("SsmCallback: system time_diff = " << std::setprecision(18) << system_time - ssm_time
                                        << ", between system_time:" << system_time
                                        << " and cloud_time:" << ssm_time);
  ROS_DEBUG_STREAM("SsmCallback: now time = " << std::setprecision(18) << tv.tv_sec + tv.tv_usec * 1e-6);
  localization::Localization localization;
  if (!QueryNearestLoc(ssm_time, localization)) {
    ROS_WARN("SsmCallback: Fail to get localization for obu message timestamp.");
    return;
  }
  // partcipants
  const int pnt_size = ssm_measurement.ssmframe().participants().list_size();
  for (size_t i = 0; i < pnt_size; i++) {
    ::SensorSharing_DetectedPTCData_PB Pnt = ssm_measurement.ssmframe().participants().list(i);
    ::common::SSM_PNT_PB pnt_tmp;
    ros::Time ts;
    ts.fromSec(ssm_time);
    pnt_tmp.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    pnt_tmp.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    ::SensorSharing_DetectedPTCData_PB* pnt = pnt_tmp.mutable_participantdata();
    pnt->CopyFrom(Pnt);
    int pnt_id = Pnt.ptc().ptcid();
    std::unique_lock<std::mutex> lock(ssm_mutex_, std::defer_lock);
    lock.lock();
    if (ssm_measurement.ssmframe().equipmenttype() == 2) {
      std::unordered_map<int, ::common::SSM_PNT_PB>::iterator iter = v2v_ssm_objects_.find(pnt_id);
      if (iter != v2v_ssm_objects_.end()) {
        v2v_ssm_objects_[pnt_id] = pnt_tmp;
      } else {
        v2v_ssm_objects_.insert(std::make_pair(pnt_id, pnt_tmp));
      }
    } else if (ssm_measurement.ssmframe().equipmenttype() == 1) {
      std::unordered_map<int, ::common::SSM_PNT_PB>::iterator iter = v2i_ssm_objects_.find(pnt_id);
      if (iter != v2i_ssm_objects_.end()) {
        v2i_ssm_objects_[pnt_id] = pnt_tmp;
      } else {
        v2i_ssm_objects_.insert(std::make_pair(pnt_id, pnt_tmp));
      }
    }
    lock.unlock();
  }
  ROS_INFO_STREAM("SsmCallback: v2i_ssm_objects_.size: " << v2i_ssm_objects_.size());
  ROS_INFO_STREAM("SsmCallback: v2v_ssm_objects_.size: " << v2v_ssm_objects_.size());
}

void V2xFusion::V2nRsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  ::common::RSM_PB rsm_measurement;
  V2xRosToProto(*msg, rsm_measurement);
  ROS_DEBUG_STREAM(rsm_measurement.DebugString());
  const double cloud_time =
      rsm_measurement.header().stamp().sec() + rsm_measurement.header().stamp().nsec() * 1e-9;
  double system_time = ros::Time::now().toSec();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ROS_DEBUG_STREAM("V2nRsmCallback: system time_diff = "
                   << std::setprecision(18) << system_time - cloud_time
                   << ", between system_time:" << system_time << "and cloud_time:" << cloud_time);
  ROS_DEBUG_STREAM("V2nRsmCallback: now time = " << tv.tv_sec + tv.tv_usec * 1e-6);

  localization::Localization localization;
  // current time diff threshold is 0.1 - liuxinyu
  if (use_time_filter_) {
    if (!QueryNearestLoc(cloud_time, localization)) {
      ROS_WARN("V2nRsmCallback: Fail to get localization for obu message timestamp.");
      return;
    }
  }
  const int obj_size = rsm_measurement.rsmframe().participants().participantdata_size();
  for (size_t i = 0; i < obj_size; ++i) {
    ::common::ParticipantData_PB Pnt_object =
        rsm_measurement.rsmframe().participants().participantdata(i);
    // Modify(@liuxinyu): modify store time
    ::common::RSM_PNT_PB obj_tmp;
    ros::Time ts;
    ts.fromSec(cloud_time);
    obj_tmp.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    obj_tmp.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    ::common::ParticipantData_PB* pnt_obj = obj_tmp.mutable_participantdata();
    pnt_obj->CopyFrom(Pnt_object);

    const int rsm_id = Pnt_object.ptcid();
    std::unordered_map<int, ::common::RSM_PNT_PB>::iterator iter = v2n_rsm_objects_.find(rsm_id);
    std::unique_lock<std::mutex> lock(v2n_rsm_mutex_, std::defer_lock);
    lock.lock();
    if (iter != v2n_rsm_objects_.end()) {
      v2n_rsm_objects_[rsm_id] = obj_tmp;
    } else {
      v2n_rsm_objects_.insert(std::make_pair(rsm_id, obj_tmp));
    }
    lock.unlock();
  }
  ROS_INFO_STREAM("V2nRsmCallback: v2n_rsm_objects_.size: " << v2n_rsm_objects_.size());
}

void V2xFusion::V2nRsiCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  ::common::RSI_PB rsi_measurement;
  V2xRosToProto(*msg, rsi_measurement);
  ROS_DEBUG_STREAM(rsi_measurement.DebugString());
  double cloud_time =
      rsi_measurement.header().stamp().sec() + rsi_measurement.header().stamp().nsec() * 1e-9;
  double system_time = ros::Time::now().toSec();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ROS_DEBUG_STREAM("V2nRsiCallback: system time_diff = "
                   << std::setprecision(18) << system_time - cloud_time
                   << ", between system_time:" << system_time << "and cloud_time:" << cloud_time);
  ROS_DEBUG_STREAM("V2nRsiCallback: now time = " << tv.tv_sec + tv.tv_usec * 1e-6);

  localization::Localization localization;
  // current time diff threshold is 0.1 - liuxinyu
  if (use_time_filter_) {
    if (!QueryNearestLoc(cloud_time, localization)) {
      ROS_WARN("V2nRsiCallback: Fail to get localization for obu message timestamp.");
      return;
    }
  }
  int obj_size = rsi_measurement.rsiframe().rtes().rtedata_size();
  for (size_t i = 0; i < obj_size; ++i) {
    ::common::RTEData_PB Pnt_object = rsi_measurement.rsiframe().rtes().rtedata(i);

    ::common::RSI_RTE_PB obj_tmp;
    ros::Time ts;
    ts.fromSec(cloud_time);
    obj_tmp.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    obj_tmp.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    ::common::RTEData_PB* pnt_obj = obj_tmp.mutable_rtedata();
    pnt_obj->CopyFrom(Pnt_object);

    int rsi_id = Pnt_object.rteid();
    std::unordered_map<int, ::common::RSI_RTE_PB>::iterator iter = v2n_rsi_objects_.find(rsi_id);
    std::unique_lock<std::mutex> lock(rsi_mutex_, std::defer_lock);
    lock.lock();
    if (iter != v2n_rsi_objects_.end()) {
      v2n_rsi_objects_[rsi_id] = obj_tmp;
    } else {
      v2n_rsi_objects_.insert(std::make_pair(rsi_id, obj_tmp));
    }
    lock.unlock();
  }
  ROS_INFO_STREAM("V2nRsiCallback: v2n_rsi_objects_.size: " << v2n_rsi_objects_.size());
}

void V2xFusion::LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  static double last_ts = 0.0;
  double ts_dis = msg->header.stamp.toSec() - last_ts;
  if (last_ts != 0 && ts_dis > 1.0) {
    ROS_WARN("LocalizationCallback: Localization message time out!");
  }
  last_ts = msg->header.stamp.toSec();

  while (global_localizations_.size() >= 100) {
    global_localizations_.pop_front();
  }

  localization::Localization global_localization;
  RosToProto(*msg, global_localization);
  if (global_localization.utm_zone() == 0) {
    ROS_ERROR("LocalizationCallback: Localization utm_zone zero error!");
  }
  global_localizations_.push_back(global_localization);
}

void V2xFusion::CollectionV2xObj() {
  if (bsm_objects_.size() == 0 && rsm_objects_.size() == 0 && v2n_rsm_objects_.size() == 0 &&
      v2v_ssm_objects_.size() == 0 && v2n_rsi_objects_.size() == 0 &&
      v2i_ssm_objects_.size() == 0) {
    ROS_WARN_THROTTLE(
        5,
        "CollectionV2xObj: bsm_objects_, rsm_objects_ ,v2n_rsm_objects_, v2v_ssm_objects_, "
        "v2n_rsi_objects_, v2i_ssm_objects_ are null.");
    return;
  }
  // time check
  localization::Localization localization;
  UpdateV2xObjToImu(localization);
  // set host
  self_veh_ident_ptr_->SetHost(localization);
  ros::Time ts;
  double timestamp =
      localization.header().stamp().sec() + localization.header().stamp().nsec() * 1e-9;
  ts.fromSec(timestamp);
  perception::ObuObjects obu_objects;
  static int seq = 0;
  obu_objects.mutable_header()->set_seq(seq++);
  obu_objects.mutable_header()->mutable_stamp()->set_sec(ts.sec);
  obu_objects.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
  obu_objects.set_sensor_name("obu");
  // v2x类型转换成MOGO
  InitClassIndex();
  perception::ObjectSource source;
  // parse bsm
  if (pub_bsm_ && bsm_objects_.size() != 0) {
    source = perception::ObjectSource::V2V_BSM;
    BsmObj2ObuObjectConvertor(bsm_objects_, obu_objects, source);
  }
  // parse v2i rsm
  if (pub_rsm_ && rsm_objects_.size() != 0) {
    source = perception::ObjectSource::V2I_RSM;
    RsmObj2ObuObjectConvertor(rsm_objects_, obu_objects, source);
  }
  // parse v2n rsm
  if (pub_v2n_rsm_ && v2n_rsm_objects_.size() != 0) {
    source = perception::ObjectSource::V2N_RSM;
    RsmObj2ObuObjectConvertor(v2n_rsm_objects_, obu_objects, source);
  }
  // parse ssm
  if (pub_v2v_ssm_ && v2v_ssm_objects_.size() != 0) {
    source = perception::ObjectSource::V2V_SSM;
    SsmObj2ObuObjectConvertor(v2v_ssm_objects_, obu_objects, source);
  }
  // parse v2n rsi
  if (pub_v2n_rsi_ && v2n_rsi_objects_.size() != 0) {
    source = perception::ObjectSource::V2N_RSI;
    RsiObj2ObuObjectConvertor(v2n_rsi_objects_, obu_objects, source);
  }
  // parse v2i ssm
  if (pub_v2i_ssm_ && v2i_ssm_objects_.size() != 0) {
    source = perception::ObjectSource::V2I_SSM;
    SsmObj2ObuObjectConvertor(v2i_ssm_objects_, obu_objects, source);
  }
  // pubulish obu frame for fusion
  std_msgs::String obu_frame;
  obu_objects.SerializeToString(&obu_frame.data);
  v2x_objects_pub_.publish(obu_frame);
  ROS_INFO_STREAM("CollectionV2xObj: obu frame publish: sensor name="
                  << obu_objects.sensor_name() << "object number = " << obu_objects.objs_size());

  if (pub_v2x_rviz_) {
    v2x_display_.ObuRTEDisplay(obu_objects, localization, v2x_rte_rviz_pub_);
    v2x_display_.ObuPNTDisplay(obu_objects, localization, v2x_pnt_rviz_pub_);
  }

}

// @brief: the threshold for determing data on different links is different
bool V2xFusion::UpdateV2xObjToImu(localization::Localization& localization) {
  if (global_localizations_.empty()) {
    ROS_ERROR("UpdateV2xObjToImu: Localization message NOT received.");
    return false;
  }
  localization = global_localizations_.back();
  double loc_ts = localization.header().stamp().sec() + localization.header().stamp().nsec() * 1e-9;
  // When the location information is not updated, clear the cached data
  std::unique_lock<std::mutex> bsm_lock(bsm_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> v2n_rsm_lock(v2n_rsm_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> v2x_rsm_lock(v2x_rsm_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> v2n_rsi_lock(rsi_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> v2i_ssm_lock(ssm_mutex_, std::defer_lock);
  if (latest_time_ == loc_ts) {
    bsm_lock.lock();
    bsm_objects_.clear();
    bsm_lock.unlock();
    
    v2x_rsm_lock.lock();
    rsm_objects_.clear();
    v2x_rsm_lock.unlock();

    v2n_rsm_lock.lock();
    v2n_rsm_objects_.clear();
    v2n_rsm_lock.unlock();

    v2n_rsi_lock.lock();
    v2n_rsi_objects_.clear();
    v2n_rsi_lock.unlock();

    v2i_ssm_lock.lock();
    v2i_ssm_objects_.clear();
    v2v_ssm_objects_.clear();
    v2i_ssm_lock.unlock();
    return false;
  }
  bsm_lock.lock();
  TimeCheck(bsm_objects_, loc_ts, TimeCheckOfStoreV2VBSM_);
  bsm_lock.unlock();

  v2x_rsm_lock.lock();
  TimeCheck(rsm_objects_, loc_ts, TimeCheckOfStoreV2VBSM_);
  v2x_rsm_lock.unlock();
  if (use_time_filter_) {
    v2n_rsm_lock.lock();
    TimeCheck(v2n_rsm_objects_, loc_ts, TimeCheckOfStoreV2NRSM_);
    v2n_rsm_lock.unlock();

    v2n_rsi_lock.lock();
    TimeCheck(v2n_rsi_objects_, loc_ts, TimeCheckOfStoreV2NRSI_);
    v2n_rsi_lock.unlock();

    v2i_ssm_lock.lock();
    TimeCheck(v2i_ssm_objects_, loc_ts, TimeCheckOfStoreV2ISSM_);
    TimeCheck(v2v_ssm_objects_, loc_ts, TimeCheckOfStoreV2ISSM_);
    v2i_ssm_lock.unlock();
  }
  latest_time_ = loc_ts;
  ROS_INFO_STREAM("UpdateV2xObjToImu: Size After QueryNearestLoc, v2n_rsm_objects_: "
                  << v2n_rsm_objects_.size());
  ROS_INFO_STREAM(
      "UpdateV2xObjToImu: Size After QueryNearestLoc, bsm_objects_: " << bsm_objects_.size());
  ROS_INFO_STREAM("UpdateV2xObjToImu: Size After QueryNearestLoc, v2n_rsi_objects_: "
                  << v2n_rsi_objects_.size());
  ROS_INFO_STREAM("UpdateV2xObjToImu: Size After QueryNearestLoc, v2i_ssm_objects_: "
                  << v2i_ssm_objects_.size());
  ROS_INFO_STREAM("UpdateV2xObjToImu: Size After QueryNearestLoc, v2v_ssm_objects_: "
                  << v2v_ssm_objects_.size());
  return true;
}

bool V2xFusion::IsHex(std::string str) {
  int len = str.length();
  for (int i = 0; i < len; i++) {
    if (str[i] >= '0' && str[i] <= '9' || str[i] >= 'a' && str[i] <= 'f' ||
        str[i] >= 'A' && str[i] <= 'F') {
      continue;
    } else {
      ROS_WARN_STREAM("IsHex: it is not hex");
      return false;
    }
  }
  return true;
}

void V2xFusion::InitClassIndex() {
  // 0: unkown; // 10: car; // 50: bus;　－　BSM
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(0, perception::TYPE_UNKNOWN));
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(2, perception::TYPE_BICYCLE));
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(3, perception::TYPE_PEDESTRIAN));
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(10, perception::TYPE_CAR));
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(20, perception::TYPE_TRUCK));
  class_index_.insert(std::pair<int32_t, perception::ObjectType>(40, perception::TYPE_MOTOR));

  class_index_.insert(std::pair<int32_t, perception::ObjectType>(50, perception::TYPE_BUS));

  // Modify-baihaijiang
  class_index_.insert(
      std::pair<int32_t, perception::ObjectType>(501, perception::TYPE_ROADWORK_OCCUPY_0501));
  class_index_.insert(
      std::pair<int32_t, perception::ObjectType>(502, perception::TYPE_ROADWORK_BREAK_0502));
}

bool V2xFusion::QueryNearestLoc(const double& timestamp, localization::Localization& localization) {
  if (global_localizations_.empty()) {
    ROS_ERROR("QueryNearestLoc: Localization message NOT received.");
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
  if (abs(stamp - loc_ts) > TimeCheckOfLocalization_) {
#if DEBUG
    std::ofstream outfile_v2n("/home/mogo/data/V2nTimeDiff.txt", std::ios::app);
    // time diff | obj time | localization
    outfile_v2n << std::setprecision(7) << abs(stamp - loc_ts) << " |" << std::setprecision(18)
                << stamp << " |" << std::setprecision(18) << loc_ts << "\n";
#endif

    ROS_ERROR_STREAM("QueryNearestLoc: Time distance "
                     << std::setprecision(7) << abs(stamp - loc_ts)
                     << " between Obj objects: " << std::setprecision(18) << stamp
                     << " and localization: " << std::setprecision(18) << loc_ts
                     << " is too long.");
    return false;
  }
  return true;
}

bool V2xFusion::BsmObj2ObuObjectConvertor(std::unordered_map<int, ::common::BSM_PB>& bsm_objects,
                                          perception::ObuObjects& obu_objects,
                                          perception::ObjectSource& source) {
  int tmp_uuid = 1;
  for (auto iter = bsm_objects.begin(); iter != bsm_objects.end(); iter++) {
    ::common::BSM_PB bsm = iter->second;
    if (!IsHex(bsm.bsmframe().id().str_value().c_str())) {
      ROS_WARN_STREAM(
          "BsmObj2ObuObjectConvertor: id is't hex: " << bsm.bsmframe().id().str_value().c_str());
      continue;
    }
    // printf("\033[31m BsmObj2ObuObjectConvertor:vehid %s.\033[0m\n",
    //        bsm.bsmframe().id().str_value().c_str());
    if (bsm.bsmframe().id().str_value().empty()) {
      ROS_WARN_STREAM("BsmObj2ObuObjectConvertor: id is empty");
    }
    perception::ObuObject* obu_obj = obu_objects.add_objs();
    perception::Object* obj = obu_obj->mutable_obj();
    obu_obj->set_longitude(bsm.bsmframe().pos().long_() * 1e-7);
    obu_obj->set_latitude(bsm.bsmframe().pos().lat() * 1e-7);
    obu_obj->set_alt(bsm.bsmframe().pos().elevation() * 0.1);
    obu_obj->set_speed(bsm.bsmframe().speed() * 0.02);  // units of 0.02 m/s
    obu_obj->set_heading(bsm.bsmframe().heading() * 0.0125);
    obu_obj->set_source(source);
    obu_obj->mutable_obj()->mutable_size()->set_x(
        bsm.bsmframe().size().length() * 0.01);  // units of 1 cm with a range of >40 meters
    obu_obj->mutable_obj()->mutable_size()->set_y(
        bsm.bsmframe().size().width() * 0.01);  // units of 1 cm with a range of >10 meters
    obu_obj->mutable_obj()->mutable_size()->set_z(bsm.bsmframe().size().height() *
                                                  0.05);  // units of 5 cm, range to 6.35 meters
    // convert type
    obj->set_type(class_index_[bsm.bsmframe().vehicleclass().classification()]);
    try {
      int vehid = stoi(bsm.bsmframe().id().str_value(), 0, 16);
      obj->set_id(vehid);
    } catch (const std::exception& e) {
      obj->set_id(tmp_uuid);
    }
    obj->set_confidence(0.99);
    tmp_uuid++;
#if DEBUG
    printf("\033[31m longitude is %lf.\033[0m\n", obu_obj->longitude());
    printf("\033[31m latitude is %lf.\033[0m\n", obu_obj->latitude());
    printf("\033[31m alt is %lf.\033[0m\n", obu_obj->alt());
    printf("\033[31m speed is %lf.\033[0m\n", obu_obj->speed());
    printf("\033[31m heading is %lf.\033[0m\n", obu_obj->heading());
    length width height(in meter)
        printf("\033[31m length is %f.\033[0m\n", obu_obj->obj().size().x());
    printf("\033[31m width is %f.\033[0m\n", obu_obj->obj().size().y());
    printf("\033[31m height is %f.\033[0m\n", obu_obj->obj().size().z());
    printf("\033[31m obu-type %d.\033[0m\n", bsm.bsmframe().vehicleclass().classification());
    printf("\033[31m fusion-type %d.\033[0m\n", obj->type());
#endif
  }
  return true;
}

bool V2xFusion::RsmObj2ObuObjectConvertor(
    std::unordered_map<int, ::common::RSM_PNT_PB>& rsm_objects, perception::ObuObjects& obu_objects,
    perception::ObjectSource& source) {
  for (auto iter = rsm_objects.begin(); iter != rsm_objects.end(); iter++) {
    ::common::RSM_PNT_PB rsm = iter->second;
    ROS_DEBUG_STREAM("RsmObj2ObuObjectConvertor: ****** id *******" << rsm.participantdata().ptcid());
    // filter ego car
    self_veh_ident_ptr_->GetRsmObs(rsm);
    self_veh_ident_ptr_->LocateSelf();
    if(use_filter_ego_car_ && self_veh_ident_ptr_->is_filter_flag_){
      continue;
    }
    perception::ObuObject* obu_obj = obu_objects.add_objs();
    perception::Object* obj = obu_obj->mutable_obj();
    double obj_lng = rsm.participantdata().pos().offsetll().position_latlon().lon() * 1e-7;
    double obj_lat = rsm.participantdata().pos().offsetll().position_latlon().lat() * 1e-7;
    double timestamp = rsm.header().stamp().sec() + rsm.header().stamp().nsec() * 1e-9;
    obj->set_id(rsm.participantdata().ptcid());
    obu_obj->set_longitude(obj_lng);
    obu_obj->set_latitude(obj_lat);
    obu_obj->set_alt(rsm.participantdata().pos().offsetv().elevation() * 0.1);
    obu_obj->set_speed(rsm.participantdata().speed() * 0.02);
    obu_obj->set_heading(rsm.participantdata().heading() * 0.0125);
    obu_obj->set_source(source);
    obu_obj->mutable_obj()->set_time_stamp(timestamp);
    obu_obj->mutable_obj()->mutable_size()->set_x(rsm.participantdata().size().length() * 0.01);
    obu_obj->mutable_obj()->mutable_size()->set_y(rsm.participantdata().size().width() * 0.01);
    obu_obj->mutable_obj()->mutable_size()->set_z(rsm.participantdata().size().height() * 0.05);
    if (rsm.participantdata().ptctype() == 2 || rsm.participantdata().ptctype() == 3) {
      obu_obj->mutable_obj()->set_type(class_index_[rsm.participantdata().ptctype()]);
    } else {
      obu_obj->mutable_obj()->set_type(
          class_index_[rsm.participantdata().vehicleclass().classification()]);
    }
#if DEBUG
    printf("\033[31m length is %f.\033[0m\n", obu_obj->obj().size().x());
    printf("\033[31m width is %f.\033[0m\n", obu_obj->obj().size().y());
    printf("\033[31m height is %f.\033[0m\n", obu_obj->obj().size().z());
#endif
  }
  return true;
}

bool V2xFusion::RsiObj2ObuObjectConvertor(
    std::unordered_map<int, ::common::RSI_RTE_PB>& rsi_objects, perception::ObuObjects& obu_objects,
    perception::ObjectSource& source) {
  for (auto iter = rsi_objects.begin(); iter != rsi_objects.end(); iter++) {
    ::common::RSI_RTE_PB rsi = iter->second;
    // filter ego car
    // self_veh_ident_ptr_->GetSsmObs(ssm);
    // self_veh_ident_ptr_->LocateSelf();
    // if (use_filter_ego_car_ && self_veh_ident_ptr_->is_filter_flag_) {
    //   continue;
    // }
    perception::ObuObject* obu_obj = obu_objects.add_objs();
    perception::Object* obj = obu_obj->mutable_obj();
    double obj_lng = rsi.rtedata().eventpos().offsetll().position_latlon().lon() * 1e-7;
    double obj_lat = rsi.rtedata().eventpos().offsetll().position_latlon().lat() * 1e-7;
    double timestamp = rsi.header().stamp().sec() + rsi.header().stamp().nsec() * 1e-9;
    obj->set_id(rsi.rtedata().rteid());
    obu_obj->set_longitude(obj_lng);
    obu_obj->set_latitude(obj_lat);
    obu_obj->set_source(source);
    obu_obj->mutable_obj()->set_type(class_index_[rsi.rtedata().eventtype()]);
    obu_obj->mutable_obj()->set_time_stamp(timestamp);

    int referencepath_size = rsi.rtedata().referencepaths().referencepath_size();
    for (size_t i = 0; i < referencepath_size; ++i) {
      ::common::ReferencePath_PB referencePath = rsi.rtedata().referencepaths().referencepath(i);
      int offsetllv_size = referencePath.activepath().offsetllv_size();
      geometry::Point* polygon_p;
      for (size_t j = 0; j < offsetllv_size; ++j) {
        PositionOffsetLLV_PB offsetLLV = referencePath.activepath().offsetllv(j);
        polygon_p = obj->add_polygon();
        polygon_p->set_x(offsetLLV.offsetll().position_latlon().lon() * 1e-7);
        polygon_p->set_y(offsetLLV.offsetll().position_latlon().lat() * 1e-7);
        polygon_p->set_z(0);
      }
    }
  }
}

bool V2xFusion::SsmObj2ObuObjectConvertor(
    std::unordered_map<int, ::common::SSM_PNT_PB>& ssm_objects, perception::ObuObjects& obu_objects,
    perception::ObjectSource& source) {
  for (auto iter = ssm_objects.begin(); iter != ssm_objects.end(); iter++) {
    ::common::SSM_PNT_PB ssm = iter->second;
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: ****** id *******" << ssm.participantdata().ptc().ptcid());
    // filter ego car
    self_veh_ident_ptr_->GetSsmObs(ssm);
    self_veh_ident_ptr_->LocateSelf();
    if (use_filter_ego_car_ && self_veh_ident_ptr_->is_filter_flag_) {
      continue;
    }
    if (ssm.participantdata().ptc().ptctype() == 4) {  // filter rsu pos
      continue;
    }
    perception::ObuObject* obu_obj = obu_objects.add_objs();
    perception::Object* obj = obu_obj->mutable_obj();
    double obj_lng = ssm.participantdata().ptc().pos().offsetll().position_latlon().lon() * 1e-7;
    double obj_lat = ssm.participantdata().ptc().pos().offsetll().position_latlon().lat() * 1e-7;
    double timestamp = ssm.header().stamp().sec() + ssm.header().stamp().nsec() * 1e-9;
    obj->set_id(ssm.participantdata().ptc().ptcid());
    obu_obj->set_longitude(obj_lng);
    obu_obj->set_latitude(obj_lat);
    obu_obj->set_alt(ssm.participantdata().ptc().pos().offsetv().elevation() * 0.1);
    obu_obj->set_speed(ssm.participantdata().ptc().speed() * 0.02);
    obu_obj->set_heading(ssm.participantdata().ptc().heading() * 0.0125);
    obu_obj->set_source(source);
    obu_obj->mutable_obj()->set_time_stamp(timestamp);
    obu_obj->mutable_obj()->mutable_size()->set_x(ssm.participantdata().ptc().size().length() *
                                                  0.01);
    obu_obj->mutable_obj()->mutable_size()->set_y(ssm.participantdata().ptc().size().width() *
                                                  0.01);
    obu_obj->mutable_obj()->mutable_size()->set_z(ssm.participantdata().ptc().size().height() *
                                                  0.05);
    if (ssm.participantdata().ptc().ptctype() == 2 || ssm.participantdata().ptc().ptctype() == 3) {
      obu_obj->mutable_obj()->set_type(class_index_[ssm.participantdata().ptc().ptctype()]);
    } else {
      obu_obj->mutable_obj()->set_type(
          class_index_[ssm.participantdata().ptc().vehicleclass().classification()]);
    }
    obu_obj->set_status_duration(ssm.participantdata().statusduration()); // 10ms
    obu_obj->mutable_obj()->mutable_obu_supplement()->set_status_duration(ssm.participantdata().statusduration());// 10ms
    obu_obj->mutable_obj()->mutable_obu_supplement()->set_type_confidence(ssm.participantdata().typeconfidence());
    obu_obj->mutable_obj()->mutable_obu_supplement()->set_position_confidence(ssm.participantdata().ptc().posconfidence().poscon());// 10ms
    obu_obj->mutable_obj()->mutable_obu_supplement()->set_velocity_confidence(ssm.participantdata().ptc().motioncfd().speedcfd());// 10ms
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: -----------------------");
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_id: " << ssm.participantdata().ptc().ptcid());
    ROS_DEBUG_STREAM(
        "SsmObj2ObuObjectConvertor: set_longitude: " << ssm.participantdata().ptc().pos().offsetll().position_latlon().lon() *
                                 1e-7);
    ROS_DEBUG_STREAM(
        "SsmObj2ObuObjectConvertor: set_latitude: " << ssm.participantdata().ptc().pos().offsetll().position_latlon().lat() *
                                1e-7);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_alt: " << ssm.participantdata().ptc().pos().offsetv().elevation());
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_speed: " << ssm.participantdata().ptc().speed() * 0.02);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_heading: " << ssm.participantdata().ptc().heading() * 0.0125);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_x: " << ssm.participantdata().ptc().size().length() * 0.01);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_y: " << ssm.participantdata().ptc().size().width() * 0.01);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: set_z: " << ssm.participantdata().ptc().size().height() * 0.05);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: statusduration: " << ssm.participantdata().statusduration() * 10);
    ROS_DEBUG_STREAM("SsmObj2ObuObjectConvertor: -----------------------");
  }
  return true;
}

}  // namespace v2x
}  // namespace perception