#include "lidar_data_fusion.h"


namespace perception{
namespace xiaoba_fusion{

LidarDataFusion::LidarDataFusion()
    : private_nh_("~")
    , monitor_stamp_flag_(true)
    , fusion_frame_id_("base_link")
    , over_time_threshold_(0.03)
    , lidar_time_offset_(0.0)
    , pub_sequence_(0)
    , pub_abnormal_count_(0)
    , temp_cloud_ptr(new sensor_msgs::PointCloud2)
    , m_pclsLidarDataManager(nullptr)
    , m_pclsLidarFaultReporter(new LidarFaultReporter)
{
  std::string output_cloud_topic_;
  private_nh_.getParam("output_cloud_topic", output_cloud_topic_);
  private_nh_.getParam("fusion_frame_id", fusion_frame_id_);
  private_nh_.getParam("monitor_stamp_flag", monitor_stamp_flag_);
  private_nh_.getParam("over_time_threshold", over_time_threshold_);
  private_nh_.getParam("lidar_time_offset", lidar_time_offset_);
  ROS_INFO_STREAM("monitor_stamp_flag = " << monitor_stamp_flag_);
  ROS_INFO_STREAM("over_time_threshold = " << over_time_threshold_);
  ROS_INFO_STREAM("lidar_time_offset = " << lidar_time_offset_);

  fusion_pub = private_nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1000);

  Init();

  m_pclsLidarDataManager = LidarDataManager::Instance();
  m_pclsLidarDataManager->SetMonitorStampFlag(monitor_stamp_flag_);

}

LidarDataFusion::~LidarDataFusion(){
}

void LidarDataFusion::Init(){
  last_topic_publish_time_ = ros::Time(0, 0);
  current_topic_start_time_ = 0;
  init_time_ = 0;
  pub_sequence_ = 0;
  isget_map_.clear();
}

void LidarDataFusion::FusionAndPublishCloud(){
  // 1. Get latest frame and fusion
  int fusion_topic_num_ = 0;
  pub_abnormal_count_ += 1;
  if(pub_sequence_ > ULONG_MAX) pub_sequence_ = 0;
  pub_sequence_++;
  ros::Time max_time_ = last_topic_publish_time_;
  sensor_msgs::PointCloud2Ptr cloud_ptr(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr merge_point_cloud_(new sensor_msgs::PointCloud2);
  auto start_time = std::chrono::steady_clock::now();
  for (const std::string input : lidar_names_) {
    if (!m_pclsLidarDataManager->GetLatestSensorFrames(input, cloud_ptr)) {
      ROS_WARN("Get latest frame failed. The frame id is %s", input.c_str());
      continue;
    }
    if ((last_topic_publish_time_.toSec() > 1600000000) 
      &&(cloud_ptr->header.stamp.toSec() - last_topic_publish_time_.toSec()) < 0.001)
    {
      ROS_WARN_STREAM("Discard data due to cloud time [ " << std::setprecision(18) << cloud_ptr->header.stamp.toSec()
        << " ] less than last topic publish time [ " << std::setprecision(18) << last_topic_publish_time_.toSec() << " ], sensor id = " << input.c_str());
      continue;
    }
    max_time_ = cloud_ptr->header.stamp.toSec() > max_time_.toSec() ? cloud_ptr->header.stamp : max_time_;

    merge_point_cloud_->height = cloud_ptr->height;
    merge_point_cloud_->fields = cloud_ptr->fields;
    merge_point_cloud_->is_dense = cloud_ptr->is_dense;
    merge_point_cloud_->point_step = cloud_ptr->point_step;
    merge_point_cloud_->is_bigendian = cloud_ptr->is_bigendian;
    merge_point_cloud_->width += cloud_ptr->width;
    merge_point_cloud_->row_step = merge_point_cloud_->width * merge_point_cloud_->height;
    merge_point_cloud_->data.insert(merge_point_cloud_->data.end(), cloud_ptr->data.begin(), cloud_ptr->data.end());

    fusion_topic_num_ += 1;
  }
  if ((max_time_.toSec() - last_topic_publish_time_.toSec()) > 0.05) {
    last_topic_publish_time_ = max_time_;
    if (merge_point_cloud_->width) {
      // publsh points
      merge_point_cloud_->header.seq = pub_sequence_;
      merge_point_cloud_->header.stamp = ros::Time(last_topic_publish_time_.toSec() + lidar_time_offset_);
      merge_point_cloud_->header.frame_id = fusion_frame_id_;
      fusion_pub.publish(*merge_point_cloud_);
    }
    if(fusion_topic_num_ == m_vecSubsrciber.size())
      pub_abnormal_count_ = 0;
    else if(fusion_topic_num_ < m_vecSubsrciber.size())
      ROS_WARN("fusion topic number is %d", fusion_topic_num_);
  }
  std::lock_guard<std::mutex> lock(storage_mutex_);
  isget_map_.clear();
}

void LidarDataFusion::PointCouldCallBack(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (abs(init_time_ - msg->header.stamp.toSec()) < 0.001) {
    Init();
    m_pclsLidarDataManager->Reset();
  }
  if (init_time_ < DBL_MIN)
    init_time_ = msg->header.stamp.toSec();

  std::string sensor_id = msg->header.frame_id;
  ros::Time cur_sys_time = ros::Time::now();
  if (isget_map_.empty()) {
    current_topic_start_time_ = ros::Time::now().toSec();
    first_lidar_stamp = msg->header.stamp.toSec();
  }
  if (fabs(cur_sys_time.toSec() - msg->header.stamp.toSec()) > SYS_TIMESTAMP_OVERTIME_THRESHOLD)
    ROS_WARN("Time synchronization from xavier to lidar failed.");
  {
    std::lock_guard<std::mutex> lock(storage_mutex_);
    isget_map_[sensor_id] = true;
    *temp_cloud_ptr = *msg;
    m_pclsLidarDataManager->AddSensorMeasurements(std::make_pair(cur_sys_time, temp_cloud_ptr));
  }
  if (isget_map_.size() == m_vecSubsrciber.size())
    FusionAndPublishCloud();
}

void LidarDataFusion::OvertimeEvent(const ros::TimerEvent &){
  double cur_time = ros::Time::now().toSec();
  if (!isget_map_.empty() && ((cur_time - current_topic_start_time_) > over_time_threshold_)) {
    ROS_WARN("over time occure.");
    FusionAndPublishCloud();
  }
  if (pub_abnormal_count_ > 10) {
    Init();
    pub_abnormal_count_ = 0;
    m_pclsLidarDataManager->Reset();
    m_pclsLidarFaultReporter->ReportFaultMsg(cur_time);
    ROS_WARN("The publish topic no any message, so reset topic start time and init time.");
  }
}

void LidarDataFusion::Run() {
  ROS_INFO_STREAM("FLAGS_lidar_input: " << FLAGS_lidar_input.c_str());
  lidar_names_ = base::split(FLAGS_lidar_input, ';');
  for (const std::string input : lidar_names_) {
    std::string topic_name = m_pclsLidarDataManager->GetTopicName(input);
    if (topic_name.empty()) {
      ROS_ERROR("Can not parser topic name based sensor name.");
      continue;
    } else {
      ROS_INFO("[%s]:%s", input.c_str(), topic_name.c_str());
      m_vecSubsrciber.emplace_back(private_nh_.subscribe(topic_name, 10, &LidarDataFusion::PointCouldCallBack, this));
    }
  }

  over_timer = nh_.createTimer(ros::Duration(0.005), &LidarDataFusion::OvertimeEvent, this);
}

}   // namespace xiaoba_fusion
}   // namespace perception

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "xiaoba_lidars_fusion");

  perception::xiaoba_fusion::LidarDataFusion clsFusion;
  clsFusion.Run();

  ros::spin();
}