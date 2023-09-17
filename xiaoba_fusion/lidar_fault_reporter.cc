#include "lidar_fault_reporter.h"

#include "autopilot_msgs/BinaryData.h"
#include "common/include/pb_utils.h"

namespace perception{
namespace xiaoba_fusion{

LidarFaultReporter::LidarFaultReporter()
  : private_nh_("~")
  , seq_(0)
  , output_fault_topic_("/system/fm/fault_report")
  , ids_lidars_time_unsync_("Perception_XiaobaFusion_LidarUnsynchronized")
{
  Init();

  fault_pub_ = private_nh_.advertise<autopilot_msgs::BinaryData>(output_fault_topic_, 1);
}

LidarFaultReporter::~LidarFaultReporter(){

}

void LidarFaultReporter::Init(){
  pub_time_in_ms_ = (uint64_t)(ros::Time::now().toSec() * 1e3 + 0.5);
}

void LidarFaultReporter::ReportFaultMsg(double stamp_sec){
  uint64_t cur_time_in_ms = (uint64_t)(stamp_sec * 1e3 + 0.5);
  if(cur_time_in_ms - pub_time_in_ms_ < 1e3)
    return;

  pub_time_in_ms_ = cur_time_in_ms;

  fault_management::FaultReportMsg *p_fault_msg_ = new fault_management::FaultReportMsg;
  p_fault_msg_->set_src("xiaoba_lidars_fusion");

  fault_management::FaultInfo *infos = p_fault_msg_->add_infos();
  infos->set_fault_id(ids_lidars_time_unsync_);
  infos->set_fault_time(pub_time_in_ms_);
  infos->set_fault_level("warn");

  autopilot_msgs::BinaryData ros_msg;
  ros_msg.header.stamp = ros::Time::now();
  ros_msg.header.seq = seq_++;
  ros_msg.name = p_fault_msg_->GetTypeName();
  common::SerializeProto(*p_fault_msg_, ros_msg.data);
  ros_msg.size = ros_msg.data.size();
  fault_pub_.publish(ros_msg);
}

}   // namespace xiaoba_fusion
}   // namespace perception