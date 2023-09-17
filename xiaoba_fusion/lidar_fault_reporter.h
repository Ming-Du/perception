#pragma once

#include <ros/ros.h>
#include "common/proto/fm_info.pb.h"

namespace perception{
namespace xiaoba_fusion{

class LidarFaultReporter {
public:
  LidarFaultReporter();
  ~LidarFaultReporter();

  void ReportFaultMsg(double stamp_sec);

private:
  void Init();

private:
  ros::NodeHandle private_nh_;

  ros::Publisher fault_pub_;

  uint64_t seq_; 
  uint64_t pub_time_in_ms_;
  
  std::string output_fault_topic_;
  std::string ids_lidars_time_unsync_;

};

typedef std::shared_ptr<LidarFaultReporter> LidarFaultReporterPtr;
typedef std::shared_ptr<const LidarFaultReporter> LidarFaultReporterConstPtr;

}   // namespace xiaoba_fusion
}   // namespace perception