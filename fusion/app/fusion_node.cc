#include <ros/console.h>
#include <ros/ros.h>
#include "perception_fusion_component.h"

int main(int argc, char** argv) {
  ROS_DEBUG("main: input arg:");

  for (int i = 0; i < argc; i++) {
    ROS_DEBUG_STREAM(argv[i]);
  }

  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // set ROS LOG output level : Debug	Info Warn Error Fatal Count
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ros::init(argc, argv, "perception_fusion");

  std::unique_ptr<perception::fusion::PerceptionFusionComponent> perception_fusion_ptr_;
  perception_fusion_ptr_.reset(new perception::fusion::PerceptionFusionComponent());
  if (!perception_fusion_ptr_->Init()) {
    ROS_ERROR("main: perception fusion failed to init!");
    return -1;
  }

  ros::spin();
  return 0;
}
