#include <ros/ros.h>
#include "obstacle_radar_fusion.h"

std::unique_ptr<perception::radar::RadarFusion> radar_fusion_ptr_;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // set ROS LOG output level : Debug    Info Warn Error Fatal Count
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ros::init(argc, argv, "radar_fusion");
  ros::NodeHandle pn("~");
  std::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle);

  radar_fusion_ptr_.reset(new perception::radar::RadarFusion(node));
  if (!radar_fusion_ptr_->Init()) {
    ROS_ERROR("main: radar fusion failed to init!");
    return -1;
  }

  ros::spin();
  return 0;
}