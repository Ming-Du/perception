#include <ros/ros.h>
#include <thread>
#include "obstacle_v2x_fusion.h"

std::unique_ptr<perception::v2x::V2xFusion> v2x_fusion_ptr_;

void proc() {
  ros::Rate loop_rate(20);
  static double last_post_time;
  while (ros::ok()) {
    double this_post_time = ros::Time::now().toSec();
    if (last_post_time < 1e-6 || this_post_time - last_post_time > 0.04) {
      v2x_fusion_ptr_->CollectionV2xObj();
      last_post_time = this_post_time;
    }
    loop_rate.sleep();
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // set ROS LOG output level : Debug	Info Warn Error Fatal Count
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::init(argc, argv, "v2x_fusion");
  ros::NodeHandle pn("~");
  std::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle);

  v2x_fusion_ptr_.reset(new perception::v2x::V2xFusion(node));
  if (!v2x_fusion_ptr_->Init()) {
    ROS_ERROR("main: v2x fusion failed to init!");
    return -1;
  }
  std::thread parser{proc};

  ros::spin();
  parser.join();
  return 0;
}