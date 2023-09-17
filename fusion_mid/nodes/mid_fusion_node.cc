// headers in STL
#include <iostream>
// headers in local files
#include <termios.h>

#include <thread>

#include "fpnet_processing_unit.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "perception_fusion_mid");
  ROS_INFO("main: input arg:");

  for (int i = 0; i < argc; i++) {
    ROS_INFO_STREAM(argv[i]);
  }

  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // set ROS LOG output level : Debug	Info Warn Error Fatal Count
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  std::shared_ptr<perception::mid_fusion::FpnetProcessingUnit> fpnet_processing_unit_ptr =
      std::make_shared<perception::mid_fusion::FpnetProcessingUnit>();
  fpnet_processing_unit_ptr->CreateROSPubSub();
  ros::Rate rate(100); 
  while (ros::ok()) {
    ros::spinOnce();
    fpnet_processing_unit_ptr->Run();
    rate.sleep();
  }
  return 0;
}
