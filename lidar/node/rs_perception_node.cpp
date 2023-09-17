#include <ros/ros.h>
#include "application/include/rs_perception.h"

using namespace robosense;

int main(int argc, char** argv) {
//   gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "rs_perception_node");

  LidarPerceptionPtr lidar_perception_ptr_(new LidarPerception);
  lidar_perception_ptr_->perception();

  return true;
}
