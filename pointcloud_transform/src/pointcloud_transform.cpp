#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf2/convert.h"
#include "common/include/log.h"
#include "common/include/pb_utils.h"
#include "perception/base/perception_gflags.h"
#include "perception/base/proto/perception_component.pb.h"
#include "common/proto/sensor_meta.pb.h"
#include "tf2_sensor_msgs.h"



perception::LidarComponentInitOptions options_;
perception::base::RegisteredSensor sensor_list_proto;
std::vector<std::string> topics;
std::vector<std::string> frame_ids;
std::string base_frame;
std::vector<ros::Publisher> PointCloudPublisher;

bool IsLidar(const perception::base::SensorType& type);
void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
std::string Orientation(perception::base::SensorOrientation orientation);
int GetPublisher(std::string frame_id);

int main(int argc, char **argv){

  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true); // 命令行设置参数
  ros::init(argc, argv, "calibration_lidar");
  ros::NodeHandle node("~");

  AINFO << "read config from: " << FLAGS_sensor_meta_path;
  if (!common::GetProtoFromFile(FLAGS_sensor_meta_path, sensor_list_proto)) {
    return -1;
  }
  base_frame = sensor_list_proto.base();

  std::vector<ros::Subscriber> LidarSubsrciber;
  for (const perception::base::SensorInfo& sensor_meta_info : sensor_list_proto.sensor_info()) {
    if (IsLidar(sensor_meta_info.type())){
      LidarSubsrciber.emplace_back(node.subscribe(sensor_meta_info.topic(), 3,&PointCloudCallback));
            PointCloudPublisher.emplace_back(node.advertise<sensor_msgs::PointCloud2>(
          "/calibration/lidar/" + Orientation(sensor_meta_info.orientation()),3));
      topics.push_back(sensor_meta_info.topic());
      frame_ids.push_back(sensor_meta_info.name());
    }
  }

  ros::spin();
  return 0;

}




void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){

  static tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped transform_msg;
  sensor_msgs::PointCloud2 pc_msg;

  //read tf based on frame_id
  try {
    listener.lookupTransform(base_frame, msg->header.frame_id,
                             ros::Time(0), transform);
    std::cout <<"listen!!"<<std::endl;
    tf::transformStampedTFToMsg(transform, transform_msg);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.1).sleep();
  }

  //do transform
  std::cout <<"================================"<<std::endl;
  std::cout <<transform_msg.transform.rotation.x<<std::endl;
  std::cout <<transform_msg.transform.rotation.y<<std::endl;
  std::cout <<transform_msg.transform.rotation.z<<std::endl;
  std::cout <<transform_msg.transform.rotation.w<<std::endl;
  std::cout <<"================================"<<std::endl;

  tf2::doTransform(*msg, pc_msg, transform_msg);
  pc_msg.header = msg->header;
  pc_msg.header.frame_id = base_frame;

//  geometry_msgs::TransformStamped I_t;
//  I_t.transform.rotation.x = 0;
//  I_t.transform.rotation.y = 0;
//  I_t.transform.rotation.z = 0;
//  I_t.transform.rotation.w = 1;
//  I_t.transform.translation.x = 0;
//  I_t.transform.translation.y = 0;
//  I_t.transform.translation.z = 0;
//  tf2::doTransform(*msg, pc_msg, I_t);
//  pc_msg.header = msg->header;
  std::cout <<msg->header.frame_id<<std::endl;

  //publish
  PointCloudPublisher[GetPublisher(msg->header.frame_id)].publish(pc_msg);
}

bool IsLidar(const perception::base::SensorType& type) {
  return type == perception::base::SensorType::LSLIDAR_C16 ||
         type == perception::base::SensorType::LSLIDAR_C32 ||
         type == perception::base::SensorType::LSLIDAR_CH ||
         type == perception::base::SensorType::LIVOX_HORIZON ||
         type == perception::base::SensorType::HESAI_XT32 ||
         type == perception::base::SensorType::VELODYNE_64;
}

std::string Orientation(perception::base::SensorOrientation orientation){
  switch (orientation) {
    case 0: return "FRONT";
    case 1: return "LEFT_FORWARD";
    case 2: return "LEFT";
    case 3: return "LEFT_BACKWARD";
    case 4: return "REAR";
    case 5: return "RIGHT_BACKWARD";
    case 6: return "RIGHT";
    case 7: return "RIGHT_FORWARD";
    case 8: return "PANORAMIC";
  }
}

int GetPublisher(std::string frame_id){
  for(int i = 0; i< frame_ids.size(); i++){
    if (frame_id == frame_ids[i]) return i;
  }
}