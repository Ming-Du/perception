/*
 * @Description: 订阅camera2d信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#ifndef CAMERA2D_SUBSCRIBER_SUBSCRIBER_HPP_
#define CAMERA2D_SUBSCRIBER_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <deque>

#include "common/proto/object.pb.h"

namespace perception {
namespace mid_fusion {
class Camera2dSubscriber {
 public:
  Camera2dSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  Camera2dSubscriber() = default;
  void ParseData(std::deque<perception::VisualObjects>& deque_camera2d_data);

 private:
  void msg_callback(const std_msgs::StringConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<perception::VisualObjects> new_camera2d_data_deque_;
  std::string topicname_;
};
}  // namespace mid_fusion
}  // namespace perception

#endif