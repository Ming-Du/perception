/*
 * @Description: 订阅激光object信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#ifndef LIDAROBJECT_SUBSCRIBER_SUBSCRIBER_HPP_
#define LIDAROBJECT_SUBSCRIBER_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <deque>

#include "common/proto/object.pb.h"

namespace perception {
namespace mid_fusion {
class LidarobjectSubscriber {
 public:
  LidarobjectSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  LidarobjectSubscriber() = default;
  void ParseData(std::deque<perception::TrackedObjects>& deque_lidarobject_data);

 private:
  void msg_callback(const std_msgs::StringConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<perception::TrackedObjects> new_lidarobject_data_deque_;
  std::string topicname_;
};
}  // namespace mid_fusion
}  // namespace perception

#endif