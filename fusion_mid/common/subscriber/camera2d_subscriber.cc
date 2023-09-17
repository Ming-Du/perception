/*
 * @Description: 订阅camera2d信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#include "camera2d_subscriber.h"

#include "glog/logging.h"

namespace perception {
namespace mid_fusion {
constexpr int kCamera2DImageBufferSize = 10;
constexpr int kCamera2DObstacleBufferSize = 10;
Camera2dSubscriber::Camera2dSubscriber(ros::NodeHandle& nh, std::string topic_name,
                                       size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &Camera2dSubscriber::msg_callback, this);
  topicname_ = topic_name;
}

void Camera2dSubscriber::msg_callback(const std_msgs::StringConstPtr& msg) {
  perception::VisualObjects camera_measurement;
  camera_measurement.ParseFromString(msg->data);
  if (new_camera2d_data_deque_.size() > kCamera2DImageBufferSize) {
    new_camera2d_data_deque_.pop_front();
  }
  new_camera2d_data_deque_.emplace_back(camera_measurement);
  while (new_camera2d_data_deque_.size() > kCamera2DImageBufferSize) {
    new_camera2d_data_deque_.pop_front();
  }
}

void Camera2dSubscriber::ParseData(std::deque<perception::VisualObjects>& cloud_camera2d_buff) {
  if (new_camera2d_data_deque_.size() > 0) {
    cloud_camera2d_buff.insert(cloud_camera2d_buff.end(), new_camera2d_data_deque_.begin(),
                               new_camera2d_data_deque_.end());
    new_camera2d_data_deque_.clear();
  }
  while (cloud_camera2d_buff.size() > kCamera2DObstacleBufferSize) {
    cloud_camera2d_buff.pop_front();
  }
}
}  // namespace mid_fusion
}  // namespace perception