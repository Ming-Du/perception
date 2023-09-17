/*
 * @Description: 订阅激光object信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#include "lidarobject_subscriber.h"

#include "glog/logging.h"

namespace perception {
namespace mid_fusion {
 std::string recv_lidar_sensor_name_("lidar_main_test");
LidarobjectSubscriber::LidarobjectSubscriber(ros::NodeHandle& nh, std::string topic_name,
                                             size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &LidarobjectSubscriber::msg_callback, this);
  topicname_ = topic_name;
}

void LidarobjectSubscriber::msg_callback(const std_msgs::StringConstPtr& msg) {
  perception::TrackedObjects lidarobject_measurement;
  lidarobject_measurement.ParseFromString(msg->data);
  recv_lidar_sensor_name_ = lidarobject_measurement.sensor_name();
  new_lidarobject_data_deque_.emplace_back(lidarobject_measurement);
  double recv_timestamp = lidarobject_measurement.header().stamp().sec() +
                          lidarobject_measurement.header().stamp().nsec() * 1e-9;
  ROS_INFO_THROTTLE(1, "msg_callback:DataCheck size: %d time: %lf\n", lidarobject_measurement.objs_size(),recv_timestamp);
  while (new_lidarobject_data_deque_.size() > 10) {
    new_lidarobject_data_deque_.pop_front();
  }
}

void LidarobjectSubscriber::ParseData(
    std::deque<perception::TrackedObjects>& deque_lidarobject_data) {
  if (new_lidarobject_data_deque_.size() > 0) {
    deque_lidarobject_data.insert(deque_lidarobject_data.end(), new_lidarobject_data_deque_.begin(),
                                  new_lidarobject_data_deque_.end());
    new_lidarobject_data_deque_.clear();
  }
  while (deque_lidarobject_data.size() > 10) {
    deque_lidarobject_data.pop_front();
  }
}
}  // namespace mid_fusion
}  // namespace perception
