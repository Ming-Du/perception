/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#include "cloud_subscriber.h"

#include "glog/logging.h"

namespace perception {
namespace mid_fusion {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
  CloudData cloud_data;
  pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
  // cloud_data.time =cloud_msg_ptr->header.stamp.toNSec()*1e-6;
  cloud_data.time = cloud_data.cloud_ptr->header.stamp * 1e-6;
  new_cloud_data_.emplace_back(cloud_data);
  while (new_cloud_data_.size() > 2) {
    new_cloud_data_.pop_front();
  }
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
  if (new_cloud_data_.size() > 0) {
    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
  while (cloud_data_buff.size() > 2) {
    cloud_data_buff.pop_front();
  }
}
}  // namespace mid_fusion
}  // namespace perception
