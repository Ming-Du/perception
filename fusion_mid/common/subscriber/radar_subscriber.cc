/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: lijun
 * @Date: 2022-08-05
 */

#include "radar_subscriber.h"

#include "glog/logging.h"

namespace perception {
namespace mid_fusion {
RadarSubscriber::RadarSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &RadarSubscriber::msg_callback, this);
  topicname_ = topic_name;
}

void RadarSubscriber::msg_callback(const std_msgs::StringConstPtr& msg) {
  perception::RadarObjects radar_measurement;
  radar_measurement.ParseFromString(msg->data);

  perception::RadarObjects radar_measurement_temp;
  radar_measurement_temp.mutable_header()->set_seq(radar_measurement.header().seq());
  radar_measurement_temp.mutable_header()->mutable_stamp()->set_sec(
      radar_measurement.header().stamp().sec());
  radar_measurement_temp.mutable_header()->mutable_stamp()->set_nsec(
      radar_measurement.header().stamp().nsec());
  radar_measurement_temp.mutable_header()->set_frame_id(radar_measurement.header().frame_id());
  radar_measurement_temp.mutable_header()->set_module_name(
      radar_measurement.header().module_name());
  radar_measurement_temp.set_sensor_name(radar_measurement.sensor_name());
  // perception::VisualObjects newest_camera_frame_front60;
  // if(is_publish_radar_obstacle_)
  // {
  //     double tempTimeradarObj = radar_measurement.header().stamp().sec() +
  //     radar_measurement.header().stamp().nsec() * 1e-9;
  //     FindNewestCameraDetectionFrame(tempTimeradarObj,visualObjectsBuffer_front60_,
  //     newest_camera_frame_front60);
  // }

  for (size_t i = 0; i < radar_measurement.objs_size(); ++i) {
    perception::RadarObject radar_object = radar_measurement.objs(i);

    // if(is_publish_radar_obstacle_)
    // {
    //     bool
    //     is_onimage=RadartoImageprecess(radar_object,newest_camera_frame_front60);
    //     if(is_onimage){
    //         perception::RadarObject *Radar_object_output =
    //         output_radar_measurement.add_objs();
    //         Radar_object_output->CopyFrom(radar_object);
    //     }
    // }

    if (radar_object.meas_state() != 2)  // 2 = measured, 3 = predicted
    {
      continue;
    }
    // if (fabs(radar_object.obj().center().x()) > 60 ||
    //     fabs(radar_object.obj().center().y()) > 10)  //和激光雨滴模式同步
    // {
    //   continue;
    // }
    // if(std::abs(radar_object.velocity().x()) < 1.0)  // add
    // temporarily-guoxiaoxiao, need or not?
    //   continue;
    // rewriteRadarObject(radar_object, localization);
    perception::RadarObject* Radar_object = radar_measurement_temp.add_objs();
    Radar_object->CopyFrom(radar_object);
  }
  new_radar_data_deque_.emplace_back(radar_measurement_temp);
  while (new_radar_data_deque_.size() > 10) {
    new_radar_data_deque_.pop_front();
  }
  // if(is_publish_radar_obstacle_)
  // {
  //     std_msgs::String middle_fusion_radar;
  //     output_radar_measurement.SerializeToString(&middle_fusion_radar.data);
  //     camera_radar_marker_pub_.publish(middle_fusion_radar);

  //     uint32_t shape = visualization_msgs::Marker::CYLINDER;
  //     visualization_msgs::MarkerArray marker_array;

  //     for (int i = 0; i < output_radar_measurement.objs_size(); i++)
  //     {
  //         Eigen::Vector4d X;
  //         X.setIdentity();
  //         X(0, 0) =
  //         output_radar_measurement.mutable_objs(i)->mutable_obj()->center().x();
  //         X(1, 0) =
  //         output_radar_measurement.mutable_objs(i)->mutable_obj()->center().y();
  //         X(2, 0) =
  //         output_radar_measurement.mutable_objs(i)->mutable_obj()->center().z();
  //         X(3, 0) = 1;
  //         visualization_msgs::Marker marker;
  //         pubSingleFeature(marker, shape, X(0), X(1), X(2));
  //         marker_array.markers.emplace_back(marker);
  //     }
  //     camera_radar_marker_visualization_pub_.publish(marker_array);
  // }
}

void RadarSubscriber::ParseData(std::deque<perception::RadarObjects>& deque_radar_data) {
  if (new_radar_data_deque_.size() > 0) {
    deque_radar_data.insert(deque_radar_data.end(), new_radar_data_deque_.begin(),
                            new_radar_data_deque_.end());
    new_radar_data_deque_.clear();
  }
  while (deque_radar_data.size() > 10) {
    deque_radar_data.pop_front();
  }
}
}  // namespace mid_fusion
}  // namespace perception
