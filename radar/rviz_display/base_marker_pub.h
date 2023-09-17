#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace perception {
namespace radar {

struct Params {
  visualization_msgs::MarkerArray radar_marker_array_ptr;
  std::vector<visualization_msgs::Marker> marker_list;
  size_t max_obj_size = 0;
};


template <typename T>
std::string NumToStr(T num, int precision) {
  std::stringstream ss;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(precision);
  std::string st;
  ss << num;
  ss >> st;
  return st;
}

}  // namespace radar
}  // namespace perception 

