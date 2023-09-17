#pragma once

#include "base_pub.h"

namespace perception {
namespace fusion {
  constexpr double kDuration = 0.5;
class MarkerPubOptions : public BasePubOptions {
 public:
};

class BaseMarkerPub {
 public:
  using Ptr = std::shared_ptr<BaseMarkerPub>;

  virtual ~BaseMarkerPub() = default;

  virtual void init(const MarkerPubOptions& options) = 0;

  virtual std::vector<ROS_VISUALIZATION_MARKER>& display(const TrackedObjects& tracked_objects) = 0;

  virtual std::string name() = 0;

  std::map<perception::ObjectType, std::string> object_type_2_name_map = {
      {perception::ObjectType::TYPE_UNKNOWN, "UNKNOWN"},
      {perception::ObjectType::TYPE_PEDESTRIAN, "PED"},
      {perception::ObjectType::TYPE_BICYCLE, "BIC"},
      {perception::ObjectType::TYPE_CAR, "CAR"},
      {perception::ObjectType::TYPE_TRUCK, "TRUCK"},
      {perception::ObjectType::TYPE_BUS, "BUS"},
      {perception::ObjectType::TYPE_MOTOR, "MOTOR"},
      {perception::ObjectType::TYPE_UNKNOWN_STATIC, "UNKNOWN_STATIC"},
      {perception::ObjectType::TYPE_UNKNOWN_DYNAMIC, "UNKNOWN_DYNAMIC"},
      {perception::ObjectType::TYPE_ROADWORK_OCCUPY_0501, "TYPE_ROADWORK_OCCUPY_0501"},
      {perception::ObjectType::TYPE_ROADWORK_BREAK_0502, "TYPE_ROADWORK_BREAK_0502"},
      {perception::ObjectType::TYPE_TRIANGLEROADBLOCK, "CONE"},
      {perception::ObjectType::TYPE_VEGETATION, "VEGETATION"},
      {perception::ObjectType::TYPE_RIDER, "RIDER"},
      };

 protected:
  ros::Publisher pub_;
  MarkerPubOptions options_;

  struct Params {
    std::vector<ROS_VISUALIZATION_MARKER> marker_list;
    std::map<std::string, ROS_VISUALIZATION_MARKER::_color_type> color_kind_map;

    std::map<perception::ObjectType, std::string> class_color_map = {
        {perception::ObjectType::TYPE_UNKNOWN, _Lavender},
        {perception::ObjectType::TYPE_PEDESTRIAN, _Yellow},
        {perception::ObjectType::TYPE_BICYCLE, _Green},
        {perception::ObjectType::TYPE_CAR, _CyanBlue},
        {perception::ObjectType::TYPE_TRUCK, _CyanBlue},
        {perception::ObjectType::TYPE_BUS, _CyanBlue},
        {perception::ObjectType::TYPE_VEGETATION, _DeepPink},
    };

    ROS_VISUALIZATION_MARKER::_color_type default_color_type;
    ROS_VISUALIZATION_MARKER::_scale_type default_scale_type;
    size_t max_obj_size = 1;

    Params() {
      marker_list.resize(1);
      color_kind_map.clear();
      ROS_VISUALIZATION_MARKER::_color_type tmp;
      //淡紫色
      tmp.r = 0.6;
      tmp.g = 0.5;
      tmp.b = 0.6;
      color_kind_map[_Lavender] = tmp;
      //黄色
      tmp.r = 1.;
      tmp.g = 1.;
      tmp.b = 0.;
      color_kind_map[_Yellow] = tmp;
      //青色
      tmp.r = 0.;
      tmp.g = 1.;
      tmp.b = 1.;
      color_kind_map[_CyanBlue] = tmp;
      //红色
      tmp.r = 0.7;
      tmp.g = 0.;
      tmp.b = 0.;
      color_kind_map[_Red] = tmp;
      //蓝色
      tmp.r = 0.;
      tmp.g = 0.;
      tmp.b = 1.;
      color_kind_map[_Blue] = tmp;
      //绿色
      tmp.r = 0.;
      tmp.g = 1.;
      tmp.b = 0.;
      color_kind_map[_Green] = tmp;
      //深粉色
      tmp.r = 255.0 / 255.0;;
      tmp.g = 20.0 / 255.0;
      tmp.b = 147.0 / 255.0;
      color_kind_map[_DeepPink] = tmp;

      default_color_type.r = 0;
      default_color_type.g = 0;
      default_color_type.b = 0;
      default_color_type.a = 0;

      default_scale_type.x = 1.0;
      default_scale_type.y = 1.0;
      default_scale_type.z = 1.0;
    }
  } params_;

  ROS_VISUALIZATION_MARKER::_color_type getClassColor(const perception::Object& obj) {
    if (params_.class_color_map.find(obj.type()) == params_.class_color_map.end()) {
      return params_.color_kind_map[_Lavender];
    } else {
      return params_.color_kind_map[params_.class_color_map.at(obj.type())];
    }
  }
  void drawText(const Eigen::Vector3d& pos,
                const std::string& info,
                ROS_VISUALIZATION_MARKER& marker,
                double alpha, double scale) {
    marker.type = ROS_VISUALIZATION_MARKER::TEXT_VIEW_FACING;
    marker.action = ROS_VISUALIZATION_MARKER::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
    tf::quaternionTFToMsg(quat, marker.pose.orientation);
    marker.color.a = alpha;
    marker.text = info;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
  }
  template <typename T>
  inline std::string num2str(T num, int precision) {
    std::stringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(precision);
    std::string st;
    ss << num;
    ss >> st;

    return st;
  }
};

}  // namespace fusion
}  // namespace perception
