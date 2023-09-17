#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/proto/object.pb.h"
#include "common/proto/localization.pb.h"
#include "common/transform.h"

namespace perception {
namespace v2x {

class V2xDisplay {
public:
    void ObuRTEDisplay(const perception::ObuObjects obu_objets,
                       const localization::Localization localization,
                       ros::Publisher &publisher);

    void ObuPNTDisplay(const perception::ObuObjects obu_objets,
                       const localization::Localization localization,
                       ros::Publisher &publisher);

private:
    void AddTypeText(const perception::Object obj,
                     visualization_msgs::Marker &marker);

    void AddBboxPoint(const perception::Object obj,
                      visualization_msgs::Marker &marker);

    void PointRotate(const double cosYaw, const double sinYaw,
                     const double dcx, const double dcy,
                     geometry_msgs::Point &point);

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

    void SetPointLFD(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() - object.size().x() / 2.;
        point.y = object.center().y() - object.size().y() / 2.;
        point.z = object.center().z() - object.size().z() / 2.;
    }
    void SetPointLBD(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() - object.size().x() / 2.;
        point.y = object.center().y() + object.size().y() / 2.;
        point.z = object.center().z() - object.size().z() / 2.;
    }
    void SetPointRFD(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() + object.size().x() / 2.;
        point.y = object.center().y() - object.size().y() / 2.;
        point.z = object.center().z() - object.size().z() / 2.;
    }
    void SetPointRBD(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() + object.size().x() / 2.;
        point.y = object.center().y() + object.size().y() / 2.;
        point.z = object.center().z() - object.size().z() / 2.;
    }
    void SetPointLFU(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() - object.size().x() / 2.;
        point.y = object.center().y() - object.size().y() / 2.;
        point.z = object.center().z() + object.size().z() / 2.;
    }
    void SetPointLBU(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() - object.size().x() / 2.;
        point.y = object.center().y() + object.size().y() / 2.;
        point.z = object.center().z() + object.size().z() / 2.;
    }
    void SetPointRFU(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() + object.size().x() / 2.;
        point.y = object.center().y() - object.size().y() / 2.;
        point.z = object.center().z() + object.size().z() / 2.;
    }
    void SetPointRBU(const perception::Object &object, geometry_msgs::Point &point) {
        point.x = object.center().x() + object.size().x() / 2.;
        point.y = object.center().y() + object.size().y() / 2.;
        point.z = object.center().z() + object.size().z() / 2.;
    }

};

}  // namespace v2x
}  // namespace perception
