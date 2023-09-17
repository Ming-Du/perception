#pragma once

#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include "common/proto/object.pb.h"

namespace perception {
namespace fusion {
using ROS_VISUALIZATION_MARKER = visualization_msgs::Marker;
using ROS_VISUALIZATION_MARKERARRAY = visualization_msgs::MarkerArray;
using ROS_GEOMETRY_POINT = geometry_msgs::Point;

static const char _Lavender[] = "Lavender";
static const char _Yellow[] = "Yellow";
static const char _CyanBlue[] = "CyanBlue";
static const char _Red[] = "Red";
static const char _Blue[] = "Blue";
static const char _Green[] = "Green";
static const char _DeepPink[] = "DeepPink";

/**
 * Base Pub Options
 */
class BasePubOptions {
 public:
  std::string pre_fix = "";
  std::string frame_id = "base_link";
  ros::NodeHandlePtr node_ptr = nullptr;
};

struct RotateBox {
  using Ptr = std::shared_ptr<RotateBox>;

  RotateBox() { heading = Eigen::Vector3d(0., 0., 0.); }

  RotateBox(const Eigen::Vector3d& center,
            const Eigen::Vector3d& size,
            const Eigen::Vector3d& dir) {
    this->center = center;
    this->size = size;
    this->heading = dir;
    this->heading.normalize();
    this->angle = std::atan2(this->heading.y(), this->heading.x());
  }

  RotateBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const double& ang) {
    this->center = center;
    this->size = size;
    this->angle = normalizeAngle(ang);
    this->heading[0] = std::cos(this->angle);
    this->heading[1] = std::sin(this->angle);
    this->heading[2] = 0;
  }

  double area() const { return this->size.x() * this->size.y(); }

  double volume() const { return this->size.x() * this->size.y() * this->size.z(); }

  // get 8 corners from a box
  void corners(std::vector<Eigen::Vector3d>& corners) const {
    corners.resize(8, Eigen::Vector3d(0, 0, 0));

    auto dir = this->heading;
    dir.z() = 0.f;
    dir.normalize();
    Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
    Eigen::Vector3d z_dir = Eigen::Vector3d(0, 0, 1);

    const auto& center = this->center;
    const auto& size = this->size;

    corners[0] =
        center + dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
    corners[1] =
        center - dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
    corners[2] =
        center - dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
    corners[3] =
        center + dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;

    corners[4] =
        center + dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
    corners[5] =
        center - dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
    corners[6] =
        center - dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
    corners[7] =
        center + dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
  }

  // get the nearest corner from a box
  Eigen::Vector3d anchor() const {
    std::vector<Eigen::Vector3d> corners;
    this->corners(corners);

    double min_dist = 1e9;
    int idx = -1;
    for (int i = 0; i < 4; ++i) {
      double dist = corners[i].norm();
      if (dist < min_dist) {
        min_dist = dist;
        idx = i;
      }
    }

    return corners[idx];
  }

  double normalizeAngle(const double& a) {
    double b = a;
    while (b < -M_PI) {
      b += 2 * M_PI;
    }
    while (b > M_PI) {
      b -= 2 * M_PI;
    }
    return b;
  }

  //    bool isInside(const Eigen::Vector3d point) const{
  //        std::vector<Eigen::Vector3d> corners;
  //        this->corners(corners);
  //
  //        double vertx[4], verty[4];
  //        for (int k = 0; k < 4; ++k) {
  //            vertx[k] = corners[k].x();
  //            verty[k] = corners[k].y();
  //        }
  //
  //        if (inPolygon(4, vertx, verty, point.x(), point.y()) && point.z() > corners[0].z() &&
  //            point.z() < corners[6].z()) {
  //            return true;
  //        }
  //
  //        return false;
  //    }

  Eigen::Vector3d size, center, heading;
  double angle = 0.f;
};

}  // namespace fusion
}  // namespace perception
