#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"
#include <sensor_msgs/Image.h>

namespace perception {
namespace base {

// void imdecode(const sensor_msgs::ImageConstPtr& msg, cv::Mat& img)
// {
  // double time_decode = (double)cv::getTickCount();
  // img = cv::imdecode(cv::Mat(msg->data), 1);
  // time_decode = ((double)cv::getTickCount()-time_decode)*1000/(double)cv::getTickFrequency();
  // ROS_INFO_STREAM("time_decode: " << time_decode << " ms.");
// }

class BaseCameraModel {
 public:
  virtual ~BaseCameraModel() = default;

  virtual Eigen::Vector2f Project(const Eigen::Vector3f& point3d) = 0;
  virtual Eigen::Vector3f UnProject(const Eigen::Vector2f& point2d) = 0;
  virtual std::string name() const = 0;

  inline void set_width(size_t width) { image_width_ = width; }
  inline void set_height(size_t height) { image_height_ = height; }

  inline size_t get_width() const { return image_width_; }
  inline size_t get_height() const { return image_height_; }

 protected:
  size_t image_width_ = 0;
  size_t image_height_ = 0;
};

class PinholeCameraModel : public BaseCameraModel {
 public:
  ~PinholeCameraModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;
  Eigen::Vector3f UnProject(const Eigen::Vector2f& point2d) override;
  std::string name() const override { return "PinholeCameraModel"; }

  inline void set_intrinsic_params(const Eigen::Matrix3f& params) {
    intrinsic_params_ = params;
  }

  inline Eigen::Matrix3f get_intrinsic_params() const {
    return intrinsic_params_;
  }

 protected:
  /*     fx  0   cx
         0   fy  cy
         0    0  1
  */
  Eigen::Matrix3f intrinsic_params_;
};

// TODO(all) remove later
typedef std::shared_ptr<BaseCameraModel> BaseCameraModelPtr;
typedef std::shared_ptr<const BaseCameraModel> BaseCameraModelConstPtr;
typedef std::shared_ptr<PinholeCameraModel> PinholeCameraModelPtr;
typedef std::shared_ptr<const PinholeCameraModel> PinholeCameraModelConstPtr;

}  // namespace base
}  // namespace perception
