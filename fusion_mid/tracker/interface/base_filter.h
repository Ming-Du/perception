#pragma once

#include <string>

#include "perception/fusion_mid/tracker/base/object.h"
#include "Eigen/Dense"
#include <ros/ros.h>

namespace perception {
namespace mid_fusion {
class BaseFilter {
 public:
  BaseFilter() : name_("BaseFilter") {}
  virtual ~BaseFilter() {}
  virtual void Init(const mid_fusion::Object& object) = 0;
  virtual Eigen::VectorXd Predict(double time_diff) = 0;
  virtual Eigen::VectorXd UpdateWithObject(const mid_fusion::Object& new_object,
                                           double time_diff) = 0;
  virtual void GetState(Eigen::Vector3d* anchor_point,
                        Eigen::Vector3d* velocity) = 0;
  virtual Eigen::Matrix4d GetCovarianceMatrix() = 0;
  std::string Name() { return name_; }

 protected:
  std::string name_;

};

}  // namespace mid_fusion
}  // namespace perception
