#pragma once
#include <Eigen/Dense>

namespace perception {
namespace mid_fusion {

class GlobalToLocalTransformer {
 public:
  GlobalToLocalTransformer() {
    matrix_t_ = Eigen::MatrixXd::Zero(2, 1);
    matrix_R_ = Eigen::MatrixXd::Zero(2, 2);
    matrix_RT_ = Eigen::MatrixXd::Zero(2, 2);
    matrix_utm_ = Eigen::MatrixXd::Zero(2, 1);
    matrix_local_ = Eigen::MatrixXd::Zero(2, 1);
  }
  void update(double x, double y, double yaw) {
    matrix_t_(0, 0) = x;
    matrix_t_(1, 0) = y;
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    matrix_R_(0, 0) = matrix_RT_(0, 0) = cos_yaw;
    matrix_R_(0, 1) = matrix_RT_(1, 0) = sin_yaw;
    matrix_R_(1, 0) = matrix_RT_(0, 1) = -sin_yaw;
    matrix_R_(1, 1) = matrix_RT_(1, 1) = cos_yaw;
  }
  void transform(double &x, double &y) {
    matrix_utm_(0, 0) = x;
    matrix_utm_(1, 0) = y;
    matrix_local_ = matrix_R_ * (matrix_utm_ - matrix_t_);
    x = matrix_local_(0, 0);
    y = matrix_local_(1, 0);
  }
  void untransform(double &x, double &y) {
    matrix_local_(0, 0) = x;
    matrix_local_(1, 0) = y;
    matrix_utm_ = matrix_RT_ * matrix_local_ + matrix_t_;
    x = matrix_utm_(0, 0);
    y = matrix_utm_(1, 0);
  }

 private:
  Eigen::MatrixXd matrix_t_;
  Eigen::MatrixXd matrix_R_;   // global to local
  Eigen::MatrixXd matrix_RT_;  // local to global
  Eigen::MatrixXd matrix_utm_;
  Eigen::MatrixXd matrix_local_;
};

}  // namespace mid_fusion
}  // namespace perception
